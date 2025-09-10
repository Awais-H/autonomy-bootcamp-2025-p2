"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

import multiprocessing as mp
import queue
import time
import shutil
import pathlib

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.command import command
from modules.command import command_worker
from modules.heartbeat import heartbeat_receiver_worker
from modules.heartbeat import heartbeat_sender_worker
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from utilities.workers import worker_manager


# MAVLink connection
CONNECTION_STRING = "tcp:localhost:12345"


# =================================================================================================
#                         ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
HEARTBEAT_SENDER_OUTPUT_QUEUE_MAX_SIZE = 10
HEARTBEAT_RECEIVER_OUTPUT_QUEUE_MAX_SIZE = 10
TELEMETRY_OUTPUT_QUEUE_MAX_SIZE = 10
COMMAND_OUTPUT_QUEUE_MAX_SIZE = 10

# Set worker counts
HEARTBEAT_SENDER_WORKER_COUNT = 1
HEARTBEAT_RECEIVER_WORKER_COUNT = 1
TELEMETRY_WORKER_COUNT = 1
COMMAND_WORKER_COUNT = 1

# Any other constants
TARGET = command.Position(10, 20, 30)
MAIN_PROCESS_RUN_TIME = 100
# =================================================================================================
#                         ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def clean_logs() -> None:
    """
    Deletes all files and folders in the logs directory.
    """
    log_dir = pathlib.Path(__file__).parent.parent.parent / "logs"
    if log_dir.exists() and log_dir.is_dir():
        for item in log_dir.iterdir():
            if item.is_dir():
                shutil.rmtree(item)
            else:
                item.unlink()
    else:
        log_dir.mkdir(parents=True, exist_ok=True)


def main() -> int:
    """
    Main function.
    """
    # Clean up old logs for a fresh start
    clean_logs()

    # Configuration settings
    result, config = read_yaml.open_config(logger.CONFIG_FILE_PATH)
    if not result:
        print("ERROR: Failed to load configuration file")
        return -1

    # Get Pylance to stop complaining
    assert config is not None

    # Setup main logger
    result, main_logger, _ = logger_main_setup.setup_main_logger(config)
    if not result:
        print("ERROR: Failed to create main logger")
        return -1

    # Get Pylance to stop complaining
    assert main_logger is not None

    # Create a connection to the drone. Assume that this is safe to pass around to all processes
    # In reality, this will not work, but to simplify the bootamp, preetend it is allowed
    # To test, you will run each of your workers individually to see if they work
    # (test "drones" are provided for you test your workers)
    # NOTE: If you want to have type annotations for the connection, it is of type mavutil.mavfile
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.wait_heartbeat(timeout=30)  # Wait for the "drone" to connect

    # =============================================================================================
    #                         ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Create a worker controller
    main_controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    manager = mp.Manager()

    # Create queues
    heartbeat_sender_output_queue = queue_proxy_wrapper.QueueProxyWrapper(
        manager, HEARTBEAT_SENDER_OUTPUT_QUEUE_MAX_SIZE
    )
    heartbeat_receiver_output_queue = queue_proxy_wrapper.QueueProxyWrapper(
        manager, HEARTBEAT_RECEIVER_OUTPUT_QUEUE_MAX_SIZE
    )
    telemetry_output_queue = queue_proxy_wrapper.QueueProxyWrapper(
        manager, TELEMETRY_OUTPUT_QUEUE_MAX_SIZE
    )
    command_output_queue = queue_proxy_wrapper.QueueProxyWrapper(
        manager, COMMAND_OUTPUT_QUEUE_MAX_SIZE
    )

    # Create worker properties for each worker type (what inputs it takes, how many workers)
    # Heartbeat sender
    result, heartbeat_sender_worker_properties = worker_manager.WorkerProperties.create(
        target=heartbeat_sender_worker.heartbeat_sender_worker,
        count=HEARTBEAT_SENDER_WORKER_COUNT,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[heartbeat_sender_output_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        return -1

    # Heartbeat receiver
    result, heartbeat_receiver_worker_properties = (
        worker_manager.WorkerProperties.create(
            target=heartbeat_receiver_worker.heartbeat_receiver_worker,
            count=HEARTBEAT_RECEIVER_WORKER_COUNT,
            work_arguments=(connection,),
            input_queues=[],
            output_queues=[heartbeat_receiver_output_queue],
            controller=main_controller,
            local_logger=main_logger,
        )
    )
    if not result:
        return -1

    # Telemetry
    result, telemetry_worker_properties = worker_manager.WorkerProperties.create(
        target=telemetry_worker.telemetry_worker,
        count=TELEMETRY_WORKER_COUNT,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[telemetry_output_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        return -1

    # Command
    result, command_worker_properties = worker_manager.WorkerProperties.create(
        target=command_worker.command_worker,
        count=COMMAND_WORKER_COUNT,
        work_arguments=(connection, TARGET),
        input_queues=[telemetry_output_queue],
        output_queues=[command_output_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        return -1

    # Get pylance to stop complaining
    assert heartbeat_sender_worker_properties is not None
    assert heartbeat_receiver_worker_properties is not None
    assert telemetry_worker_properties is not None
    assert command_worker_properties is not None

    # Create the workers (processes) and obtain their managers
    worker_properties_list = [
        heartbeat_sender_worker_properties,
        heartbeat_receiver_worker_properties,
        telemetry_worker_properties,
        command_worker_properties,
    ]
    result, main_worker_manager = worker_manager.WorkerManager.create(
        worker_properties=worker_properties_list,
        local_logger=main_logger,
    )
    if not result:
        return -1

    # Get pylance to stop complaining
    assert main_worker_manager is not None

    # Start worker processes
    main_worker_manager.start_workers()

    main_logger.info("Started")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for 100 seconds or until the drone disconnects
    start_time = time.time()
    all_queues = [
        heartbeat_sender_output_queue,
        heartbeat_receiver_output_queue,
        telemetry_output_queue,
        command_output_queue,
    ]
    while (
        time.time() - start_time
    ) < MAIN_PROCESS_RUN_TIME and connection.target_system != 0:
        for output_queue in all_queues:
            while True:
                try:
                    message = output_queue.queue.get_nowait()
                    main_logger.info(f"Main received: {message}")
                except queue.Empty:
                    break
        time.sleep(1)

    # Stop the processes
    main_worker_manager.request_exit_all()
    main_logger.info("Requested exit")

    # Fill and drain queues from END TO START
    for output_queue in all_queues:
        while not output_queue.queue.empty():
            output_queue.queue.get()
    main_logger.info("Queues cleared")

    # Clean up worker processes
    main_worker_manager.join_all()
    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance
    main_controller.reset()
    # =============================================================================================
    #                         ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")
