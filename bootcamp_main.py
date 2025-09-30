"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

import multiprocessing as mp
import queue
import time

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
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
HEARTBEAT_RECEIVER_QUEUE_SIZE = 10
TELEMETRY_QUEUE_MAXSIZE = 10
COMMAND_QUEUE_MAXSIZE = 10
HEARTBEAT_QUEUE_MAXSIZE = 10

# Set worker counts
NUM_HEARTBEAT_SENDERS = 1
NUM_HEARTBEAT_RECEIVER = 1
NUM_TELEMETRY = 1
NUM_COMMAND = 1

# Heartbeat period
HEARTBEAT_PERIOD = 1.0

# Any other constants
TARGET = command.Position(10, 10, 10)

# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Main function.
    """
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
    receiver_queue = queue_proxy_wrapper.QueueProxyWrapper(manager, TELEMETRY_QUEUE_MAXSIZE)
    telemetry_queue = queue_proxy_wrapper.QueueProxyWrapper(manager, TELEMETRY_QUEUE_MAXSIZE)
    command_queue = queue_proxy_wrapper.QueueProxyWrapper(manager, COMMAND_QUEUE_MAXSIZE)

    # Create worker properties for each worker type (what inputs it takes, how many workers)
    # Heartbeat sender
    result, hb_send_props = worker_manager.WorkerProperties.create(
        count=NUM_HEARTBEAT_SENDERS,
        target=heartbeat_sender_worker.heartbeat_sender_worker,
        work_arguments=(connection, HEARTBEAT_PERIOD),
        input_queues=[],
        output_queues=[],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        main_logger.error("Sender worker failed")
        return -1

    # Heartbeat receiver
    result, heartbeat_receiver_worker_prop = worker_manager.WorkerProperties.create(
        target=heartbeat_receiver_worker.heartbeat_receiver_worker,
        count=NUM_HEARTBEAT_RECEIVER,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[receiver_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        main_logger.error("Receiver worker failed")
        return -1

    # Telemetry
    result, telemetry_worker_prop = worker_manager.WorkerProperties.create(
        target=telemetry_worker.telemetry_worker,
        count=NUM_TELEMETRY,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[telemetry_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        main_logger.error("Telemetry worker failed")
        return -1

    # Command
    result, command_worker_prop = worker_manager.WorkerProperties.create(
        target=command_worker.command_worker,
        count=NUM_COMMAND,
        work_arguments=(connection, TARGET),
        input_queues=[telemetry_queue],
        output_queues=[command_queue],
        controller=main_controller,
        local_logger=main_logger,
    )
    if not result:
        main_logger.error("Command worker failed")
        return -1

    assert hb_send_props is not None
    assert heartbeat_receiver_worker_prop is not None
    assert telemetry_worker_prop is not None
    assert command_worker_prop is not None

    # Create the workers (processes) and obtain their managers
    all_worker_properties_list = [
        hb_send_props,
        heartbeat_receiver_worker_prop,
        telemetry_worker_prop,
        command_worker_prop,
    ]

    result, main_worker_manager = worker_manager.WorkerManager.create(
        worker_properties=all_worker_properties_list,
        local_logger=main_logger,
    )

    if not result:
        return -1
    assert main_worker_manager is not None

    # Start worker processes
    main_worker_manager.start_workers()
    main_logger.info("Started workers")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for 100 seconds or until the drone disconnects
    start_time = time.time()
    queues = [receiver_queue, telemetry_queue, command_queue]

    while time.time() - start_time < 100 and connection.target_system != 0:
        for output in queues:
            try:
                msg = output.queue.get_nowait()
                main_logger.info(f"Received message: {msg}")

                if msg == "Disconnected":
                    break

            except queue.Empty:
                continue
        time.sleep(1)

    # Stop the processes
    main_controller.request_exit()
    main_logger.info("Requested exit")

    # Fill and drain queues from END TO START
    receiver_queue.fill_and_drain_queue()
    telemetry_queue.fill_and_drain_queue()
    command_queue.fill_and_drain_queue()
    main_logger.info("Queues cleared")

    # Clean up worker processes
    main_worker_manager.join_workers()
    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance
    main_controller = worker_controller.WorkerController()
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
