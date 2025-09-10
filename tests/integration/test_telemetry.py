"""
Test the telemetry worker with a mocked drone.
"""

import multiprocessing as mp
import subprocess
import threading

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller


MOCK_DRONE_MODULE = "tests.integration.mock_drones.telemetry_drone"
CONNECTION_STRING = "tcp:localhost:12345"

# Please do not modify these, these are for the test cases (but do take note of them!)
TELEMETRY_PERIOD = 1
NUM_TRIALS = 5
NUM_FAILS = 3

# =================================================================================================
#                         ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Add your own constants here
TEST_DURATION = 15  # Adjust as needed
# =================================================================================================
#                         ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


# Same utility functions across all the integration tests
# pylint: disable=duplicate-code
def start_drone() -> None:
    """
    Start the mocked drone.
    """
    subprocess.run(["python", "-m", MOCK_DRONE_MODULE], shell=True, check=False)


# =================================================================================================
#                         ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def stop(
    args,
) -> None:
    """
    Stop the workers.
    """
    if args:
        args[0].put("stop")


def read_queue(
    args,
    main_logger: logger.Logger,
) -> None:
    """
    Read and print the output queue.
    """
    while True:
        try:
            # This will block until an item is available
            item = args[0].get(timeout=1)
            if item == "stop":
                break
            main_logger.info(f"Telemetry worker output: {item}")
        except mp.queues.Empty:
            pass


# =================================================================================================
#                         ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Start the telemetry worker simulation.
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

    # Mocked GCS, connect to mocked drone which is listening at CONNECTION_STRING
    # source_system = 255 (groundside)
    # source_component = 0 (ground control station)
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )
    main_logger.info("Connected!")
    # pylint: enable=duplicate-code

    # =============================================================================================
    #                         ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Mock starting a worker, since cannot actually start a new process
    # Create a worker controller for your worker
    telemetry_worker_controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    manager = mp.Manager()

    # Create your queues
    telemetry_output_queue = queue_proxy_wrapper.QueueProxyWrapper(manager.Queue())

    # Just set a timer to stop the worker after a while, since the worker infinite loops
    threading.Timer(TEST_DURATION, stop, ([telemetry_worker_controller],)).start()

    # Read the main queue (worker outputs)
    threading.Thread(target=read_queue, args=([telemetry_output_queue], main_logger)).start()

    telemetry_worker.telemetry_worker(
        # Put your own arguments here
        controller=telemetry_worker_controller,
        connection=connection,
        output_queue=telemetry_output_queue,
    )
    # =============================================================================================
    #                         ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    # Start drone in another process
    drone_process = mp.Process(target=start_drone)
    drone_process.start()

    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")

    drone_process.join()
