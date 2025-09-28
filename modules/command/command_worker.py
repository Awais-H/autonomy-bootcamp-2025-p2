"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#     v BOOTCAMPERS MODIFY BELOW THIS COMMENT v
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
    altitude_threshold: float = 0.5,
    yaw_threshold_deg: float = 5.0,
) -> None:
    """
    Worker process.

    args... describe what the arguments are
    """
    # =============================================================================================
    #     ^ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ^
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #     v BOOTCAMPERS MODIFY BELOW THIS COMMENT v
    # =================================================================================================

    # Instantiate class object (command.Command)
    result, command_instance = command.Command.create(connection, target, local_logger, altitude_threshold, yaw_threshold_deg)
    if not result:
        local_logger.error("Failed to create Command instance.")
        return

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            telemetry_data = input_queue.queue.get()

            command_string = command_instance.run(telemetry_data)

            if command_string:
                output_queue.queue.put(command_string)
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Error in command worker loop: {e}")

    local_logger.info("Command worker finished successfully.")


# =================================================================================================
#     ^ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ^
# =================================================================================================
