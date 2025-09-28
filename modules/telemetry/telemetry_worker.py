"""
Telemetry worker that gathers GPS data.
"""

import os
import pathlib
import time

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from modules.common.modules.logger import logger
from . import telemetry


# =================================================================================================
#           ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection is the MAVLink connection to the drone.
    output_queue is the data queue to pass TelemetryData to.
    controller is how the main process communicates to this worker process.
    """
    # =============================================================================================
    #           ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
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
    #           ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (telemetry.Telemetry)
    result, telemetry_instance = telemetry.Telemetry.create(connection, local_logger)
    if not result:
        local_logger.error("Failed to create telemetry instance.")
        return

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            telemetry_data = telemetry_instance.run()
            if telemetry_data:
                local_logger.info("Telemetry data received and processed.")
                output_queue.queue.put(telemetry_data)
            else:
                local_logger.warning(
                    "Failed to receive telemetry data - timeout or missing messages"
                )
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Error in telemetry worker loop: {e}")

        time.sleep(0.01)

    local_logger.info("Worker has been terminated.", True)


# =================================================================================================
#           ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
