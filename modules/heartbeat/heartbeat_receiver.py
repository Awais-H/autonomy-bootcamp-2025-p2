"""
Heartbeat receiving logic.
"""

import time
from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    max_threshold = 3

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ):
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        try:
            return True, cls(cls.__private_key, connection, local_logger)
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(
                f"Failed to create HeartbeatReceiver due to MAVLink/OS error: {e}"
            )
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"
        self._connection = connection
        self._local_logger = local_logger
        self._last_heartbeat_time = time.time()
        self._missing_count = 0
        self._status = "Disconnected"

    def run(self):
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        try:
            msg = self._connection.recv_match(
                type="HEARTBEAT", blocking=False, timeout=1.0
            )

            if msg:
                self._last_heartbeat_time = time.time()
                self._missing_count = 0
                self._status = "Connected"
                self._local_logger.info("Connection status: Connected")

            else:
                time_since_last_heartbeat = time.time() - self._last_heartbeat_time
                if time_since_last_heartbeat >= 1.0:
                    self._missing_count += 1
                    self._local_logger.warning(
                        f"Missed a heartbeat. Count: {self._missing_count}"
                    )

            if self._missing_count >= self.max_threshold:
                if self._status != "Disconnected":
                    self._status = "Disconnected"
                    self._local_logger.error(
                        f"Connection status: Disconnected. Heart beats went past {self.max_threshold} heartbeats."
                    )

        except (OSError, mavutil.mavlink.MAVError) as e:
            self._local_logger.error(f"Error in HeartbeatReceiver.run: {e}")

        return self._status


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
