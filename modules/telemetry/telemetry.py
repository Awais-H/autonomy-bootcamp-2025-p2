"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


# pylint: disable=too-many-instance-attributes, too-many-arguments
class TelemetryData:
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Telemetry:  # pylint: disable=too-many-instance-attributes
    """
    Telemetry class to read position and attitude (orientation).
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, Telemetry] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.
        """
        try:
            return True, cls(cls.__private_key, connection, local_logger)
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create Command object due to MAVLink/OS error: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"
        # Do any initialization here
        self._connection = connection
        self._local_logger = local_logger
        self._last_position = None
        self._last_attitude = None

    def run(
        self,
    ) -> TelemetryData | None:
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.
        """
        start_time = time.time()

        # Set to smaller than 1.0, as 1.0s is the timeout
        while start_time < 1.0:
            msg = self._connection.recv_match(
                type=["ATTITUDE", "LOCAL_POSITION_NED"], blocking=False, timeout=0.0
            )
            if msg:
                if msg.get_type() == "ATTITUDE":
                    self._last_attitude = msg
                elif msg.get_type() == "LOCAL_POSITION_NED":
                    self._last_position = msg

            if self._last_position and self._last_attitude:
                telemetry_data = TelemetryData()

                if self._last_position.time_boot_ms > self._last_attitude.time_boot_ms:
                    telemetry_data.time_since_boot = self._last_position.time_boot_ms
                else:
                    telemetry_data.time_since_boot = self._last_attitude.time_boot_ms

                telemetry_data.x = self._last_position.x
                telemetry_data.y = self._last_position.y
                telemetry_data.z = self._last_position.z
                telemetry_data.x_velocity = self._last_position.vx
                telemetry_data.y_velocity = self._last_position.vy
                telemetry_data.z_velocity = self._last_position.vz

                telemetry_data.roll = self._last_attitude.roll
                telemetry_data.pitch = self._last_attitude.pitch
                telemetry_data.yaw = self._last_attitude.yaw
                telemetry_data.roll_speed = self._last_attitude.rollspeed
                telemetry_data.pitch_speed = self._last_attitude.pitchspeed
                telemetry_data.yaw_speed = self._last_attitude.yawspeed

                self._last_position = None
                self._last_attitude = None

                return telemetry_data

        return None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
