"""
Decision-making logic.
"""

import math
import time

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#           ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    # Thresholds for altitude and yaw adjustments
    ALTITUDE_THRESHOLD = 0.5  # meters
    YAW_THRESHOLD_DEG = 5.0  # degrees

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            return True, cls(cls.__private_key, connection, target, local_logger)
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create Command object: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"
        self._connection = connection
        self._target = target
        self._local_logger = local_logger
        self._previous_telemetry_data = None
        self._start_position = None  # Add this to store the start position
        self._start_time = time.time()

    def run(self, telemetry_data: telemetry.TelemetryData) -> str | None:
        """
        Make a decision based on received telemetry data.
        """
        if self._start_position is None:
            self._start_position = Position(telemetry_data.x, telemetry_data.y, telemetry_data.z)

        # Calculate average velocity
        if self._previous_telemetry_data:
            elapsed_time = time.time() - self._start_time
            if elapsed_time > 0:
                delta_x = telemetry_data.x - self._start_position.x
                delta_y = telemetry_data.y - self._start_position.y
                delta_z = telemetry_data.z - self._start_position.z

                # Calculate the magnitude of the displacement vector
                total_displacement_magnitude = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

                # Calculate average velocity magnitude
                average_velocity_magnitude = total_displacement_magnitude / elapsed_time
                self._local_logger.info(
                    f"Average velocity so far: {average_velocity_magnitude:.2f} m/s"
                )

        self._previous_telemetry_data = telemetry_data

        # Adjust altitude if off by more than 0.5m
        delta_altitude = self._target.z - telemetry_data.z
        if abs(delta_altitude) > self.ALTITUDE_THRESHOLD:
            try:
                self._connection.mav.command_long_send(
                    target_system=1,
                    target_component=0,
                    command=mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                    confirmation=0,
                    param1=1,
                    param2=0,
                    param3=0,
                    param4=0,
                    param5=0,
                    param6=0,
                    param7=self._target.z,
                )
                return f"CHANGE ALTITUDE: {delta_altitude:.2f}"
            except (OSError, mavutil.mavlink.MAVError) as e:
                self._local_logger.error(f"Failed to send MAV_CMD_CONDITION_CHANGE_ALT: {e}")

        # Adjust yaw if off by more than 5 degrees
        target_yaw_rad = math.atan2(
            self._target.y - telemetry_data.y, self._target.x - telemetry_data.x
        )
        current_yaw_rad = telemetry_data.yaw

        delta_yaw_rad = target_yaw_rad - current_yaw_rad
        # Normalize the angle to the range [-pi, pi]
        delta_yaw_rad = math.atan2(math.sin(delta_yaw_rad), math.cos(delta_yaw_rad))
        delta_yaw_deg = math.degrees(delta_yaw_rad)

        if abs(delta_yaw_deg) > self.YAW_THRESHOLD_DEG:
            try:
                self._connection.mav.command_long_send(
                    target_system=1,
                    target_component=0,
                    command=mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    confirmation=0,
                    param1=delta_yaw_deg,
                    param2=5,
                    param3=1,
                    param4=1,
                    param5=0,
                    param6=0,
                    param7=0,
                )
                return f"CHANGE YAW: {delta_yaw_deg:.2f}"
            except (OSError, mavutil.mavlink.MAVError) as e:
                self._local_logger.error(f"Failed to send MAV_CMD_CONDITION_YAW: {e}")

        # If no commands were sent, return None
        return None


# =================================================================================================
#           ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
