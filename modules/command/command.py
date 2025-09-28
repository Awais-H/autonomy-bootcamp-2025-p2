"""
Decision-making logic.
"""

import math

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

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
        altitude_threshold: float = 0.5,
        yaw_threshold_deg: float = 5.0,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            return True, cls(
                cls.__private_key,
                connection,
                target,
                local_logger,
                altitude_threshold,
                yaw_threshold_deg,
            )
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create Command object: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
        altitude_threshold: float = 0.5,
        yaw_threshold_deg: float = 5.0,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"
        self._connection = connection
        self._target = target
        self._local_logger = local_logger
        self._altitude_threshold = altitude_threshold
        self._yaw_threshold_deg = yaw_threshold_deg
        self._input_count = 0
        self._x_velocity = 0
        self._y_velocity = 0
        self._z_velocity = 0

    def run(self, telemetry_data: telemetry.TelemetryData) -> str | None:
        """
        Make a decision based on received telemetry data.
        """
        self._input_count += 1
        self._x_velocity += telemetry_data.x_velocity
        self._y_velocity += telemetry_data.y_velocity
        self._z_velocity += telemetry_data.z_velocity
        avg_velo = (
            self._x_velocity / self._input_count,
            self._y_velocity / self._input_count,
            self._z_velocity / self._input_count,
        )

        self._local_logger.info(f"Average velocity: {avg_velo}")

        position_error_x = self._target.x - telemetry_data.x
        position_error_y = self._target.y - telemetry_data.y
        position_error_z = self._target.z - telemetry_data.z
        if abs(position_error_z) > self._altitude_threshold:
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
                return f"CHANGE_ALTITUDE: {position_error_z:.2f}"
            except (OSError, mavutil.mavlink.MAVError) as e:
                self._local_logger.error(f"Failed to send MAV_CMD_CONDITION_CHANGE_ALT: {e}")

        target_yaw_rad = math.atan2(position_error_y, position_error_x)
        current_yaw_rad = telemetry_data.yaw

        delta_yaw_rad = target_yaw_rad - current_yaw_rad
        delta_yaw_rad = (delta_yaw_rad + math.pi) % (2 * math.pi) - math.pi
        delta_yaw_deg = math.degrees(delta_yaw_rad)

        if abs(delta_yaw_deg) > self._yaw_threshold_deg:
            if delta_yaw_deg > 0:
                direction = -1
            else:
                direction = 1

            try:
                self._connection.mav.command_long_send(
                    target_system=1,
                    target_component=0,
                    command=mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    confirmation=0,
                    param1=delta_yaw_deg,
                    param2=5,
                    param3=direction,
                    param4=1,
                    param5=0,
                    param6=0,
                    param7=0,
                )
                return f"CHANGING_YAW: {delta_yaw_deg:.2f}"
            except (OSError, mavutil.mavlink.MAVError) as e:
                self._local_logger.error(f"Failed to send MAV_CMD_CONDITION_YAW: {e}")

        return None


# =================================================================================================
#           ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
