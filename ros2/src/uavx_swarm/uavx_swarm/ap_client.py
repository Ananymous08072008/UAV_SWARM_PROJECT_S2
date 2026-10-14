"""ArduPilot DDS interface for one vehicle (/ap/v<SYSID>/...), as a companion computer sees it.

Everything here goes through ArduPilot's native ROS 2 (XRCE-DDS) interface - the same
interface a Raspberry Pi / Jetson would use over a serial link on the real aircraft.
All calls are non-blocking: service calls return a Future that the caller polls from
its own timer, so one node can drive a state machine without spinning inside callbacks.

World position: ArduPilot's EKF pose (/pose/filtered) is relative to its EKF origin,
which each vehicle sets at its own pad. While the vehicle is disarmed on the ground we
learn the offset between that frame and the shared world frame from the autopilot's GPS
(/navsat -> latitude/longitude -> ENU about CMAC), then freeze it on arming.
"""
from __future__ import annotations

import math
from typing import Optional

from ardupilot_msgs.msg import GlobalPosition, Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, NavSatFix
from std_srvs.srv import Trigger

from .common import (COMMAND_QOS, SENSOR_QOS, enu_to_latlon, latlon_to_enu,
                     yaw_from_quaternion)

COPTER_MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5,
                "RTL": 6, "CIRCLE": 7, "LAND": 9, "BRAKE": 17, "SMART_RTL": 21}
MODE_NAMES = {v: k for k, v in COPTER_MODES.items()}


class ArduPilotClient:
    def __init__(self, node, sysid: int, callback_group=None):
        self.node = node
        self.sysid = int(sysid)
        self.ns = f"/ap/v{self.sysid}"
        cbg = callback_group

        self.status: Optional[Status] = None
        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.navsat: Optional[NavSatFix] = None
        self.battery: Optional[BatteryState] = None
        self.last_status_s = -1.0
        self.last_pose_s = -1.0
        self.last_navsat_s = -1.0
        self._offset: Optional[list[float]] = None   # world ENU - EKF local (east, north)
        self._offset_frozen = False

        node.create_subscription(Status, f"{self.ns}/status", self._on_status, SENSOR_QOS, callback_group=cbg)
        node.create_subscription(PoseStamped, f"{self.ns}/pose/filtered", self._on_pose, SENSOR_QOS, callback_group=cbg)
        node.create_subscription(TwistStamped, f"{self.ns}/twist/filtered", self._on_twist, SENSOR_QOS, callback_group=cbg)
        node.create_subscription(NavSatFix, f"{self.ns}/navsat", self._on_navsat, SENSOR_QOS, callback_group=cbg)
        node.create_subscription(BatteryState, f"{self.ns}/battery", self._on_battery, SENSOR_QOS, callback_group=cbg)

        self._mode_cli = node.create_client(ModeSwitch, f"{self.ns}/mode_switch", callback_group=cbg)
        self._arm_cli = node.create_client(ArmMotors, f"{self.ns}/arm_motors", callback_group=cbg)
        self._takeoff_cli = node.create_client(Takeoff, f"{self.ns}/experimental/takeoff", callback_group=cbg)
        self._prearm_cli = node.create_client(Trigger, f"{self.ns}/prearm_check", callback_group=cbg)
        self._getparam_cli = node.create_client(GetParameters, f"{self.ns}/get_parameters", callback_group=cbg)

        self._gps_pub = node.create_publisher(GlobalPosition, f"{self.ns}/cmd_gps_pose", COMMAND_QOS)
        self._vel_pub = node.create_publisher(TwistStamped, f"{self.ns}/cmd_vel", COMMAND_QOS)

    # ------------------------------------------------------------ callbacks
    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg: Status) -> None:
        self.status = msg
        self.last_status_s = self._now()
        if msg.armed:
            self._offset_frozen = self._offset is not None

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose = msg
        self.last_pose_s = self._now()

    def _on_twist(self, msg: TwistStamped) -> None:
        self.twist = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self.battery = msg

    def _on_navsat(self, msg: NavSatFix) -> None:
        self.navsat = msg
        self.last_navsat_s = self._now()
        if self._offset_frozen or self.pose is None or msg.status.status < 0:
            return
        gps_e, gps_n = latlon_to_enu(msg.latitude, msg.longitude)
        sample = [gps_e - self.pose.pose.position.x, gps_n - self.pose.pose.position.y]
        if self._offset is None:
            self._offset = sample
        else:   # average out GPS noise while sitting on the pad
            self._offset = [0.9 * o + 0.1 * s for o, s in zip(self._offset, sample)]

    # ------------------------------------------------------------ state
    @property
    def connected(self) -> bool:
        return self.status is not None and (self._now() - self.last_status_s) < 3.0

    @property
    def armed(self) -> bool:
        return bool(self.status and self.status.armed)

    @property
    def flying(self) -> bool:
        return bool(self.status and self.status.flying)

    @property
    def mode(self) -> int:
        return self.status.mode if self.status else -1

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, str(self.mode))

    @property
    def has_position(self) -> bool:
        return self.pose is not None and self._offset is not None

    @property
    def position(self) -> tuple[float, float, float]:
        """World ENU position (m) from the EKF, height above the pad as z."""
        p = self.pose.pose.position
        return (p.x + self._offset[0], p.y + self._offset[1], p.z)

    @property
    def gps_position(self) -> Optional[tuple[float, float]]:
        if self.navsat is None:
            return None
        return latlon_to_enu(self.navsat.latitude, self.navsat.longitude)

    @property
    def yaw(self) -> float:
        return yaw_from_quaternion(self.pose.pose.orientation) if self.pose else 0.0

    @property
    def ground_speed(self) -> float:
        if self.twist is None:
            return 0.0
        v = self.twist.twist.linear
        return math.hypot(v.x, v.y)

    @property
    def vertical_speed(self) -> float:
        return self.twist.twist.linear.z if self.twist else 0.0

    # ------------------------------------------------------------ commands
    def services_ready(self) -> bool:
        return all(c.service_is_ready() for c in (self._mode_cli, self._arm_cli, self._takeoff_cli))

    def set_mode(self, mode: str | int):
        req = ModeSwitch.Request()
        req.mode = COPTER_MODES[mode] if isinstance(mode, str) else int(mode)
        return self._mode_cli.call_async(req) if self._mode_cli.service_is_ready() else None

    def arm(self, arm: bool = True):
        req = ArmMotors.Request()
        req.arm = bool(arm)
        return self._arm_cli.call_async(req) if self._arm_cli.service_is_ready() else None

    def takeoff(self, alt_m: float):
        req = Takeoff.Request()
        req.alt = float(alt_m)
        return self._takeoff_cli.call_async(req) if self._takeoff_cli.service_is_ready() else None

    def prearm_check(self):
        return self._prearm_cli.call_async(Trigger.Request()) if self._prearm_cli.service_is_ready() else None

    def get_parameters(self, names: list[str]):
        req = GetParameters.Request()
        req.names = list(names)
        return self._getparam_cli.call_async(req) if self._getparam_cli.service_is_ready() else None

    def goto(self, east_m: float, north_m: float, alt_m: float) -> None:
        """Guided-mode position target in the world frame, altitude above home (the pad)."""
        lat, lon = enu_to_latlon(east_m, north_m)
        msg = GlobalPosition()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | GlobalPosition.IGNORE_VZ
                         | GlobalPosition.IGNORE_AFX | GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ
                         | GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE)
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float(alt_m)
        self._gps_pub.publish(msg)

    def velocity(self, ve: float, vn: float, vu: float, yaw_rate: float = 0.0) -> None:
        """Guided-mode velocity in the world ENU frame (must be refreshed < GUID_TIMEOUT)."""
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(ve)
        msg.twist.linear.y = float(vn)
        msg.twist.linear.z = float(vu)
        msg.twist.angular.z = float(yaw_rate)
        self._vel_pub.publish(msg)


def param_value(value) -> float | None:
    """rcl_interfaces ParameterValue -> python number (None if not set)."""
    if value.type == 2:
        return float(value.integer_value)
    if value.type == 3:
        return float(value.double_value)
    if value.type == 1:
        return float(value.bool_value)
    return None
