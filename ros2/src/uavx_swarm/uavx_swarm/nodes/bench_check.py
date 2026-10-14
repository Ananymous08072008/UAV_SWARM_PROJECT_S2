"""Level 0 - bench check of one uavx_quad before it is allowed to fly.

The simulated equivalent of a bench test: power up the airframe on its pad and prove
that every sensor, the autopilot configuration, the ROS 2 link, the motors and the arming
path work. Writes bench_report.json + summary.json to the run directory and exits 0 only
if every check passes.

Checks
  sensors      camera, rangefinder, GPS, magnetometer, barometer, ground truth publish at rate
  camera       image is not blank; rangefinder reads the pad below; baro / mag plausible
  ap_link      ArduPilot status over DDS (/ap/v<SYSID>/status)
  params       mission-constraint parameters loaded (speed, fence, DDS namespace, SYSID)
  prearm       ArduPilot pre-arm checks pass (EKF + GPS ready)
  position     EKF world position and GPS agree with Gazebo ground truth (frame alignment)
  motors       MAVLink motor test spins each of the 4 motors on its own output channel
  arming       GUIDED + arm + disarm through the DDS services
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, LaserScan, MagneticField, NavSatFix

from ..ap_client import ArduPilotClient, param_value
from ..common import SENSOR_QOS, latlon_to_enu, load_scenario, now_s, uav_by_id

SENSORS = {  # topic suffix -> (type, minimum rate Hz)
    "camera/image": (Image, 3.0),
    "rangefinder": (LaserScan, 7.0),
    "navsat": (NavSatFix, 20.0),
    "magnetometer": (MagneticField, 20.0),
    "air_pressure": (FluidPressure, 20.0),
    "odometry": (Odometry, 15.0),
}
EXPECTED_PARAMS = {"WP_SPD": 5, "LOIT_SPEED_MS": 5, "RTL_SPEED_MS": 5, "RTL_ALT_M": 30,
                   "FENCE_ENABLE": 1, "FENCE_ALT_MAX": 100, "FENCE_RADIUS": 1600,
                   "FRAME_CLASS": 1, "DDS_USE_NS": 1}


class BenchCheck(Node):
    def __init__(self):
        super().__init__("bench_check")
        self.declare_parameter("scenario", "level0")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("uav_id", "uav1")
        self.declare_parameter("rate_window_s", 10.0)
        self.scenario = load_scenario(self.get_parameter("scenario").value)
        self.run_dir = self.get_parameter("run_dir").value
        self.uav = uav_by_id(self.scenario, self.get_parameter("uav_id").value)
        self.window = float(self.get_parameter("rate_window_s").value)
        self.ap = ArduPilotClient(self, self.uav["sysid"])

        self.stamps: dict[str, list[float]] = {k: [] for k in SENSORS}
        self.last: dict[str, object] = {}
        for suffix, (msg_type, _) in SENSORS.items():
            self.create_subscription(msg_type, f"/{self.uav['id']}/{suffix}",
                                     lambda m, s=suffix: self._on_sensor(s, m), SENSOR_QOS)

        self.results: list[dict] = []
        self.phase = "wait_data"
        self.phase_t0 = None
        self.future = None
        self.next_try = 0.0
        self.mav_thread: threading.Thread | None = None
        self.mav_result: dict | None = None
        self.exit_code: int | None = None
        self.t_start_wall = time.time()
        self.create_timer(0.5, self._step)
        self.get_logger().info(f"Level 0 bench check for {self.uav['id']} (SYSID {self.uav['sysid']})")

    # ---------------------------------------------------------------- helpers
    def _on_sensor(self, suffix: str, msg) -> None:
        self.stamps[suffix].append(now_s(self))
        self.last[suffix] = msg

    def _record(self, name: str, ok: bool, value, detail: str = "") -> None:
        self.results.append({"check": name, "pass": bool(ok), "value": value, "detail": detail})
        self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name:<22} {value}  {detail}")

    def _enter(self, phase: str) -> None:
        self.phase = phase
        self.phase_t0 = now_s(self)
        self.future = None
        self.next_try = 0.0

    def _elapsed(self) -> float:
        return now_s(self) - (self.phase_t0 if self.phase_t0 is not None else now_s(self))

    def _truth(self):
        odo = self.last.get("odometry")
        if odo is None:
            return None
        p = odo.pose.pose.position
        return (p.x, p.y, p.z)

    # ---------------------------------------------------------------- state machine
    def _step(self) -> None:
        if self.exit_code is not None:
            return
        if now_s(self) <= 0.0:          # no /clock yet
            return
        if self.phase_t0 is None:
            self.phase_t0 = now_s(self)
        handler = getattr(self, f"_phase_{self.phase}")
        handler()

    def _phase_wait_data(self) -> None:
        if all(self.stamps[s] for s in SENSORS):
            for s in SENSORS:
                self.stamps[s].clear()
            self._enter("rates")
        elif self._elapsed() > 90.0:
            missing = [s for s in SENSORS if not self.stamps[s]]
            self._record("sensors", False, "no data", f"missing: {missing}")
            self._enter("ap_link")

    def _phase_rates(self) -> None:
        if self._elapsed() < self.window:
            return
        for suffix, (_, min_rate) in SENSORS.items():
            rate = len(self.stamps[suffix]) / self.window
            self._record(f"rate {suffix}", rate >= min_rate, f"{rate:.1f} Hz", f">= {min_rate} Hz")
        self._check_values()
        self._enter("ap_link")

    def _check_values(self) -> None:
        img = self.last.get("camera/image")
        if img is not None:
            arr = np.frombuffer(bytes(img.data), dtype=np.uint8)
            mean, std = float(arr.mean()), float(arr.std())
            self._record("camera image", 15 < mean < 240 and std > 2.0,
                         f"{img.width}x{img.height} mean {mean:.0f} std {std:.1f}", "not blank")
        scan = self.last.get("rangefinder")
        if scan is not None and scan.ranges:
            r = float(scan.ranges[0])
            self._record("rangefinder on pad", 0.0 < r < 1.0, f"{r:.2f} m", "reads the pad below")
        baro = self.last.get("air_pressure")
        if baro is not None:
            kpa = baro.fluid_pressure / 1000.0
            self._record("barometer", 85.0 < kpa < 105.0, f"{kpa:.1f} kPa", "584 m site elevation")
        mag = self.last.get("magnetometer")
        if mag is not None:
            f = mag.magnetic_field
            norm = math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2)
            self._record("magnetometer", norm > 0.0, f"|B| {norm:.2e}", "non-zero field")
        fix = self.last.get("navsat")
        truth = self._truth()
        if fix is not None and truth is not None:
            e, n = latlon_to_enu(fix.latitude, fix.longitude)
            err = math.hypot(e - truth[0], n - truth[1])
            self._record("gazebo GPS vs truth", err < 1.0, f"{err:.2f} m", "world origin = CMAC")
        if truth is not None:
            pad = self.uav["pad_m"]
            err = math.hypot(truth[0] - pad[0], truth[1] - pad[1])
            self._record("spawned on pad", err < 0.5, f"{err:.2f} m", f"pad {pad}")

    def _phase_ap_link(self) -> None:
        if self.ap.connected and self.ap.services_ready():
            self._record("ArduPilot DDS link", True, f"mode {self.ap.mode_name}",
                         f"/ap/v{self.uav['sysid']}/status + services")
            self._enter("params")
        elif self._elapsed() > 90.0:
            self._record("ArduPilot DDS link", False, "no status", "is the micro-ROS agent running?")
            self._finish()

    def _phase_params(self) -> None:
        # ArduPilot's DDS GetParameters takes a handful of names per request, so read in
        # batches, and re-send a request whose reply never came (lost during discovery).
        expected = dict(EXPECTED_PARAMS, MAV_SYSID=self.uav["sysid"])
        if not hasattr(self, "param_got"):
            self.param_got, self.param_queue = {}, list(expected)
        if self.future is not None and self.future.done():
            batch = self.param_batch
            for name, value in zip(batch, self.future.result().values):
                self.param_got[name] = param_value(value)
            self.param_queue = self.param_queue[len(batch):]
            self.future = None
        if self.param_queue and (self.future is None or now_s(self) >= self.next_try):
            self.param_batch = self.param_queue[:4]
            self.future = self.ap.get_parameters(self.param_batch)
            self.next_try = now_s(self) + 5.0
        if self.param_queue and self._elapsed() < 60.0:
            return
        got = {k: self.param_got.get(k) for k in expected}
        bad = {k: v for k, v in got.items() if v is None or abs(v - expected[k]) > 1e-3}
        self._record("parameters", not bad, json.dumps(got), f"mismatch: {bad}" if bad else "constraints loaded")
        self._enter("prearm")

    def _phase_prearm(self) -> None:
        if self.future is not None and self.future.done():
            res = self.future.result()
            self.future = None
            if res.success:
                self._record("pre-arm checks", True, f"after {self._elapsed():.0f} s sim", res.message)
                self._enter("position")
                return
            self.last_prearm_msg = res.message
        if self.future is None and now_s(self) >= self.next_try:
            self.future = self.ap.prearm_check()
            self.next_try = now_s(self) + 2.0
        if self._elapsed() > 180.0:
            self._record("pre-arm checks", False, "not ready", getattr(self, "last_prearm_msg", ""))
            self._enter("position")

    def _phase_position(self) -> None:
        truth = self._truth()
        if not self.ap.has_position or truth is None:
            if self._elapsed() > 30.0:
                self._record("EKF position vs truth", False, "no EKF position")
                self._enter("motors")
            return
        e, n, _ = self.ap.position
        err = math.hypot(e - truth[0], n - truth[1])
        self._record("EKF position vs truth", err < 1.5, f"{err:.2f} m", "ArduPilot GPS/EKF -> world frame")
        gps = self.ap.gps_position
        if gps:
            gerr = math.hypot(gps[0] - truth[0], gps[1] - truth[1])
            self._record("ArduPilot GPS vs truth", gerr < 2.5, f"{gerr:.2f} m", "SITL home = world origin")
        self._enter("motors")

    def _phase_motors(self) -> None:
        if self.mav_thread is None:
            port = 14550 + 10 * int(self.uav["instance"])
            self.mav_thread = threading.Thread(target=self._motor_test, args=(port,), daemon=True)
            self.mav_thread.start()
            return
        if self.mav_thread.is_alive():
            return
        res = self.mav_result or {"ok": False, "detail": "no result"}
        self._record("motor test", res["ok"], res.get("mapping", "-"), res.get("detail", ""))
        self._enter("arming")

    def _motor_test(self, port: int) -> None:
        """Blocking MAVLink motor test (own thread): each motor at 20 % for 2 s."""
        try:
            from pymavlink import mavutil
            mav = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=250)
            if mav.wait_heartbeat(timeout=20) is None:
                self.mav_result = {"ok": False, "detail": f"no heartbeat on udp {port}"}
                return
            mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                             mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)
            def outputs(timeout=1.0):
                msg = mav.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=timeout)
                return None if msg is None else [getattr(msg, f"servo{ch + 1}_raw") for ch in range(4)]

            def wait_idle(limit_s=15.0):
                # The test timeout runs on sim time; below real time it lasts longer on the
                # wall clock, so wait for every output to drop back before the next motor.
                t_end = time.time() + limit_s
                while time.time() < t_end:
                    out = outputs()
                    if out is not None and max(out) <= 1150:
                        return True
                return False

            mapping, raised = {}, []
            for motor in range(1, 5):
                wait_idle()
                mav.mav.command_long_send(mav.target_system, mav.target_component,
                                          mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                          motor, 0, 20, 2.0, 1, 0, 0)
                peak = [0] * 4
                t_end = time.time() + 3.0
                while time.time() < t_end:
                    out = outputs(0.5)
                    if out is not None:
                        peak = [max(p, o) for p, o in zip(peak, out)]
                spinning = [ch + 1 for ch in range(4) if peak[ch] > 1150]
                mapping[f"motor{motor}"] = spinning
                raised.append(spinning)
            distinct = all(len(s) == 1 for s in raised) and len({s[0] for s in raised}) == 4
            self.mav_result = {"ok": distinct, "mapping": json.dumps(mapping),
                               "detail": "each motor on its own output" if distinct else "unexpected outputs"}
            mav.close()
        except Exception as exc:  # pragma: no cover - reported as a failed check
            self.mav_result = {"ok": False, "detail": f"{type(exc).__name__}: {exc}"}

    def _phase_arming(self) -> None:
        step = getattr(self, "arm_step", "mode")
        if self._elapsed() > 60.0:
            self._record("arm / disarm", False, f"stuck at {step}", f"mode {self.ap.mode_name}")
            self._finish()
            return
        if step == "mode":
            if self.ap.mode_name == "GUIDED":
                self.arm_step = "arm"
            elif now_s(self) >= self.next_try:
                self.ap.set_mode("GUIDED")
                self.next_try = now_s(self) + 2.0
        elif step == "arm":
            if self.ap.armed:
                self.arm_step = "disarm"
                self.next_try = 0.0
            elif now_s(self) >= self.next_try:
                self.ap.arm(True)
                self.next_try = now_s(self) + 3.0
        elif step == "disarm":
            if not self.ap.armed:
                self._record("arm / disarm", True, "GUIDED -> armed -> disarmed", "DDS services")
                self._finish()
            elif now_s(self) >= self.next_try:
                self.ap.arm(False)
                self.next_try = now_s(self) + 2.0

    # ---------------------------------------------------------------- report
    def _finish(self) -> None:
        passed = sum(r["pass"] for r in self.results)
        ok = bool(self.results) and passed == len(self.results)
        summary = {"level": 0, "name": self.scenario["name"], "uav": self.uav["id"], "pass": ok,
                   "checks_passed": passed, "checks_total": len(self.results),
                   "sim_time_s": round(now_s(self), 1),
                   "wall_time_s": round(time.time() - self.t_start_wall, 1), "results": self.results}
        os.makedirs(self.run_dir, exist_ok=True)
        for name in ("bench_report.json", "summary.json"):
            with open(os.path.join(self.run_dir, name), "w", encoding="utf-8") as fh:
                json.dump(summary, fh, indent=2)
        lines = ["", "=" * 72, f" LEVEL 0 BENCH CHECK - {self.uav['id']}: {'PASS' if ok else 'FAIL'}"
                 f"  ({passed}/{len(self.results)} checks)", "=" * 72]
        for r in self.results:
            lines.append(f"  {'PASS' if r['pass'] else 'FAIL'}  {r['check']:<24} {r['value']}")
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self.exit_code = 0 if ok else 1


def main(args=None):
    rclpy.init(args=args)
    node = BenchCheck()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    code = node.exit_code if node.exit_code is not None else 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
