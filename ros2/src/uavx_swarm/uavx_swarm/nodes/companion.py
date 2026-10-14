"""Onboard companion computer of one UAV (Levels 1-4).

Runs next to the autopilot and drives it only through ArduPilot's ROS 2 interface
(GUIDED mode, /ap/v<SYSID>/...), exactly as a Raspberry Pi / Jetson would on the real
aircraft. Mission sources:

  mission.mode: local   the plan is on board (Levels 1-2): waypoints and/or PoI surveys
  mission.mode: gcs     the GCS assigns PoIs over the mesh radio (Levels 3-4)

State machine
  BOOT -> PREFLIGHT (pre-arm OK) -> READY -> ARMING -> TAKEOFF -> NAVIGATE <-> SURVEY
       -> HOLD (waiting for tasks) / TO_COMM (lost link, ferry data back) -> RETURN
       -> LANDING -> LANDED            FAILED if the vehicle drops out of the sky

On-board rules (the parts of the mission constraints the autopilot cannot enforce)
  * fake battery: 20 min endurance (5 %/min hover + 1.5 %/min at 5 m/s), return home at
    20 % or earlier if the trip home needs more; released PoIs go back to the GCS
  * lost GCS link: keep surveying, buffer reports (ARQ store-and-forward); afterwards fly
    back towards the last place the link worked; give up and go home after lost_link_rth_s
  * separation: the higher SYSID yields (holds) when a peer is within separation_m
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from uavx_interfaces.msg import PoiDetection, RadioPacket

from ..ap_client import ArduPilotClient
from ..common import (EVENT_QOS, SENSOR_QOS, EventBus, dist2, dumps, load_scenario,
                      now_s, spin, uav_by_id)
from ..radio_endpoint import RadioEndpoint, packet_age_s
from ..world_map import WorldMap

AIRBORNE = {"TAKEOFF", "NAVIGATE", "SURVEY", "HOLD", "TO_COMM", "RETURN", "LANDING"}
FINAL = {"LANDED", "FAILED", "DONE"}

DEFAULTS = {
    "survey_s": 20.0,
    "arrival_radius_m": 2.0,
    "alt_tolerance_m": 1.5,
    "status_period_s": 1.0,
    "lost_link_s": 4.0,
    "lost_link_rth_s": 150.0,
    "return_to_comm": True,
    "separation_m": 25.0,
    "detection_radius_m": 15.0,
    "extra_search_s": 15.0,
    "start_delay_s": 0.0,
    "cruise_speed_mps": 5.0,
    "obstacle_clearance_m": 10.0,
    "battery": {"initial_pct": 100.0, "hover_pct_per_min": 5.0, "cruise_pct_per_min": 1.5,
                "critical_pct": 20.0, "reserve_pct": 8.0},
}


class FakeBattery:
    """20-minute battery: the organisers' endurance limit (same model as the Python sim)."""

    def __init__(self, cfg: dict, cruise_speed: float):
        self.pct = float(cfg["initial_pct"])
        self.hover = float(cfg["hover_pct_per_min"])
        self.cruise = float(cfg["cruise_pct_per_min"])
        self.cruise_speed = cruise_speed

    def update(self, dt: float, airborne: bool, speed: float) -> None:
        if airborne and dt > 0:
            rate = self.hover + self.cruise * min(speed / self.cruise_speed, 1.5)
            self.pct = max(0.0, self.pct - rate * dt / 60.0)

    def needed_to_reach(self, dist_m: float, alt_m: float) -> float:
        seconds = dist_m / self.cruise_speed + alt_m / 1.5 + 15.0
        return seconds / 60.0 * (self.hover + self.cruise)


class Companion(Node):
    def __init__(self):
        super().__init__("companion")
        self.declare_parameter("scenario", "level1")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("uav_id", "uav1")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.uav = uav_by_id(self.sc, self.get_parameter("uav_id").value)
        self.id = self.uav["id"]
        cfg = json.loads(json.dumps(DEFAULTS))
        for k, v in (self.sc.get("companion") or {}).items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
        cfg.update(self.uav.get("companion", {}))
        self.cfg = cfg
        self.mode = self.sc["mission"].get("mode", "local")
        self.alt = float(self.uav["cruise_alt_m"])
        self.pad = tuple(self.uav["pad_m"][:2])
        self.gcs_id = self.sc["gcs"].get("id", "gcs")

        self.ap = ArduPilotClient(self, self.uav["sysid"])
        self.events = EventBus(self, self.id)
        self.battery = FakeBattery(cfg["battery"], cfg["cruise_speed_mps"])
        self.use_radio = "radio_net" in self.sc["components"]
        self.radio = RadioEndpoint(self, self.id, self._on_radio, self._on_acked) if self.use_radio else None

        self.state = "BOOT"
        self.state_t = 0.0
        self.tasks: list[dict] = self._local_plan() if self.mode == "local" else []
        self.task_version = -1
        self.current: dict | None = None
        self.done_ids: list[str] = []
        self.reported: dict[str, dict] = {}
        self.detections: dict[str, list] = {}
        self.return_reason = ""
        self.future = None
        self.next_cmd_t = 0.0
        self.last_t = None
        self.was_airborne = False
        self.last_gcs_hb = -1e9
        self.link_ok = False
        self.link_lost_t = None
        self.last_good_pos = None
        self.peers: dict[str, dict] = {}
        self.yielding = False
        self.max_speed = 0.0
        self.max_alt = 0.0
        self.takeoff_t = None
        self.wp_arrive_t = None
        self.map = WorldMap.for_scenario(self.sc)
        self.path: list = []
        self.path_goal = None

        self.create_subscription(PoiDetection, f"/{self.id}/poi_detections", self._on_detection, SENSOR_QOS)
        self.create_subscription(String, "/scenario/inject", self._on_inject, EVENT_QOS)
        self.status_pub = self.create_publisher(String, f"/{self.id}/companion/status", EVENT_QOS)
        self.batt_pub = self.create_publisher(BatteryState, f"/{self.id}/battery_sim", SENSOR_QOS)
        self.create_timer(0.2, self._tick)
        self.create_timer(float(cfg["status_period_s"]), self._broadcast_status)
        self.get_logger().info(f"companion {self.id}: mission mode '{self.mode}', {len(self.tasks)} on-board tasks, "
                               f"cruise {self.alt:.0f} m, radio {'on' if self.use_radio else 'off'}")

    # ================================================================ plan
    def _local_plan(self) -> list[dict]:
        m = self.sc["mission"]
        tasks = []
        for i, wp in enumerate(m.get("waypoints_m", [])):
            tasks.append({"id": f"WP{i + 1}", "kind": "waypoint", "pos": [float(wp[0]), float(wp[1])],
                          "alt": float(wp[2]) if len(wp) > 2 else self.alt, "hover_s": float(m.get("hover_s", 0.0)),
                          "priority": 1})
        if m.get("survey_pois"):
            for p in self.sc["pois"]:
                if p.get("assigned_to", self.id) != self.id:
                    continue
                tasks.append(self._poi_task(p))
        return tasks

    def _poi_task(self, p: dict) -> dict:
        reported = p.get("reported_m", p["pos_m"])
        return {"id": p["id"], "kind": "survey", "pos": [float(reported[0]), float(reported[1])],
                "alt": float(p.get("survey_alt_m", self.alt)), "survey_s": float(p.get("survey_s", self.cfg["survey_s"])),
                "priority": int(p.get("priority", 1))}

    # ================================================================ helpers
    def _set(self, state: str, **info) -> None:
        if state != self.state:
            self.events.emit("STATE", state=state, prev=self.state, **info)
        self.state = state
        self.state_t = now_s(self)
        self.future = None
        self.next_cmd_t = 0.0

    def _pos(self):
        return self.ap.position if self.ap.has_position else (self.pad[0], self.pad[1], 0.0)

    def _every(self, period: float) -> bool:
        t = now_s(self)
        if t >= self.next_cmd_t:
            self.next_cmd_t = t + period
            return True
        return False

    def _arrived(self, target, alt) -> bool:
        e, n, z = self._pos()
        return dist2((e, n), target) < self.cfg["arrival_radius_m"] and abs(z - alt) < self.cfg["alt_tolerance_m"]

    # ================================================================ main loop
    def _tick(self) -> None:
        t = now_s(self)
        if t <= 0.0:
            return
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        airborne = self.ap.armed and self.ap.flying
        self.battery.update(dt, airborne, self.ap.ground_speed)
        if self.ap.has_position and airborne:
            self.max_speed = max(self.max_speed, self.ap.ground_speed)
            self.max_alt = max(self.max_alt, self._pos()[2])
        if self.radio:
            self.radio.tick()
            self._update_link(t)

        if self.state in FINAL:
            return
        # the vehicle stopped flying without being told to land: crash / motor failure
        if self.state in AIRBORNE and self.state != "LANDING" and self.was_airborne and not self.ap.armed:
            self._fail("disarmed in flight")
            return
        if self.state in AIRBORNE:
            # only a vehicle that really got off the ground can "fail in flight"
            # (ArduPilot also auto-disarms a GUIDED vehicle whose take-off was refused)
            if self.ap.has_position and self._pos()[2] > 3.0:
                self.was_airborne = True
            if self.state != "LANDING" and self.ap.mode_name in ("RTL", "LAND", "SMART_RTL") and t - self.state_t > 3.0:
                self.events.emit("AUTOPILOT_TOOK_OVER", mode=self.ap.mode_name)
                self._set("LANDING", reason=f"autopilot {self.ap.mode_name}")
            if self.state not in ("RETURN", "LANDING") and not self.return_reason:
                self._check_battery()
        getattr(self, f"_st_{self.state.lower()}")()

    # ---------------------------------------------------------------- ground states
    def _st_boot(self) -> None:
        if self.ap.connected and self.ap.has_position and self.ap.services_ready():
            self._set("PREFLIGHT")

    def _st_preflight(self) -> None:
        if self.future is not None and self.future.done():
            ok = self.future.result().success
            self.future = None
            if ok:
                self._set("READY")
                return
        if self.future is None and self._every(2.0):
            self.future = self.ap.prearm_check()

    def _st_ready(self) -> None:
        if self.mode == "local":
            if now_s(self) - self.state_t >= float(self.cfg["start_delay_s"]):
                self.events.emit("MISSION_START", tasks=[x["id"] for x in self.tasks])
                self._set("ARMING")
        elif self.tasks:
            self._set("ARMING")
        elif self.return_reason:          # recalled before it ever flew
            self._set("DONE", reason=self.return_reason)

    def _st_arming(self) -> None:
        if self.ap.mode_name != "GUIDED":
            if self._every(2.0):
                self.ap.set_mode("GUIDED")
        elif not self.ap.armed:
            if self._every(3.0):
                self.ap.arm(True)
        else:
            self.ap.takeoff(self.alt)
            self.takeoff_t = now_s(self)
            self._set("TAKEOFF", alt=self.alt)
        if now_s(self) - self.state_t > 90.0:
            self.events.emit("ARMING_TIMEOUT", mode=self.ap.mode_name)
            self._set("PREFLIGHT")

    def _st_takeoff(self) -> None:
        z = self._pos()[2]
        if not self.ap.armed and not self.was_airborne and now_s(self) - self.state_t > 2.0:
            self.events.emit("TAKEOFF_RETRY", reason="disarmed on the ground")
            self._set("ARMING")
            return
        if z >= self.alt - self.cfg["alt_tolerance_m"]:
            self.events.emit("AIRBORNE", alt=round(z, 1), climb_s=round(now_s(self) - self.takeoff_t, 1))
            self._next_task()
        elif z < 1.0 and now_s(self) - self.state_t > 5.0 and self._every(4.0):
            self.ap.takeoff(self.alt)       # takeoff request lost / refused: ask again

    # ---------------------------------------------------------------- tasking
    def _next_task(self) -> None:
        self.current = None
        if self.return_reason:
            self._set("RETURN", reason=self.return_reason)
            return
        if (self.radio and not self.link_ok and self.radio.pending and self.cfg["return_to_comm"]
                and self.last_good_pos is not None):
            self._set("TO_COMM", buffered=self.radio.pending)
            return
        if self.tasks:
            self.current = self.tasks.pop(0)
            self._set("NAVIGATE", task=self.current["id"], target=self.current["pos"])
            return
        if self.mode == "local":
            self.return_reason = "mission complete"
            self._set("RETURN", reason=self.return_reason)
        else:
            self._set("HOLD")

    def _complete_current(self, outcome: str) -> None:
        task = self.current
        self.done_ids.append(task["id"])
        est = self._estimate(task["id"])
        self.events.emit("TASK_DONE", task=task["id"], kind=task["kind"], outcome=outcome, estimate=est)
        if self.radio and task["kind"] == "survey":
            self.radio.send(self.gcs_id, "TASK_DONE", {"poi_id": task["id"], "outcome": outcome,
                                                      "estimate": est, "t": now_s(self)}, reliable=True)
        self._next_task()

    def _estimate(self, poi_id: str):
        pts = self.detections.get(poi_id)
        if not pts:
            return None
        return [round(sum(p[0] for p in pts) / len(pts), 2), round(sum(p[1] for p in pts) / len(pts), 2)]

    def _separation_hold(self) -> bool:
        """Yield rule: the higher SYSID holds while a peer is too close."""
        me = self._pos()
        t = now_s(self)
        for pid, peer in self.peers.items():
            if t - peer["t"] > 3.0 or peer["sysid"] > self.uav["sysid"]:
                continue
            p = peer["pos"]
            if dist2(me, p) < self.cfg["separation_m"] and abs(me[2] - p[2]) < 15.0:
                if not self.yielding:
                    self.events.emit("SEPARATION_HOLD", peer=pid, distance_m=round(dist2(me, p), 1))
                self.yielding = True
                return True
        if self.yielding:
            self.events.emit("SEPARATION_CLEAR")
        self.yielding = False
        return False

    # ---------------------------------------------------------------- path following
    def _fly_to(self, target, alt: float, period: float = 1.0) -> bool:
        """Fly an obstacle-free path to target at height alt; True once there.

        With a 3-D map the companion plans around every obstacle taller than alt minus the
        clearance (A*, world_map.py) and feeds ArduPilot one leg at a time; without a map
        the path is the straight line. Re-plans whenever the target changes.
        """
        goal = (round(float(target[0]), 1), round(float(target[1]), 1), round(float(alt), 1))
        if goal != self.path_goal:
            e, n, _ = self._pos()
            if self.map is not None:
                self.path = self.map.plan((e, n), goal[:2], alt, self.cfg["obstacle_clearance_m"])
            else:
                self.path = [goal[:2]]
            self.path_goal = goal
            self.next_cmd_t = 0.0
            if len(self.path) > 1:
                self.events.emit("PATH_PLANNED", goal=list(goal), legs=len(self.path),
                                 path=[[round(p[0], 1), round(p[1], 1)] for p in self.path])
        e, n, _ = self._pos()
        while len(self.path) > 1 and dist2((e, n), self.path[0]) < 4.0:
            self.path.pop(0)             # intermediate waypoint passed: next leg straight away
            self.next_cmd_t = 0.0
        wp = self.path[0]
        if self._every(period):
            self.ap.goto(wp[0], wp[1], alt)
        return len(self.path) == 1 and self._arrived(wp, alt)

    # ---------------------------------------------------------------- airborne states
    def _st_navigate(self) -> None:
        task = self.current
        if self._separation_hold():
            if self._every(1.0):
                e, n, _ = self._pos()
                self.ap.goto(e, n, task["alt"])
            self.path_goal = None        # re-plan from wherever the hold leaves us
            return
        if self._fly_to(task["pos"], task["alt"]):
            if task["kind"] == "waypoint":
                if self.wp_arrive_t is None:
                    self.wp_arrive_t = now_s(self)
                if now_s(self) - self.wp_arrive_t >= task.get("hover_s", 0.0):
                    self.wp_arrive_t = None
                    self._complete_current("reached")
            else:
                self.events.emit("SURVEY_START", task=task["id"])
                self._set("SURVEY", task=task["id"])

    def _st_survey(self) -> None:
        task = self.current
        self._fly_to(task["pos"], task["alt"])      # holds over the end of the planned path
        elapsed = now_s(self) - self.state_t
        found = task["id"] in self.reported
        if elapsed >= task["survey_s"] and (found or elapsed >= task["survey_s"] + self.cfg["extra_search_s"]):
            self._complete_current("surveyed" if found else "not_found")

    def _st_hold(self) -> None:
        if self.tasks:
            self._next_task()
            return
        hold = self.cfg.get("hold_m")
        if hold:
            self._fly_to(hold, self.alt, period=2.0)
        elif self._every(2.0):
            e, n, _ = self._pos()
            self.ap.goto(e, n, self.alt)

    def _st_to_comm(self) -> None:
        if self.link_ok:
            self.events.emit("COMM_REGAINED", buffered=self.radio.pending)
            self._next_task()
            return
        target = self.last_good_pos
        if self._fly_to(target, self.alt):
            # still nothing where the link last worked: keep closing in on the GCS
            self.last_good_pos = (0.6 * target[0] + 0.4 * self.pad[0], 0.6 * target[1] + 0.4 * self.pad[1])

    def _st_return(self) -> None:
        self._fly_to(self.pad, self.alt)
        e, n, _ = self._pos()
        if dist2((e, n), self.pad) < 1.5:
            self.ap.set_mode("LAND")
            self._set("LANDING")

    def _st_landing(self) -> None:
        if not self.ap.armed and now_s(self) - self.state_t > 2.0:
            e, n, _ = self._pos()
            self.events.emit("LANDED", pad_error_m=round(dist2((e, n), self.pad), 2),
                             battery_pct=round(self.battery.pct, 1), flight_s=round(now_s(self) - (self.takeoff_t or 0), 1),
                             max_speed_mps=round(self.max_speed, 2), max_alt_m=round(self.max_alt, 1),
                             tasks_done=self.done_ids)
            self._set("LANDED")
            return
        if self.ap.mode_name not in ("LAND", "RTL", "SMART_RTL") and self._every(3.0):
            self.ap.set_mode("LAND")

    def _st_landed(self) -> None:
        pass

    def _st_done(self) -> None:
        pass

    def _fail(self, reason: str) -> None:
        self.events.emit("UAV_FAILED", reason=reason, task=self.current["id"] if self.current else None)
        self._set("FAILED", reason=reason)
        if self.radio:
            self.radio.enabled = False

    # ---------------------------------------------------------------- battery
    def _check_battery(self) -> None:
        e, n, z = self._pos()
        need = self.battery.needed_to_reach(dist2((e, n), self.pad), z)
        crit = self.cfg["battery"]["critical_pct"]
        reserve = self.cfg["battery"]["reserve_pct"]
        if self.battery.pct <= crit or self.battery.pct <= need + reserve:
            self.return_reason = "battery"
            released = ([self.current["id"]] if self.current and self.current["kind"] == "survey" else []) + \
                       [x["id"] for x in self.tasks if x["kind"] == "survey"]
            self.events.emit("BATTERY_RTH", battery_pct=round(self.battery.pct, 1), needed_pct=round(need, 1),
                             released=released)
            if self.radio and released:
                self.radio.send(self.gcs_id, "TASK_RELEASE", {"poi_ids": released, "reason": "battery"}, reliable=True)
            self.tasks = []
            self.current = None
            self._set("RETURN", reason="battery")

    # ================================================================ radio
    def _update_link(self, t: float) -> None:
        ok = (t - self.last_gcs_hb) < self.cfg["lost_link_s"]
        if ok != self.link_ok:
            if ok:
                down = None if self.link_lost_t is None else round(t - self.link_lost_t, 1)
                self.events.emit("LINK_RESTORED", down_s=down, buffered=self.radio.pending)
                self.radio.flush()       # deliver the store-and-forward backlog now, not at the next back-off
            elif self.state in AIRBORNE or self.state == "READY":
                self.link_lost_t = t
                self.events.emit("LINK_LOST", pos=[round(v, 1) for v in self._pos()], buffered=self.radio.pending)
            self.link_ok = ok
        if (not ok and self.link_lost_t is not None and self.state in ("NAVIGATE", "SURVEY", "HOLD", "TO_COMM")
                and not self.return_reason and t - self.link_lost_t > self.cfg["lost_link_rth_s"]):
            self.return_reason = "lost link"
            self.events.emit("LOST_LINK_RTH", down_s=round(t - self.link_lost_t, 1))
            if self.state in ("HOLD", "TO_COMM"):
                self._set("RETURN", reason=self.return_reason)

    def _on_radio(self, pkt: RadioPacket, payload: dict) -> None:
        if pkt.kind == "HEARTBEAT" and pkt.src == self.gcs_id:
            self.last_gcs_hb = now_s(self)
            if self.ap.has_position and self.state in AIRBORNE:
                e, n, _ = self._pos()
                self.last_good_pos = (e, n)
        elif pkt.kind == "STATUS" and pkt.src != self.gcs_id:
            self.peers[pkt.src] = {"pos": payload.get("pos", [0, 0, 0]), "sysid": payload.get("sysid", 99),
                                   "t": now_s(self)}
        elif pkt.kind == "TASKS":
            self._apply_tasks(payload, packet_age_s(self, pkt))
        elif pkt.kind == "RECALL":
            self.return_reason = payload.get("reason", "recall")
            self.events.emit("RECALLED", reason=self.return_reason)
            if self.state in ("HOLD", "NAVIGATE", "TO_COMM"):
                self.tasks = []
                self._set("RETURN", reason=self.return_reason)

    def _apply_tasks(self, payload: dict, age_s: float) -> None:
        version = int(payload.get("version", 0))
        if version <= self.task_version:
            return
        self.task_version = version
        incoming = [t for t in payload.get("tasks", []) if t["id"] not in self.done_ids]
        for t in incoming:
            t.setdefault("kind", "survey")
            t.setdefault("alt", self.alt)
            t.setdefault("survey_s", self.cfg["survey_s"])
        ids = [t["id"] for t in incoming]
        cur = self.current["id"] if self.current else None
        self.events.emit("TASKS_RECEIVED", version=version, tasks=ids, age_s=round(age_s, 2))
        if self.state == "SURVEY" and cur:
            # finish the survey in progress; the rest of the queue follows the GCS order
            self.tasks = [t for t in incoming if t["id"] != cur]
            return
        if self.state == "NAVIGATE" and cur:
            if cur not in ids:                               # re-assigned to someone else
                self.tasks = incoming
                self._next_task()
                return
            if ids and ids[0] != cur:                        # GCS put something more urgent first
                self.events.emit("PREEMPTED", dropped=cur, now=ids[0])
                self.tasks = incoming
                self._next_task()
                return
            self.tasks = [t for t in incoming if t["id"] != cur]
            return
        self.tasks = incoming

    def _on_acked(self, entry: dict, latency_s: float) -> None:
        if entry["kind"] == "POI_REPORT":
            self.events.emit("REPORT_ACKED", poi=entry["payload"].get("poi_id"), latency_s=round(latency_s, 2),
                             attempts=entry["attempt"])

    def _broadcast_status(self) -> None:
        if not self.ap.has_position:
            return
        e, n, z = self._pos()
        body = {"pos": [round(e, 2), round(n, 2), round(z, 2)], "sysid": self.uav["sysid"],
                "batt": round(self.battery.pct, 2), "state": self.state,
                "task": self.current["id"] if self.current else None,
                "queue": [x["id"] for x in self.tasks], "done": self.done_ids,
                "pending": self.radio.pending if self.radio else 0, "link": self.link_ok,
                "armed": self.ap.armed, "mode": self.ap.mode_name, "speed": round(self.ap.ground_speed, 2)}
        if self.radio and self.state not in ("FAILED",):
            self.radio.send("*", "STATUS", body)
        msg = String()
        msg.data = dumps(dict(body, t=round(now_s(self), 2), id=self.id))
        self.status_pub.publish(msg)
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.percentage = float(self.battery.pct / 100.0)
        b.present = True
        self.batt_pub.publish(b)

    # ================================================================ detections / injection
    def _on_detection(self, det: PoiDetection) -> None:
        task = self.current
        if task is None or task["kind"] != "survey" or self.state not in ("NAVIGATE", "SURVEY"):
            return
        if dist2((det.east_m, det.north_m), task["pos"]) > self.cfg["detection_radius_m"]:
            return
        self.detections.setdefault(task["id"], []).append((det.east_m, det.north_m))
        if task["id"] in self.reported:
            return
        t_det = det.header.stamp.sec + det.header.stamp.nanosec * 1e-9
        report = {"poi_id": task["id"], "uav": self.id, "estimate": [round(det.east_m, 2), round(det.north_m, 2)],
                  "detect_t": round(t_det, 3), "image": det.image_path, "pixel_area": round(det.pixel_area, 1)}
        self.reported[task["id"]] = report
        self.events.emit("POI_DETECTED", **report)
        if self.radio:
            self.radio.send(self.gcs_id, "POI_REPORT", report, reliable=True)

    def _on_inject(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        if ev.get("target") not in (self.id, "*"):
            return
        if ev.get("type") == "battery":
            self.battery.pct = float(ev.get("pct", self.battery.pct))
            self.events.emit("BATTERY_SET", battery_pct=self.battery.pct)


def main(args=None):
    rclpy.init(args=args)
    spin(Companion())


if __name__ == "__main__":
    main()
