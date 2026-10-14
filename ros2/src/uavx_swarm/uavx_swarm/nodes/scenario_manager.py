"""Scenario manager - plays the organisers' part (Levels 2-4).

* Mission clock: t = 0 when every UAV's companion is READY (pre-arm OK); published latched
  on /scenario/clock. Every `appear_s` / `at_s` in the scenario is relative to it.
* PoIs: at appear_s the red target is spawned in Gazebo (at its true position) and the
  PoI is announced on /scenario/poi with its *reported* position (what a field report
  would give - the UAV's camera finds the real one). PoIs with appear_s <= 0 are spawned
  as soon as Gazebo is up, so on-board plans (Level 2) find them.
* Hidden disturbances: `hidden_disturbances:` gives only types, time windows and value
  ranges; the concrete plan (when, how strong, which UAV, where new PoIs appear) is drawn
  from the run seed at t = 0, written to hidden_plan.json for the post-run audit, and never
  shown to the swarm - it only meets each one when it fires.
* Disturbances (the hidden ones of Stage 2), from the timeline or injected by hand with
  `ros2 run uavx_swarm inject ...` (-> /scenario/inject_request):
      comm_outage / packet_loss / jamming   -> radio_net
      battery                               -> that UAV's companion (fake battery)
      uav_failure                           -> radio dies + MAVLink force-disarm: it falls
      new_poi                               -> spawned and announced like any PoI
  Every one is executed by publishing on /scenario/inject and logged as DISTURBANCE.
"""
from __future__ import annotations

import json
import math
import os
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..common import EVENT_QOS, LATCHED_QOS, EventBus, dumps, load_scenario, now_s, spin
from ..world_map import WorldMap

READY_STATES = {"READY", "ARMING", "TAKEOFF", "NAVIGATE", "SURVEY", "HOLD", "TO_COMM", "RETURN", "LANDING"}


class ScenarioManager(Node):
    def __init__(self):
        super().__init__("scenario_manager")
        self.declare_parameter("scenario", "level4")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("seed", -1)
        self.sc = load_scenario(self.get_parameter("scenario").value)
        from ament_index_python.packages import get_package_share_directory
        self.marker_sdf = os.path.join(get_package_share_directory("uavx_gazebo"), "models", "poi_marker", "model.sdf")
        self.world = self.sc["world"]
        self.events = EventBus(self, "scenario")
        self.t0 = None
        self.states: dict[str, str] = {}
        self.status: dict[str, dict] = {}
        self.removed: set[str] = set()
        self.appeared: dict[str, dict] = {}
        self.hidden_count = 0
        self.run_dir = self.get_parameter("run_dir").value
        self.map = WorldMap.for_scenario(self.sc)
        seed = int(self.get_parameter("seed").value)
        if seed < 0:
            seed = self.sc.get("seed", 0)
            if seed == "random":
                seed = random.SystemRandom().randint(1, 99999)
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self.pending_pois = sorted(self.sc["pois"], key=lambda p: p["appear_s"])
        self.timeline = sorted(self.sc["disturbances"], key=lambda d: d["at_s"])
        self.spawned: set[str] = set()
        self.instances = {u["id"]: u for u in self.sc["uavs"]}

        for u in self.sc["uavs"]:
            self.create_subscription(String, f"/{u['id']}/companion/status",
                                     lambda m, uid=u["id"]: self._on_status(uid, m), EVENT_QOS)
        self.create_subscription(String, "/scenario/inject_request", self._on_request, EVENT_QOS)
        self.poi_pub = self.create_publisher(String, "/scenario/poi", EVENT_QOS)
        self.inject_pub = self.create_publisher(String, "/scenario/inject", EVENT_QOS)
        self.clock_pub = self.create_publisher(String, "/scenario/clock", LATCHED_QOS)
        self.create_timer(0.5, self._tick)
        threading.Thread(target=self._spawn_initial, daemon=True).start()
        self.get_logger().info(f"scenario '{self.sc['name']}': {len(self.pending_pois)} PoIs, "
                               f"{len(self.timeline)} scheduled disturbances")

    # ---------------------------------------------------------------- clock
    def _on_status(self, uid: str, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except ValueError:
            return
        self.states[uid] = body.get("state", "")
        self.status[uid] = body

    # ---------------------------------------------------------------- target selectors
    def _resolve_target(self, selector):
        """Pick the UAV a disturbance hits, at the moment it fires.

            uav2            that UAV
            owner:E1        whoever is flying to / surveying / has queued PoI E1
            busy            the airborne UAV with the most unfinished PoIs
            random_airborne any airborne UAV
        UAVs already failed or sent home by an earlier disturbance are never picked twice,
        and the last healthy UAV is never taken out (the mission must stay feasible).
        """
        if selector in (None, "*") or selector in self.instances:
            return selector
        airborne = [u for u, s in self.status.items()
                    if s.get("state") in ("NAVIGATE", "SURVEY", "HOLD", "TO_COMM", "TAKEOFF")
                    and u not in self.removed]
        if selector.startswith("owner:"):
            poi = selector.split(":", 1)[1]
            for u in airborne:
                s = self.status[u]
                if s.get("task") == poi or poi in s.get("queue", []):
                    return u
            selector = "busy"
        if selector == "busy":
            load = sorted(airborne, key=lambda u: -(len(self.status[u].get("queue", []))
                                                   + (1 if self.status[u].get("task") else 0)))
            return load[0] if load else None
        if selector == "random_airborne":
            return self.rng.choice(airborne) if airborne else None
        return None

    def _mission_t(self) -> float:
        return now_s(self) - self.t0

    def _tick(self) -> None:
        if now_s(self) <= 0.0:
            return
        if self.t0 is None:
            if all(self.states.get(u) in READY_STATES for u in self.instances):
                self.t0 = now_s(self)
                msg = String()
                msg.data = dumps({"t0": self.t0})
                self.clock_pub.publish(msg)
                self.events.emit("MISSION_CLOCK_START", t0=round(self.t0, 2))
                self._draw_hidden()
            return
        mt = self._mission_t()
        while self.pending_pois and self.pending_pois[0]["appear_s"] <= mt:
            self._appear(self.pending_pois.pop(0))
        while self.timeline and self.timeline[0]["at_s"] <= mt:
            self._execute(dict(self.timeline.pop(0)), scheduled=True)

    # ---------------------------------------------------------------- hidden disturbances
    def _draw(self, value):
        """[a, b] -> a random value in it (int if both ends are ints); anything else as is."""
        if isinstance(value, list) and len(value) == 2 and all(isinstance(v, (int, float)) for v in value):
            if all(isinstance(v, int) for v in value):
                return self.rng.randint(value[0], value[1])
            return round(self.rng.uniform(value[0], value[1]), 2)
        return value

    def _draw_hidden(self) -> None:
        """Turn `hidden_disturbances:` into concrete timeline entries, from the run seed.

        Nothing about them reaches the swarm: no node but this one reads the plan, and each
        entry only becomes visible (as its effect, and as a DISTURBANCE event for the log)
        when it fires. Targets like `busy` are resolved at that moment, so the draw stays
        meaningful whatever the swarm has done by then.
        """
        specs = self.sc.get("hidden_disturbances") or []
        if not specs:
            return
        plan = []
        taken = [p["pos_m"] for p in self.sc["pois"]]
        for spec in specs:
            for _ in range(int(spec.get("count", 1))):
                ev = {k: self._draw(v) for k, v in spec.items() if k not in ("window_s", "count", "region_m")}
                ev["at_s"] = round(self.rng.uniform(*spec["window_s"]), 1)
                ev["hidden"] = True
                if ev["type"] == "new_poi":
                    region = spec.get("region_m", [80, -100, 220, 100])
                    pos = self.map.random_free_point(self.rng, region, 20.0, taken) if self.map else \
                        [round(self.rng.uniform(region[0], region[2]), 1), round(self.rng.uniform(region[1], region[3]), 1)]
                    if pos is None:
                        continue
                    taken.append(pos)
                    self.hidden_count += 1
                    ev.setdefault("id", f"H{self.hidden_count}")
                    ev["pos_m"] = pos
                    off = self.rng.uniform(3.0, 7.0)
                    ang = self.rng.uniform(0, 6.283)
                    ev["reported_m"] = [round(pos[0] + off * math.cos(ang), 1), round(pos[1] + off * math.sin(ang), 1)]
                plan.append(ev)
        plan.sort(key=lambda e: e["at_s"])
        self.timeline = sorted(self.timeline + plan, key=lambda d: d["at_s"])
        with open(os.path.join(self.run_dir, "hidden_plan.json"), "w", encoding="utf-8") as fh:
            json.dump({"seed": self.seed, "disturbances": plan}, fh, indent=2)
        self.events.emit("HIDDEN_PLAN_SEALED", seed=self.seed, count=len(plan))

    def _pending_poi(self):
        done = {pid for s in self.status.values() for pid in s.get("done", [])}
        pending = [p for pid, p in self.appeared.items() if pid not in done]
        return self.rng.choice(pending) if pending else None

    # ---------------------------------------------------------------- PoIs
    def _spawn_initial(self) -> None:
        for p in self.sc["pois"]:
            if p["appear_s"] <= 0.0:
                self._spawn_marker(p, retries=30)

    def _spawn_marker(self, poi: dict, retries: int = 5) -> None:
        if poi["id"] in self.spawned:
            return
        x, y = poi["pos_m"][:2]
        req = (f'sdf_filename: "{self.marker_sdf}", name: "poi_{poi["id"]}", '
               f'pose: {{position: {{x: {x}, y: {y}, z: 0.0}}}}')
        cmd = ["gz", "service", "-s", f"/world/{self.world}/create", "--reqtype", "gz.msgs.EntityFactory",
               "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req]
        for _ in range(retries):
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
            except subprocess.TimeoutExpired:
                out = ""
            if "true" in out:
                self.spawned.add(poi["id"])
                self.get_logger().info(f"PoI marker {poi['id']} spawned at ({x}, {y})")
                return
            time.sleep(2.0)
        self.get_logger().error(f"could not spawn PoI marker {poi['id']}")

    def _appear(self, poi: dict) -> None:
        if poi["id"] not in self.spawned:
            threading.Thread(target=self._spawn_marker, args=(poi,), daemon=True).start()
        body = {"id": poi["id"], "reported_m": poi.get("reported_m", poi["pos_m"]),
                "priority": poi["priority"], "survey_s": poi["survey_s"]}
        self.appeared[poi["id"]] = poi
        msg = String()
        msg.data = dumps(body)
        self.poi_pub.publish(msg)
        self.events.emit("POI_APPEARED", poi_id=poi["id"], true_pos=poi["pos_m"], reported=body["reported_m"],
                         priority=poi["priority"], mission_t=round(self._mission_t(), 1))

    # ---------------------------------------------------------------- disturbances
    def _on_request(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            self.get_logger().error(f"bad inject request: {msg.data}")
            return
        self._execute(ev, scheduled=False)

    def _execute(self, ev: dict, scheduled: bool) -> None:
        kind = ev.get("type")
        ev.pop("at_s", None)
        if "target" in ev:
            selector = ev["target"]
            ev["target"] = self._resolve_target(selector)
            if ev["target"] is None:
                self.get_logger().warn(f"{kind}: no UAV matches '{selector}' now - skipped")
                return
            if selector != ev["target"]:
                ev["selector"] = selector
            if kind in ("uav_failure", "battery"):
                healthy = [u for u in self.instances if u not in self.removed and u != ev["target"]]
                if not healthy:
                    self.get_logger().warn(f"{kind}: {ev['target']} is the last healthy UAV - skipped")
                    return
                self.removed.add(ev["target"])
        if kind == "jamming" and ev.get("center_m") == "near_pending_poi":
            poi = self._pending_poi()
            if poi is None:
                self.get_logger().warn("jamming: no PoI left to jam near - skipped")
                return
            ev["center_m"] = [round(poi["pos_m"][0] + self.rng.uniform(-10, 10), 1),
                              round(poi["pos_m"][1] + self.rng.uniform(-10, 10), 1)]
            ev["near_poi"] = poi["id"]
        self.events.emit("DISTURBANCE", kind=kind, scheduled=scheduled,
                         mission_t=round(self._mission_t(), 1) if self.t0 else None,
                         **{k: v for k, v in ev.items() if k != "type"})
        if kind == "new_poi":
            poi = {"id": ev["id"], "pos_m": ev["pos_m"], "reported_m": ev.get("reported_m", ev["pos_m"]),
                   "priority": int(ev.get("priority", 5)), "survey_s": float(ev.get("survey_s", 20.0)),
                   "appear_s": 0.0}
            self.sc["pois"].append(poi)
            self._appear(poi)
            return
        msg = String()
        msg.data = dumps(ev)
        self.inject_pub.publish(msg)
        if kind == "uav_failure":
            uav = self.instances.get(ev.get("target"))
            if uav:
                threading.Thread(target=self._force_disarm, args=(uav,), daemon=True).start()

    def _force_disarm(self, uav: dict) -> None:
        """Motor failure: emergency-stop the motors through MAVLink (MAVProxy output port)."""
        try:
            from pymavlink import mavutil
            port = 14550 + 10 * int(uav["instance"])
            mav = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=251)
            if mav.wait_heartbeat(timeout=15) is None:
                self.get_logger().error(f"uav_failure: no heartbeat from {uav['id']} on udp {port}")
                return
            for _ in range(5):
                mav.mav.command_long_send(uav["sysid"], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                          0, 0, 21196, 0, 0, 0, 0, 0)
                hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    self.get_logger().warn(f"uav_failure: {uav['id']} motors stopped")
                    break
            mav.close()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"uav_failure: {exc}")


def main(args=None):
    rclpy.init(args=args)
    spin(ScenarioManager())


if __name__ == "__main__":
    main()
