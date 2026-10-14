"""Referee for Levels 1-4: logs everything, scores the run, ends it.

Uses ground truth (Gazebo odometry) and every node's events - information no UAV or the
GCS has - so the verdict does not depend on the autonomy grading itself.

Writes to the run directory
  events.jsonl     every event on /uavx/events (+ the referee's own)
  timeseries.csv   2 Hz per UAV: truth position, speed, state, battery, GCS link, separation
  summary.json     metrics + pass/fail per criterion of the scenario's `pass:` block

The run ends when every UAV is LANDED / FAILED / DONE (or at duration_s sim time).
Metric names follow the organisers' list (Stage 2 report template, section 10).
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ..common import EVENT_QOS, SENSOR_QOS, dist2, dist3, load_scenario, now_s
from ..world_map import WorldMap

FINAL = {"LANDED", "FAILED", "DONE"}
GEOFENCE = {"x": (-200.0, 1100.0), "y": (-550.0, 550.0), "z_max": 100.0}


class MissionMonitor(Node):
    def __init__(self):
        super().__init__("mission_monitor")
        self.declare_parameter("scenario", "level1")
        self.declare_parameter("run_dir", ".")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.run_dir = self.get_parameter("run_dir").value
        os.makedirs(self.run_dir, exist_ok=True)
        self.uavs = [u["id"] for u in self.sc["uavs"]]
        self.gcs_id = self.sc["gcs"].get("id", "gcs")
        self.gcs_pos = tuple(self.sc["gcs"]["position_m"])
        self.use_radio = "radio_net" in self.sc["components"]

        self.truth: dict[str, tuple] = {}
        self.prev_truth: dict[str, tuple] = {}
        self.status: dict[str, dict] = {}
        self.events: list[dict] = []
        self.links: dict[tuple, bool] = {}
        self.radio_stats: dict = {}
        self.max_speed = {u: 0.0 for u in self.uavs}
        self.max_alt = {u: 0.0 for u in self.uavs}
        self.airborne_s = {u: 0.0 for u in self.uavs}
        self.connected_s = {u: 0.0 for u in self.uavs}
        self.min_sep = math.inf
        self.min_sep_pair = None
        self.collisions = 0
        self.fence_violations = 0
        self.fence_flags = set()
        self.t_first_airborne = None
        self.t_last_final = None
        self.t_start_wall = time.time()
        self.done = False
        self.exit_code = None
        self.map = WorldMap.for_scenario(self.sc)
        self.track: dict[str, list] = {u: [] for u in self.uavs}       # (t, x, y, z, gcs_link)
        self.obstacle_hits: set = set()
        self.min_clearance = {u: math.inf for u in self.uavs}

        self.ev_fh = open(os.path.join(self.run_dir, "events.jsonl"), "w", encoding="utf-8")
        self.ts_fh = open(os.path.join(self.run_dir, "timeseries.csv"), "w", newline="", encoding="utf-8")
        self.ts = csv.writer(self.ts_fh)
        self.ts.writerow(["t", "uav", "east_m", "north_m", "up_m", "speed_mps", "state", "battery_pct",
                          "gcs_link_up", "min_separation_m"])

        for u in self.uavs:
            self.create_subscription(Odometry, f"/{u}/odometry", lambda m, u=u: self._on_odom(u, m), SENSOR_QOS)
            self.create_subscription(String, f"/{u}/companion/status", lambda m, u=u: self._on_status(u, m), EVENT_QOS)
        self.create_subscription(String, "/uavx/events", self._on_event, EVENT_QOS)
        if self.use_radio:
            from uavx_interfaces.msg import NetworkState
            self.create_subscription(NetworkState, "/radio/network_state", self._on_network, EVENT_QOS)
            self.create_subscription(String, "/radio/stats", self._on_radio_stats, EVENT_QOS)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"referee: level {self.sc['level']} '{self.sc['name']}', UAVs {self.uavs}, "
                               f"watchdog {self.sc['duration_s']:.0f} s sim")

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, uav: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.truth[uav] = (p.x, p.y, p.z, now_s(self))

    def _on_status(self, uav: str, msg: String) -> None:
        try:
            self.status[uav] = json.loads(msg.data)
        except ValueError:
            pass

    def _on_event(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        self.events.append(ev)
        self.ev_fh.write(json.dumps(ev) + "\n")
        self.ev_fh.flush()

    def _on_network(self, msg) -> None:
        self.links = {(l.a, l.b): l.up for l in msg.links}

    def _on_radio_stats(self, msg: String) -> None:
        try:
            self.radio_stats = json.loads(msg.data)
        except ValueError:
            pass

    def _gcs_link(self, uav: str) -> bool:
        return self.links.get((self.gcs_id, uav), self.links.get((uav, self.gcs_id), False))

    # ---------------------------------------------------------------- periodic
    def _tick(self) -> None:
        t = now_s(self)
        if t <= 0.0 or self.done:
            return
        dt = 0.5
        for u in self.uavs:
            if u not in self.truth:
                continue
            x, y, z, ts = self.truth[u]
            speed = 0.0
            if u in self.prev_truth:
                px, py, pz, pts = self.prev_truth[u]
                if ts > pts:
                    speed = math.hypot(x - px, y - py) / (ts - pts)
            self.prev_truth[u] = (x, y, z, ts)
            st = self.status.get(u, {})
            airborne = z > 1.0 and st.get("state") not in FINAL
            if airborne:
                self.airborne_s[u] += dt
                self.t_first_airborne = self.t_first_airborne or t
                self.max_speed[u] = max(self.max_speed[u], speed)
                self.max_alt[u] = max(self.max_alt[u], z)
                if self.use_radio and self._gcs_link(u):
                    self.connected_s[u] += dt
            outside = not (GEOFENCE["x"][0] <= x <= GEOFENCE["x"][1] and GEOFENCE["y"][0] <= y <= GEOFENCE["y"][1]
                           and z <= GEOFENCE["z_max"] + 2.0)
            if outside and u not in self.fence_flags:
                self.fence_violations += 1
                self.fence_flags.add(u)
                self._emit("GEOFENCE_VIOLATION", uav=u, pos=[round(x, 1), round(y, 1), round(z, 1)])
            elif not outside:
                self.fence_flags.discard(u)
            if airborne or z > 0.5:
                self.track[u].append((round(t, 1), round(x, 2), round(y, 2), round(z, 2),
                                      bool(self._gcs_link(u)) if self.use_radio else True))
            if self.map is not None and z > 2.0:
                self.min_clearance[u] = min(self.min_clearance[u], self.map.clearance(x, y, z))
                for o in self.map.obstacles:
                    if o.contains(x, y, z, 0.3) and (u, o.id) not in self.obstacle_hits:
                        self.obstacle_hits.add((u, o.id))
                        self._emit("OBSTACLE_COLLISION", uav=u, obstacle=o.id, pos=[round(x, 1), round(y, 1), round(z, 1)])
            sep = self._separation(u)
            self.ts.writerow([round(t, 2), u, round(x, 2), round(y, 2), round(z, 2), round(speed, 2),
                              st.get("state", ""), st.get("batt", ""), int(self._gcs_link(u)) if self.use_radio else "",
                              "" if sep is None else round(sep, 2)])
        self.ts_fh.flush()
        self._pair_checks()

        states = [self.status.get(u, {}).get("state") for u in self.uavs]
        all_final = all(s in FINAL for s in states)
        if all_final and self.t_first_airborne is not None:
            self.t_last_final = self.t_last_final or t
            if t - self.t_last_final > 3.0:
                self._finish("all UAVs finished")
        elif t > float(self.sc["duration_s"]):
            self._finish("watchdog: duration_s reached")

    def _separation(self, uav: str):
        if uav not in self.truth:
            return None
        best = None
        for other in self.uavs:
            if other != uav and other in self.truth:
                d = dist3(self.truth[uav][:3], self.truth[other][:3])
                best = d if best is None else min(best, d)
        return best

    def _pair_checks(self) -> None:
        for i, a in enumerate(self.uavs):
            for b in self.uavs[i + 1:]:
                if a not in self.truth or b not in self.truth:
                    continue
                d = dist3(self.truth[a][:3], self.truth[b][:3])
                if d < self.min_sep:
                    self.min_sep, self.min_sep_pair = d, (a, b)
                if d < 1.5 and (a, b) not in getattr(self, "_collided", set()):
                    self._collided = getattr(self, "_collided", set()) | {(a, b)}
                    self.collisions += 1
                    self._emit("COLLISION", pair=[a, b], distance_m=round(d, 2))

    def _emit(self, etype: str, **fields) -> None:
        ev = {"t": round(now_s(self), 3), "src": "referee", "type": etype, **fields}
        self.events.append(ev)
        self.ev_fh.write(json.dumps(ev) + "\n")
        self.get_logger().warn(f"[{etype}] {fields}")

    # ---------------------------------------------------------------- scoring
    def _ev(self, etype: str, src: str | None = None) -> list[dict]:
        return [e for e in self.events if e.get("type") == etype and (src is None or e.get("src") == src)]

    def _all_pois(self) -> list[dict]:
        """The scenario's PoIs plus every PoI that appeared during the run (hidden ones)."""
        pois = {p["id"]: {"id": p["id"], "pos_m": p["pos_m"], "priority": p["priority"], "hidden": False}
                for p in self.sc["pois"]}
        for e in self._ev("POI_APPEARED"):
            pois.setdefault(e["poi_id"], {"id": e["poi_id"], "pos_m": e["true_pos"],
                                          "priority": e.get("priority", 1), "hidden": True})
        return list(pois.values())

    def _metrics(self) -> dict:
        m: dict = {"sim_time_s": round(now_s(self), 1), "wall_time_s": round(time.time() - self.t_start_wall, 1)}
        m["real_time_factor"] = round(m["sim_time_s"] / max(m["wall_time_s"], 1e-3), 2)
        m["uavs"] = {}
        for u in self.uavs:
            landed = self._ev("LANDED", u)
            failed = self._ev("UAV_FAILED", u)
            st = self.status.get(u, {})
            m["uavs"][u] = {
                "final_state": st.get("state"),
                "battery_end_pct": st.get("batt"),
                "airborne_s": round(self.airborne_s[u], 1),
                "max_speed_mps": round(self.max_speed[u], 2),
                "max_alt_m": round(self.max_alt[u], 1),
                "pad_error_m": landed[-1].get("pad_error_m") if landed else None,
                "failed": bool(failed),
                "tasks_done": [e["task"] for e in self._ev("TASK_DONE", u)],
            }
            if self.use_radio:
                m["uavs"][u]["gcs_connectivity"] = round(self.connected_s[u] / self.airborne_s[u], 3) \
                    if self.airborne_s[u] > 0 else None
        m["min_separation_m"] = None if self.min_sep is math.inf else round(self.min_sep, 2)
        m["collisions"] = self.collisions
        m["geofence_violations"] = self.fence_violations
        if self.map is not None:
            m["obstacle_collisions"] = len(self.obstacle_hits)
            m["min_obstacle_clearance_m"] = {u: (None if v is math.inf else round(v, 1))
                                             for u, v in self.min_clearance.items()}
        # PoIs: detection + geolocation error against the true positions (hidden ones included)
        truth = {p["id"]: p for p in self._all_pois()}
        pois = {}
        for p in truth.values():
            det = [e for e in self._ev("POI_DETECTED") if e.get("poi_id") == p["id"]]
            done = [e for e in self._ev("TASK_DONE") if e.get("task") == p["id"]]
            est = next((e.get("estimate") for e in reversed(done) if e.get("estimate")), None)
            if est is None and det:
                est = det[0].get("estimate")
            err = round(dist2(est, p["pos_m"]), 2) if est else None
            rx = [e for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("poi_id") == p["id"]]
            completed = [e for e in self._ev("POI_COMPLETED", self.gcs_id) if e.get("poi_id") == p["id"]]
            pois[p["id"]] = {
                "priority": p["priority"],
                "hidden": p.get("hidden", False),
                "true_pos_m": p["pos_m"],
                "detected_by": det[0]["src"] if det else None,
                "detected_t": det[0]["t"] if det else None,
                "geolocation_error_m": err,
                "surveyed": any(e.get("outcome") == "surveyed" for e in done),
                "report_at_gcs_t": rx[0]["t"] if rx else None,
                "report_latency_s": rx[0].get("latency_s") if rx else None,
                "completed_at_gcs": bool(completed),
                "image": det[0].get("image") if det else None,
            }
        m["pois"] = pois
        if truth:
            done_w = sum(truth[k]["priority"] for k, v in pois.items() if v["surveyed"] or v["completed_at_gcs"])
            m["mission_completion_rate"] = round(
                sum(1 for v in pois.values() if v["surveyed"] or v["completed_at_gcs"]) / len(pois), 3)
            m["priority_weighted_score"] = round(done_w / sum(p["priority"] for p in truth.values()), 3)
        done_events = self._ev("TASK_DONE")
        if done_events and self.t_first_airborne:
            m["mission_completion_time_s"] = round(max(e["t"] for e in done_events) - self.t_first_airborne, 1)
        m["images_saved"] = len([f for f in os.listdir(os.path.join(self.run_dir, "images"))]) \
            if os.path.isdir(os.path.join(self.run_dir, "images")) else 0
        if self.use_radio:
            m["communication"] = self._comm_metrics()
            m["autonomy"] = self._autonomy_metrics()
        return m

    def _comm_metrics(self) -> dict:
        c: dict = {}
        stats = self.radio_stats.get("links", {})
        # keys look like "uav1->gcs|STATUS"
        tx = sum(v["tx"] for k, v in stats.items() if k.split("|")[0].endswith(f"->{self.gcs_id}") and k.endswith("|STATUS"))
        rx = sum(v["delivered"] for k, v in stats.items() if k.split("|")[0].endswith(f"->{self.gcs_id}") and k.endswith("|STATUS"))
        c["status_pdr_uav_to_gcs"] = round(rx / tx, 3) if tx else None
        all_tx = sum(v["tx"] for v in stats.values())
        all_rx = sum(v["delivered"] for v in stats.values())
        c["frame_pdr_all"] = round(all_rx / all_tx, 3) if all_tx else None
        lat = self.radio_stats.get("latency_ms", {})
        c["link_latency_ms_mean"] = lat.get("mean")
        c["link_latency_ms_p95"] = lat.get("p95")
        rep = [e.get("latency_s") for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("latency_s") is not None]
        c["poi_report_latency_s"] = {"mean": round(sum(rep) / len(rep), 2) if rep else None,
                                     "max": round(max(rep), 2) if rep else None,
                                     "within_10s": sum(1 for r in rep if r <= 10.0), "count": len(rep)}
        air = sum(self.airborne_s.values())
        c["connectivity_availability"] = round(sum(self.connected_s.values()) / air, 3) if air else None
        c["communication_downtime_s"] = {u: round(self.airborne_s[u] - self.connected_s[u], 1) for u in self.uavs}
        c["link_lost_events"] = len(self._ev("LINK_LOST"))
        return c

    def _autonomy_metrics(self) -> dict:
        a: dict = {}
        realloc = self._ev("REALLOCATED", self.gcs_id)
        a["reallocations"] = len(realloc)
        a["pois_moved"] = {k: v for e in realloc for k, v in (e.get("moved") or {}).items()}
        a["lost_declarations"] = [e.get("uav") for e in self._ev("UAV_DECLARED_LOST", self.gcs_id)]
        rec = [e.get("recovery_s") for e in self._ev("RECOVERED", self.gcs_id) if e.get("recovery_s") is not None]
        a["recovery_time_s"] = rec
        det = [e.get("detection_s") for e in self._ev("UAV_DECLARED_LOST", self.gcs_id) if e.get("detection_s") is not None]
        a["failure_detection_s"] = det
        a["preemptions"] = len(self._ev("PREEMPTED"))
        a["store_and_forward_deliveries"] = len([e for e in self._ev("POI_RECEIVED", self.gcs_id) if e.get("attempts", 1) > 1])
        return a

    # ---------------------------------------------------------------- disturbances
    def _frames(self) -> list[tuple]:
        """(t, src, dst, delivered, reason) for every radio frame, from radio_net's packets.csv."""
        path = os.path.join(self.run_dir, "packets.csv")
        if not os.path.isfile(path):
            return []
        rows = []
        with open(path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                try:
                    rows.append((float(r["t"]), r["src"], r["dst"], r["delivered"] == "1", r["reason"]))
                except (KeyError, ValueError):
                    continue
        return rows

    @staticmethod
    def _pdr(frames, t0, t1, nodes=None) -> float | None:
        """Delivery ratio of frames sent over a usable link in [t0, t1). Frames that never had a
        link (out of range) or hit a dead radio (an outage / failed UAV - its own disturbance)
        are left out, so the number isolates loss and jamming."""
        sel = [f for f in frames if t0 <= f[0] < t1 and f[4] not in ("no_link", "unknown_node", "outage")
               and (nodes is None or f[1] in nodes or f[2] in nodes)]
        return round(sum(f[3] for f in sel) / len(sel), 3) if sel else None

    def _disturbance_report(self) -> list[dict]:
        """One row per disturbance that fired: how the swarm detected it and recovered
        (the Stage 2 report template's section 7 table)."""
        frames = self._frames()
        fail_timeout = float((self.sc.get("gcs_logic") or {}).get("fail_timeout_s", 6.0))
        rows = []
        for d in self._ev("DISTURBANCE"):
            t, kind, target = d["t"], d.get("kind"), d.get("target")
            dur = float(d.get("duration_s") or 0.0)
            row = {"t": t, "mission_t": d.get("mission_t"), "type": kind, "target": target,
                   "hidden": bool(d.get("hidden")), "handled": True}
            later = [e for e in self.events if e.get("t", 0) >= t]
            if kind == "uav_failure":
                lost = next((e for e in later if e["type"] == "UAV_DECLARED_LOST" and e.get("uav") == target), None)
                realloc = next((e for e in later if e["type"] == "REALLOCATED" and e.get("lost") == target), None)
                rec = [e for e in later if e["type"] == "RECOVERED" and e.get("lost") == target]
                row.update(detection_s=round(lost["t"] - t, 2) if lost else None,
                           orphans=lost.get("orphans") if lost else None,
                           moved=realloc.get("moved") if realloc else None,
                           recovery_s=round(max(e["t"] for e in rec) - t, 2) if rec else None,
                           response="declared lost by the GCS, PoIs re-allocated")
                row["handled"] = lost is not None and (not lost.get("orphans") or bool(realloc and realloc.get("moved")))
            elif kind == "battery":
                rth = next((e for e in later if e["type"] == "BATTERY_RTH" and e["src"] == target), None)
                landed = next((e for e in later if e["type"] == "LANDED" and e["src"] == target), None)
                realloc = next((e for e in later if e["type"] == "REALLOCATED" and e.get("lost") == target), None)
                row.update(detection_s=round(rth["t"] - t, 2) if rth else None,
                           released=rth.get("released") if rth else None,
                           moved=realloc.get("moved") if realloc else None,
                           landed_battery_pct=landed.get("battery_pct") if landed else None,
                           response="returned to recharge, released PoIs re-allocated")
                row["handled"] = rth is not None and landed is not None
            elif kind == "comm_outage":
                in_contact = self._link_at(target, t)
                window = [e for e in later if e["t"] <= t + dur + 15.0 and e["src"] == target]
                down = next((e for e in window if e["type"] == "LINK_LOST"), None)
                up = next((e for e in later if e["type"] == "LINK_RESTORED" and e["src"] == target
                           and (down is None or e["t"] >= down["t"])), None)
                false_lost = [e for e in later if e["type"] == "UAV_DECLARED_LOST" and e.get("uav") == target
                              and e["t"] <= t + dur + fail_timeout + 2]
                row.update(in_contact_when_hit=in_contact,
                           detection_s=round(down["t"] - t, 2) if down else None,
                           downtime_s=round(up["t"] - down["t"], 1) if down and up else None,
                           declared_lost=bool(false_lost), response="rode through (buffered, ARQ)")
                # handled = the link came back and nobody was written off; an outage that hits a
                # UAV already out of range changes nothing it could react to
                row["handled"] = not false_lost and (up is not None or not in_contact)
            elif kind in ("packet_loss", "jamming"):
                nodes = None if target in (None, "*") else {target}
                row.update(pdr_during=self._pdr(frames, t, t + dur, nodes),
                           pdr_before=self._pdr(frames, max(0.0, t - 60.0), t, nodes),
                           response="ARQ retransmission / store-and-forward")
                if kind == "jamming":
                    row.update(center_m=d.get("center_m"), radius_m=d.get("radius_m"), near_poi=d.get("near_poi"))
            elif kind == "new_poi":
                pid = d.get("id")
                assigned = next((e for e in later if e["type"] == "ASSIGNED" and e.get("poi_id") == pid), None)
                rx = next((e for e in later if e["type"] == "POI_RECEIVED" and e.get("poi_id") == pid), None)
                done = next((e for e in later if e["type"] == "POI_COMPLETED" and e.get("poi_id") == pid), None)
                row.update(poi=pid, priority=d.get("priority"), assigned_to=assigned.get("uav") if assigned else None,
                           assign_s=round(assigned["t"] - t, 2) if assigned else None,
                           report_s=round(rx["t"] - t, 1) if rx else None,
                           completed_s=round(done["t"] - t, 1) if done else None,
                           response="prioritised and re-planned")
                row["handled"] = done is not None
            rows.append(row)
        return rows

    def _link_at(self, uav: str, t: float) -> bool:
        """Did uav have a GCS link (radio_net ground truth) at sim time t?"""
        track = self.track.get(uav) or []
        best = min(track, key=lambda p: abs(p[0] - t), default=None)
        return bool(best and abs(best[0] - t) < 2.0 and best[4])

    def _expected_failures(self) -> set:
        """UAVs allowed to end FAILED: listed in the scenario, or 'injected' = whichever
        UAV a uav_failure disturbance actually hit (targets are chosen at run time)."""
        allowed = self.sc["pass"].get("allow_failure_of", [])
        if allowed == "injected":
            return {e.get("target") for e in self._ev("DISTURBANCE") if e.get("kind") == "uav_failure"}
        return set(allowed)

    def _criteria(self, m: dict) -> list[dict]:
        p = self.sc["pass"]
        out = []

        def check(name, ok, value):
            out.append({"check": name, "pass": bool(ok), "value": value})

        expected_failed = self._expected_failures()
        survivors = [u for u in self.uavs if u not in expected_failed]
        if p.get("all_landed", True):
            bad = [u for u in self.uavs if u not in expected_failed and m["uavs"][u]["final_state"] != "LANDED"]
            check("every UAV landed on its pad", not bad, "all" if not bad else f"not landed: {bad}")
        if "pad_error_m" in p:
            errs = [m["uavs"][u]["pad_error_m"] for u in survivors if m["uavs"][u]["pad_error_m"] is not None]
            check(f"landing error <= {p['pad_error_m']} m", errs and max(errs) <= p["pad_error_m"],
                  f"max {max(errs):.2f} m" if errs else "none")
        if "max_speed_mps" in p:
            v = max(m["uavs"][u]["max_speed_mps"] for u in self.uavs)
            check(f"speed <= {p['max_speed_mps']} m/s (truth)", v <= p["max_speed_mps"], f"{v:.2f} m/s")
        if "max_alt_m" in p:
            v = max(m["uavs"][u]["max_alt_m"] for u in self.uavs)
            check(f"height <= {p['max_alt_m']} m", v <= p["max_alt_m"], f"{v:.1f} m")
        check("no geofence violation", m["geofence_violations"] == 0, m["geofence_violations"])
        if len(self.uavs) > 1:
            check("no collision", m["collisions"] == 0, m["collisions"])
            if "min_separation_m" in p:
                v = m["min_separation_m"]
                check(f"min separation >= {p['min_separation_m']} m", v is not None and v >= p["min_separation_m"],
                      f"{v} m")
        if "waypoints_reached" in p:
            n = sum(1 for e in self._ev("TASK_DONE") if e.get("kind") == "waypoint")
            check("waypoints reached", n >= p["waypoints_reached"], f"{n}/{p['waypoints_reached']}")
        pois = m.get("pois", {})
        if p.get("pois_detected") == "all":
            n = sum(1 for v in pois.values() if v["detected_by"])
            check("every PoI found by camera", n == len(pois), f"{n}/{len(pois)}")
        if "geolocation_error_m" in p:
            errs = [v["geolocation_error_m"] for v in pois.values() if v["geolocation_error_m"] is not None]
            check(f"geolocation error <= {p['geolocation_error_m']} m", errs and max(errs) <= p["geolocation_error_m"],
                  f"max {max(errs):.2f} m, mean {sum(errs) / len(errs):.2f} m" if errs else "none")
        if p.get("images_saved"):
            check("survey imagery saved", m["images_saved"] >= p["images_saved"], m["images_saved"])
        if p.get("reports_at_gcs") == "all":
            n = sum(1 for v in pois.values() if v["report_at_gcs_t"] is not None)
            check("every PoI report reached the GCS", n == len(pois), f"{n}/{len(pois)}")
        if self.map is not None:
            check("no obstacle collision (3-D map)", m.get("obstacle_collisions", 0) == 0,
                  f"{m.get('obstacle_collisions', 0)}, min clearance "
                  f"{min((v for v in m['min_obstacle_clearance_m'].values() if v is not None), default=None)} m")
        if p.get("disturbances_handled"):
            rows = m.get("disturbances", [])
            bad = [f"{r['type']}@{r['mission_t']}" for r in rows if not r["handled"]]
            hidden = sum(1 for r in rows if r["hidden"])
            check(f"every disturbance handled ({len(rows)}, {hidden} hidden)", rows and not bad,
                  f"not handled: {bad}" if bad else ", ".join(f"{r['type']}@{r['mission_t']}" for r in rows))
        if "report_latency_s" in p:
            live = p.get("live_pois") or [k for k in pois if k not in set(p.get("ferry_pois", []))]
            lat = [pois[k]["report_latency_s"] for k in live if pois[k]["report_latency_s"] is not None]
            ok = len(lat) == len(live) and all(x <= p["report_latency_s"] for x in lat)
            check(f"in-range reports <= {p['report_latency_s']} s", ok,
                  f"max {max(lat):.2f} s over {len(lat)}" if lat else "none")
        if "live_report_latency_s" in p:
            # the organisers' 10 s rule, applied where it is physically possible: the UAV
            # that found the PoI had a GCS link at that moment (otherwise it is ferried)
            live, slow = [], []
            for pid, v in pois.items():
                if v["detected_t"] is None or v["report_latency_s"] is None:
                    continue
                if self._link_at(v["detected_by"], v["detected_t"]):
                    live.append(pid)
                    if v["report_latency_s"] > p["live_report_latency_s"]:
                        slow.append(f"{pid} {v['report_latency_s']} s")
            lat = [pois[k]["report_latency_s"] for k in live]
            check(f"reports found in contact <= {p['live_report_latency_s']} s", not slow,
                  f"{len(live)} live, max {max(lat):.2f} s; {len(pois) - len(live)} ferried" if lat else
                  f"none live; {len(pois)} ferried")
        if p.get("ferry_pois"):
            ferried = [k for k in p["ferry_pois"] if pois.get(k, {}).get("report_at_gcs_t") is not None]
            check("out-of-range report delivered (store-and-forward)", len(ferried) == len(p["ferry_pois"]),
                  f"{len(ferried)}/{len(p['ferry_pois'])}")
        if p.get("pois_completed") == "all":
            n = sum(1 for v in pois.values() if v["completed_at_gcs"] or v["surveyed"])
            check("every PoI surveyed", n == len(pois), f"{n}/{len(pois)}")
        if "reallocations_min" in p:
            n = m.get("autonomy", {}).get("reallocations", 0)
            moved = m.get("autonomy", {}).get("pois_moved", {})
            check(f"orphaned PoIs re-allocated (>= {p['reallocations_min']})",
                  n >= p["reallocations_min"] and bool(moved), f"{n} re-allocation(s), moved {moved}")
        if "max_lost_declarations" in p:
            lost = m.get("autonomy", {}).get("lost_declarations", [])
            false_alarms = [u for u in lost if u not in expected_failed]
            check("no false 'UAV lost' (outage ridden through)",
                  len(lost) <= p["max_lost_declarations"] and not false_alarms, f"declared lost: {lost}")
        if "failure_detection_max_s" in p:
            d = m.get("autonomy", {}).get("failure_detection_s", [])
            check(f"failure detected within {p['failure_detection_max_s']} s", d and max(d) <= p["failure_detection_max_s"],
                  d or "not detected")
        if "status_pdr_min" in p:
            v = m.get("communication", {}).get("status_pdr_uav_to_gcs")
            check(f"status PDR >= {p['status_pdr_min']}", v is not None and v >= p["status_pdr_min"], v)
        if p.get("emergency_poi"):
            e = pois.get(p["emergency_poi"], {})
            check(f"emergency PoI {p['emergency_poi']} surveyed", e.get("surveyed") or e.get("completed_at_gcs"),
                  e.get("detected_t"))
        return out

    # ---------------------------------------------------------------- figures
    @staticmethod
    def _footprint(o, n: int = 16) -> list[tuple]:
        if o.shape == "cylinder":
            return [(o.cx + o.radius * math.cos(2 * math.pi * k / n), o.cy + o.radius * math.sin(2 * math.pi * k / n))
                    for k in range(n)]
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        hl, hw = o.length / 2, o.width / 2
        return [(o.cx + u * c - v * s, o.cy + u * s + v * c) for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]

    def _plot(self, m: dict, ok: bool) -> list[str]:
        """map3d.png (3-D view: map, flight paths, PoIs, failures, jamming) + map2d.png (top view)."""
        import types

        import matplotlib
        matplotlib.use("Agg")
        # A pip matplotlib next to Ubuntu's python3-matplotlib: the apt mpl_toolkits (a regular
        # package) would win over the pip one (a namespace package) and fail to import.
        # Bind mpl_toolkits to the copy that belongs to the matplotlib actually loaded.
        own = os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)), "mpl_toolkits")
        if os.path.isdir(own) and "mpl_toolkits.mplot3d" not in sys.modules:
            pkg = types.ModuleType("mpl_toolkits")
            pkg.__path__ = [own]
            sys.modules["mpl_toolkits"] = pkg
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        colors = {u["id"]: tuple(u["color"][:3]) for u in self.sc["uavs"]}
        pois = m.get("pois", {})
        xs = [p[1] for tr in self.track.values() for p in tr] + [self.gcs_pos[0]] + [v["true_pos_m"][0] for v in pois.values()]
        ys = [p[2] for tr in self.track.values() for p in tr] + [self.gcs_pos[1]] + [v["true_pos_m"][1] for v in pois.values()]
        x0, x1, y0, y1 = min(xs) - 25, max(xs) + 25, min(ys) - 25, max(ys) + 25
        zmax = max([p[3] for tr in self.track.values() for p in tr] + [o.height for o in (self.map.obstacles if self.map else [])]
                   + [20.0]) + 10
        obstacles = [o for o in (self.map.obstacles if self.map else [])
                     if x0 - 30 <= o.cx <= x1 + 30 and y0 - 30 <= o.cy <= y1 + 30]
        failures = {e["src"]: e["t"] for e in self._ev("UAV_FAILED")}
        jams = [r for r in m.get("disturbances", []) if r["type"] == "jamming" and isinstance(r.get("center_m"), list)]
        title = f"Level {self.sc['level']} {self.sc['name']} - {'PASS' if ok else 'FAIL'}"
        out = []

        def segments(track):
            seg, cur = [], None
            for p in track:
                if cur is None or p[4] != cur[0]:
                    if cur is not None:
                        seg.append(cur)
                    cur = (p[4], [p] if cur is None else [cur[1][-1], p])
                else:
                    cur[1].append(p)
            if cur is not None:
                seg.append(cur)
            return seg

        # ------------------------------------------------------------ 3-D
        fig = plt.figure(figsize=(13, 8.5))
        ax = fig.add_subplot(111, projection="3d")
        for o in obstacles:
            fp = self._footprint(o)
            top = [(x, y, o.height) for x, y in fp]
            bottom = [(x, y, 0.0) for x, y in fp]
            faces = [top] + [[bottom[i], bottom[(i + 1) % len(fp)], top[(i + 1) % len(fp)], top[i]] for i in range(len(fp))]
            ax.add_collection3d(Poly3DCollection(faces, facecolor=o.color, edgecolor=(0.25, 0.25, 0.25, 0.6),
                                                 linewidths=0.4, alpha=0.55))
            ax.text(o.cx, o.cy, o.height + 2, o.id, fontsize=7, color="0.25")
        if x0 < 75 < x1:
            ax.plot([75, 75], [max(y0, -500), min(y1, 500)], [0, 0], color=(0.9, 0.75, 0.1), lw=1.5)
        gx, gy, gz = self.gcs_pos
        ax.plot([gx, gx], [gy, gy], [0, gz], color="k", lw=2.5)
        ax.scatter([gx], [gy], [gz], marker="^", s=70, c="k")
        ax.text(gx, gy, gz + 3, "GCS", fontsize=8)
        for pid, v in pois.items():
            x, y = v["true_pos_m"][:2]
            ok_poi = v["surveyed"] or v["completed_at_gcs"]
            ax.scatter([x], [y], [0.5], marker="*" if v["hidden"] else "s", s=110 if v["hidden"] else 60,
                       c="magenta" if v["hidden"] else "red", edgecolors="k" if ok_poi else "none", depthshade=False)
            ax.text(x, y, 3, pid + ("" if ok_poi else " (missed)"), fontsize=8, color="darkred")
        for u, track in self.track.items():
            if not track:
                continue
            first = True
            for up, pts in segments(track):
                ax.plot([p[1] for p in pts], [p[2] for p in pts], [p[3] for p in pts], color=colors[u],
                        lw=1.8 if up else 1.2, ls="-" if up else ":", alpha=1.0 if up else 0.8,
                        label=f"{u} ({self.sc['uavs'][self.uavs.index(u)]['cruise_alt_m']:.0f} m)" if first else None)
                first = False
            if u in failures:
                last = track[-1]
                ax.scatter([last[1]], [last[2]], [max(last[3], 0.5)], marker="X", s=140, c="k")
                ax.text(last[1], last[2], last[3] + 4, f"{u} failed", fontsize=8)
        for j in jams:
            th = [2 * math.pi * k / 40 for k in range(41)]
            ax.plot([j["center_m"][0] + j["radius_m"] * math.cos(a) for a in th],
                    [j["center_m"][1] + j["radius_m"] * math.sin(a) for a in th], [0.3] * 41,
                    color="purple", ls="--", lw=1.2)
            ax.text(j["center_m"][0], j["center_m"][1], 2, "jamming", color="purple", fontsize=8)
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_zlim(0, zmax)
        ax.set_box_aspect((x1 - x0, y1 - y0, 2.0 * zmax))
        ax.set_xlabel("East (m)", labelpad=8)
        ax.set_ylabel("North (m)", labelpad=8)
        ax.set_zlabel("Height (m)", labelpad=4)
        ax.view_init(elev=30, azim=-62)
        fig.subplots_adjust(left=0.0, right=0.94, bottom=0.02, top=0.95)
        ax.set_title(title + "   (solid: GCS link up, dotted: out of contact)", fontsize=10)
        ax.legend(loc="upper left", fontsize=8)
        path = os.path.join(self.run_dir, "map3d.png")
        fig.savefig(path, dpi=130, bbox_inches="tight")
        plt.close(fig)
        out.append(path)

        # ------------------------------------------------------------ top view
        fig, ax = plt.subplots(figsize=(10, 8))
        for o in obstacles:
            fp = self._footprint(o)
            ax.fill([p[0] for p in fp], [p[1] for p in fp], color=o.color, alpha=0.8, ec="0.3", lw=0.5)
            ax.text(o.cx, o.cy, f"{o.id}\n{o.height:.0f} m", ha="center", va="center", fontsize=7)
        th = [2 * math.pi * k / 90 for k in range(91)]
        ax.plot([gx + 100 * math.cos(a) for a in th], [gy + 100 * math.sin(a) for a in th], color="0.5", ls="--",
                lw=0.8, label="100 m radio range (GCS)")
        ax.plot([gx], [gy], "k^", ms=9)
        ax.text(gx + 2, gy + 3, "GCS", fontsize=8)
        for u, track in self.track.items():
            first = True
            for up, pts in segments(track):
                ax.plot([p[1] for p in pts], [p[2] for p in pts], color=colors[u], lw=1.6 if up else 1.0,
                        ls="-" if up else ":", label=u if first else None)
                first = False
            if u in failures and track:
                ax.plot([track[-1][1]], [track[-1][2]], "kX", ms=11)
        for pid, v in pois.items():
            x, y = v["true_pos_m"][:2]
            ax.plot([x], [y], "m*" if v["hidden"] else "rs", ms=12 if v["hidden"] else 8)
            ax.text(x + 3, y + 3, pid, fontsize=8, color="darkred")
        for j in jams:
            ax.add_patch(plt.Circle(j["center_m"][:2], j["radius_m"], color="purple", fill=False, ls="--"))
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
        ax.set_title(title + " - top view", fontsize=10)
        path = os.path.join(self.run_dir, "map2d.png")
        fig.savefig(path, dpi=120, bbox_inches="tight")
        plt.close(fig)
        out.append(path)
        return out

    def _finish(self, reason: str) -> None:
        self.done = True
        self._emit("RUN_END", reason=reason)
        m = self._metrics()
        if self._ev("DISTURBANCE"):
            m["disturbances"] = self._disturbance_report()
        criteria = self._criteria(m)
        ok = all(c["pass"] for c in criteria)
        hidden_plan = os.path.join(self.run_dir, "hidden_plan.json")
        summary = {"level": self.sc["level"], "name": self.sc["name"], "pass": ok, "end_reason": reason,
                   "map": self.sc.get("map"), "criteria": criteria, "metrics": m,
                   "hidden_plan": json.load(open(hidden_plan, encoding="utf-8")) if os.path.isfile(hidden_plan) else None}
        try:
            summary["figures"] = self._plot(m, ok)
        except Exception as exc:  # a figure must never cost the verdict
            self.get_logger().error(f"figure failed: {type(exc).__name__}: {exc}")
        with open(os.path.join(self.run_dir, "summary.json"), "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2)
        lines = ["", "=" * 76, f" LEVEL {self.sc['level']} {self.sc['name']}: {'PASS' if ok else 'FAIL'}   ({reason})",
                 f" sim {m['sim_time_s']} s, wall {m['wall_time_s']} s, real-time factor {m['real_time_factor']}",
                 "=" * 76]
        for c in criteria:
            lines.append(f"  {'PASS' if c['pass'] else 'FAIL'}  {c['check']:<46} {c['value']}")
        for r in m.get("disturbances", []):
            extra = {k: v for k, v in r.items() if k not in ("t", "type", "target", "hidden", "handled", "response")
                     and v is not None}
            lines.append(f"  {'ok ' if r['handled'] else 'NOT'} {'hidden ' if r['hidden'] else 'planned'} "
                         f"{r['type']:<12} -> {r['target'] or '-':<6} {json.dumps(extra)[:110]}")
        lines.append("=" * 76)
        self.get_logger().info("\n".join(lines))
        self.ev_fh.close()
        self.ts_fh.close()
        self.exit_code = 0 if ok else 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
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
