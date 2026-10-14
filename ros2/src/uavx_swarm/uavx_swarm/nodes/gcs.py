"""Ground Control Station (Levels 3-4): task allocation, re-allocation, link watchdog.

Talks to the swarm only through the emulated mesh radio, so everything it knows can be
stale or missing - which is the point of the resilience levels.

Allocation   PoIs become known when they appear (/scenario/poi). Each pending PoI, highest
             priority first, goes to the available UAV with the earliest finish time
             (its queue + 5 m/s travel + survey time). Priority >= emergency_priority is
             put at the head of the queue (the companion pre-empts its current transit).
             Queues go out as versioned, reliable TASKS frames.
Watchdog     a UAV is heard through its 1 Hz STATUS broadcast. Silence for fail_timeout_s
             is judged against where the UAV last was and the PoI it was flying to:
               predicted link PDR at both >= alive_pdr -> LOST (it should have been heard):
                                                          re-allocate its PoIs now
               otherwise                              ->  OUT_OF_RANGE (presumed alive,
                                                          store-and-forward); declared
                                                          LOST after lost_timeout_s
Recovery     recovery time = declared LOST -> the new owner ACKs its updated queue.
Recall       once every known PoI is done and no new PoI is expected (poi_window_s), or at
             mission_time_s, every UAV is recalled to its pad.
"""
from __future__ import annotations

import json
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
from uavx_interfaces.msg import RadioPacket

from ..common import EVENT_QOS, LATCHED_QOS, EventBus, dist2, load_scenario, now_s, share_dir, spin
from ..radio_endpoint import RadioEndpoint, packet_age_s
from ..radio_model import RadioParams, predict_pdr
from ..world_map import WorldMap

DEFAULTS = {"heartbeat_s": 1.0, "fail_timeout_s": 6.0, "lost_timeout_s": 90.0, "alive_pdr": 0.5,
            "min_battery_pct": 30.0, "emergency_priority": 4, "speed_mps": 5.0, "climb_mps": 2.5,
            "poi_window_s": 0.0, "mission_time_s": 1800.0, "out_of_contact_penalty_s": 30.0}
UNAVAILABLE = {"RETURN", "LANDING", "LANDED", "FAILED", "DONE"}


class Gcs(Node):
    def __init__(self):
        super().__init__("gcs")
        self.declare_parameter("scenario", "level3")
        self.declare_parameter("run_dir", ".")
        self.sc = load_scenario(self.get_parameter("scenario").value)
        self.cfg = dict(DEFAULTS, **(self.sc.get("gcs_logic") or {}))
        self.id = self.sc["gcs"].get("id", "gcs")
        self.pos = tuple(self.sc["gcs"]["position_m"])
        with open(os.path.join(share_dir(), "config", "radio.yaml"), encoding="utf-8") as fh:
            radio = yaml.safe_load(fh) or {}
        radio.update(self.sc.get("radio", {}))
        self.radio_p = RadioParams.from_dict(radio)
        self.map = WorldMap.for_scenario(self.sc)
        self.events = EventBus(self, self.id)
        self.radio = RadioEndpoint(self, self.id, self._on_radio, self._on_acked)

        self.uavs = {}
        for u in self.sc["uavs"]:
            self.uavs[u["id"]] = {"id": u["id"], "sysid": u["sysid"], "pad": tuple(u["pad_m"][:2]),
                                  "alt": float(u["cruise_alt_m"]), "link": "UNKNOWN", "last_seen": None,
                                  "status": {}, "pos": (u["pad_m"][0], u["pad_m"][1], 0.0), "queue": [],
                                  "version": 0, "acked_version": 0, "recall_sent": False}
        self.pois: dict[str, dict] = {}
        self.t0 = None
        self.recovery_pending: list[dict] = []
        self.recalled = False
        self.dirty = False

        self.create_subscription(String, "/scenario/poi", self._on_poi, EVENT_QOS)
        self.create_subscription(String, "/scenario/clock", self._on_clock, LATCHED_QOS)
        self.create_timer(float(self.cfg["heartbeat_s"]), self._heartbeat)
        self.create_timer(0.2, self.radio.tick)
        self.create_timer(1.0, self._supervise)
        self.get_logger().info(f"GCS at {self.pos}: watchdog {self.cfg['fail_timeout_s']} s, "
                               f"emergency priority >= {self.cfg['emergency_priority']}")

    # ---------------------------------------------------------------- scenario feed
    def _on_clock(self, msg: String) -> None:
        self.t0 = json.loads(msg.data).get("t0")

    def _mission_t(self) -> float:
        return now_s(self) - self.t0 if self.t0 is not None else 0.0

    def _on_poi(self, msg: String) -> None:
        p = json.loads(msg.data)
        if p["id"] in self.pois:
            return
        self.pois[p["id"]] = {"id": p["id"], "pos": p["reported_m"][:2], "priority": int(p.get("priority", 1)),
                              "survey_s": float(p.get("survey_s", 20.0)), "status": "PENDING", "owner": None,
                              "known_t": now_s(self)}
        self.events.emit("POI_KNOWN", poi_id=p["id"], pos=p["reported_m"][:2], priority=p.get("priority", 1))
        self.dirty = True

    # ---------------------------------------------------------------- radio
    def _heartbeat(self) -> None:
        self.radio.send("*", "HEARTBEAT", {"t": round(now_s(self), 2), "pois": len(self.pois)})

    def _on_radio(self, pkt: RadioPacket, payload: dict) -> None:
        u = self.uavs.get(pkt.src)
        if u is None:
            return
        t = now_s(self)
        u["last_seen"] = t
        if u["link"] in ("UNKNOWN", "OUT_OF_RANGE", "LOST"):
            if u["link"] == "LOST":
                # its PoIs went to others while it was silent: tell it what it owns now
                self.events.emit("UAV_REJOINED", uav=u["id"], queue=list(u["queue"]))
                self._send_tasks(u)
            elif u["link"] == "OUT_OF_RANGE":
                self.events.emit("UAV_BACK_IN_RANGE", uav=u["id"], silent_s=round(t - u.get("silent_since", t), 1))
            else:
                self.events.emit("UAV_JOINED", uav=u["id"])
            u["link"] = "OK"
            self.dirty = True
        if pkt.kind == "STATUS":
            u["status"] = payload
            u["pos"] = tuple(payload.get("pos", u["pos"]))
            for pid in payload.get("done", []):
                self._mark_done(pid, u["id"])
        elif pkt.kind == "POI_REPORT":
            pid = payload.get("poi_id")
            latency = t - float(payload.get("detect_t", t))
            poi = self.pois.get(pid)
            if poi and poi["status"] != "DONE":
                poi["status"] = "DETECTED"
            self.events.emit("POI_RECEIVED", poi_id=pid, uav=u["id"], latency_s=round(latency, 2),
                             frame_age_s=round(packet_age_s(self, pkt), 2), attempts=int(pkt.attempt),
                             estimate=payload.get("estimate"), image=payload.get("image"))
        elif pkt.kind == "TASK_DONE":
            self._mark_done(payload.get("poi_id"), u["id"], payload.get("outcome"))
        elif pkt.kind == "TASK_RELEASE":
            released = payload.get("poi_ids", [])
            for pid in released:
                poi = self.pois.get(pid)
                if poi and poi["status"] != "DONE":
                    poi["status"], poi["owner"] = "PENDING", None
            u["queue"] = [q for q in u["queue"] if q not in released]
            # it is heading home: its last 1 Hz STATUS may still say otherwise, so never hand
            # the released PoIs straight back to it
            u["status"] = dict(u["status"], state="RETURN")
            self.events.emit("TASKS_RELEASED", uav=u["id"], pois=released, reason=payload.get("reason"))
            self._reallocate(u["id"], released, payload.get("reason", "released"))

    def _on_acked(self, entry: dict, latency_s: float) -> None:
        if entry["kind"] != "TASKS":
            return
        uid = entry["dst"]
        version = entry["payload"]["version"]
        self.uavs[uid]["acked_version"] = max(self.uavs[uid]["acked_version"], version)
        for rec in list(self.recovery_pending):
            if rec["to"] == uid and version >= rec["version"]:
                self.events.emit("RECOVERED", lost=rec["lost"], to=uid, pois=rec["pois"],
                                 recovery_s=round(now_s(self) - rec["declared_t"], 2))
                self.recovery_pending.remove(rec)

    def _mark_done(self, pid, uid, outcome=None) -> None:
        poi = self.pois.get(pid)
        if poi is None or poi["status"] == "DONE":
            return
        poi["status"] = "DONE"
        poi["owner"] = uid
        for u in self.uavs.values():
            u["queue"] = [q for q in u["queue"] if q != pid]
        self.events.emit("POI_COMPLETED", poi_id=pid, uav=uid, outcome=outcome,
                         since_known_s=round(now_s(self) - poi["known_t"], 1))

    # ---------------------------------------------------------------- supervision
    def _supervise(self) -> None:
        t = now_s(self)
        if t <= 0.0:
            return
        for u in self.uavs.values():
            if u["last_seen"] is None or u["link"] == "LOST":
                continue
            silent = t - u["last_seen"]
            state = u["status"].get("state", "")
            if state in ("LANDED", "DONE"):
                continue
            if u["link"] == "OK" and silent > self.cfg["fail_timeout_s"]:
                # Where should it be now? Its last report, and the PoI it was flying to:
                # a UAV heading for a PoI beyond radio range is expected to go quiet.
                expected = self._expected_pdr(u["pos"])
                task = self.pois.get(u["status"].get("task") or "")
                if task:
                    expected = min(expected, self._expected_pdr((task["pos"][0], task["pos"][1], u["alt"])))
                u["silent_since"] = u["last_seen"]
                if expected >= self.cfg["alive_pdr"] or state == "FAILED":
                    self._declare_lost(u, silent, f"silent {silent:.1f} s where the link should be {expected:.2f} PDR")
                else:
                    u["link"] = "OUT_OF_RANGE"
                    self.events.emit("UAV_OUT_OF_RANGE", uav=u["id"], silent_s=round(silent, 1),
                                     last_pos=[round(v, 1) for v in u["pos"]], expected_pdr=round(expected, 2))
            elif u["link"] == "OUT_OF_RANGE" and silent > self.cfg["lost_timeout_s"]:
                self._declare_lost(u, silent, "out of range too long")
        # safety net: unfinished PoIs held by a UAV that can no longer fly them go back to the pool
        for u in self.uavs.values():
            stuck = [pid for pid in u["queue"] if self.pois.get(pid, {}).get("status") != "DONE"]
            if stuck and u["link"] != "LOST" and u["status"].get("state") in UNAVAILABLE:
                u["queue"] = []
                for pid in stuck:
                    self.pois[pid]["status"], self.pois[pid]["owner"] = "PENDING", None
                self.events.emit("TASKS_RECLAIMED", uav=u["id"], pois=stuck, state=u["status"].get("state"))
                self._reallocate(u["id"], stuck, "owner unavailable")
        if self.dirty:
            self.dirty = False
            self._allocate()
        self._maybe_recall()

    def _expected_pdr(self, pos) -> float:
        """Healthy-link PDR between the GCS mast and pos (buildings on the path included)."""
        return predict_pdr(self.radio_p, pos, self.pos, gcs_link=True, world_map=self.map)

    def _declare_lost(self, u: dict, silent: float, why: str) -> None:
        u["link"] = "LOST"
        orphans = [pid for pid in u["queue"] if self.pois.get(pid, {}).get("status") != "DONE"]
        cur = u["status"].get("task")
        if cur and cur not in orphans and self.pois.get(cur, {}).get("status") not in (None, "DONE"):
            orphans.insert(0, cur)
        for pid in orphans:
            self.pois[pid]["status"], self.pois[pid]["owner"] = "PENDING", None
        u["queue"] = []
        self.events.emit("UAV_DECLARED_LOST", uav=u["id"], detection_s=round(silent, 2), reason=why, orphans=orphans)
        self._reallocate(u["id"], orphans, "uav lost")

    def _reallocate(self, from_uav: str, pois: list, reason: str) -> None:
        changed = self._allocate()
        moved = {}
        for pid in pois:
            owner = self.pois.get(pid, {}).get("owner")
            if owner:
                moved[pid] = owner
        self.events.emit("REALLOCATED", lost=from_uav, reason=reason, moved=moved,
                         unassigned=[p for p in pois if p not in moved])
        t = now_s(self)
        for to in set(moved.values()):
            if to in changed:
                self.recovery_pending.append({"lost": from_uav, "to": to, "pois": [p for p, o in moved.items() if o == to],
                                              "version": self.uavs[to]["version"], "declared_t": t})

    def _available(self) -> list[dict]:
        out = []
        for u in self.uavs.values():
            st = u["status"]
            if u["link"] not in ("OK", "OUT_OF_RANGE") or st.get("state") in UNAVAILABLE:
                continue
            if float(st.get("batt", 100.0)) < self.cfg["min_battery_pct"]:
                continue
            out.append(u)
        return out

    def _finish_time(self, u: dict, extra=None) -> tuple[float, tuple]:
        """Seconds until u finishes its queue (+ extra PoI) and where it ends up."""
        sp = self.cfg["speed_mps"]
        t = 0.0
        pos = u["pos"][:2]
        if u["status"].get("state") in (None, "READY", "PREFLIGHT", "BOOT", "ARMING"):
            t += u["alt"] / self.cfg["climb_mps"] + 8.0
        for pid in u["queue"] + ([extra] if extra else []):
            poi = self.pois[pid]
            t += dist2(pos, poi["pos"]) / sp + poi["survey_s"]
            pos = poi["pos"]
        return t, pos

    def _allocate(self) -> set:
        avail = self._available()
        if not avail:
            return set()
        changed = set()
        pending = sorted((p for p in self.pois.values() if p["status"] == "PENDING"),
                         key=lambda p: (-p["priority"], p["known_t"]))
        for poi in pending:
            best, best_t = None, math.inf
            for u in avail:
                if poi["priority"] >= self.cfg["emergency_priority"]:
                    cur = u["status"].get("task")
                    # head of queue: only what it is surveying right now comes first
                    finish = dist2(u["pos"][:2], poi["pos"]) / self.cfg["speed_mps"]
                    if u["status"].get("state") == "SURVEY" and cur in self.pois:
                        finish += self.pois[cur]["survey_s"]
                else:
                    finish, _ = self._finish_time(u, poi["id"])
                # connectivity-aware: prefer a UAV whose flight level keeps the GCS in
                # contact over this PoI (live report) to one that must store-and-forward
                if self._expected_pdr((poi["pos"][0], poi["pos"][1], u["alt"])) < self.cfg["alive_pdr"]:
                    finish += self.cfg["out_of_contact_penalty_s"]
                if finish < best_t:
                    best, best_t = u, finish
            if best is None:
                continue
            if poi["priority"] >= self.cfg["emergency_priority"]:
                cur = best["status"].get("task")
                head = [cur] if best["status"].get("state") == "SURVEY" and cur in best["queue"] else []
                best["queue"] = head + [poi["id"]] + [q for q in best["queue"] if q not in head]
            else:
                best["queue"].append(poi["id"])
            poi["status"], poi["owner"] = "ASSIGNED", best["id"]
            changed.add(best["id"])
            self.events.emit("ASSIGNED", poi_id=poi["id"], uav=best["id"], priority=poi["priority"],
                             eta_s=round(best_t, 1), queue=list(best["queue"]))
        for uid in changed:
            self._send_tasks(self.uavs[uid])
        return changed

    def _send_tasks(self, u: dict) -> None:
        u["version"] += 1
        self.radio.cancel(lambda e: e["kind"] == "TASKS" and e["dst"] == u["id"])   # newest queue wins
        tasks = [{"id": pid, "pos": self.pois[pid]["pos"], "priority": self.pois[pid]["priority"],
                  "survey_s": self.pois[pid]["survey_s"]} for pid in u["queue"]]
        self.radio.send(u["id"], "TASKS", {"version": u["version"], "tasks": tasks}, reliable=True)

    def _maybe_recall(self) -> None:
        if self.recalled or self.t0 is None:
            return
        mt = self._mission_t()
        all_done = bool(self.pois) and all(p["status"] == "DONE" for p in self.pois.values())
        if (all_done and mt >= self.cfg["poi_window_s"]) or mt >= self.cfg["mission_time_s"]:
            self.recalled = True
            reason = "all PoIs done" if all_done else "mission time"
            self.events.emit("RECALL", reason=reason, mission_t=round(mt, 1),
                             pois_done=sum(p["status"] == "DONE" for p in self.pois.values()), pois=len(self.pois))
            for u in self.uavs.values():
                if u["link"] != "LOST":
                    self.radio.send(u["id"], "RECALL", {"reason": reason}, reliable=True)


def main(args=None):
    rclpy.init(args=args)
    spin(Gcs())


if __name__ == "__main__":
    main()
