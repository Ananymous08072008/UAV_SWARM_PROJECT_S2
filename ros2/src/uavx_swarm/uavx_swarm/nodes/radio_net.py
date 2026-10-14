"""Emulated UAV mesh radio (Levels 3-4) - the "fake radio" sensor/actuator of every node.

Every node (GCS and each UAV's companion) transmits frames on /radio/tx. radio_net
decides, per receiver, whether the frame arrives and when, then publishes it on
/<dst>/radio/rx. Link quality comes from the channel model in radio_model.py (identical
to the Python simulator) evaluated on Gazebo ground-truth positions every 0.5 s, plus
whatever disturbances are active:

    comm_outage  {target, duration_s}            node's radio dead (in and out)
    packet_loss  {target | "*", loss, duration_s} extra random frame loss on its links
    jamming      {center_m, radius_m, attenuation_db, duration_s}
    uav_failure  {target}                        radio dies with the airframe (permanent)

With a 3-D map (scenario `map:`), every building the radio path crosses adds its
attenuation (20 dB for concrete): low UAVs behind buildings lose the link.

Level 4 is single-hop: a frame reaches its destination only over the direct link. (Level
5 adds relaying: companions re-transmit frames addressed to others along a route.)

Outputs: /radio/network_state (2 Hz), /radio/stats (1 Hz, cumulative per link and kind)
and <run_dir>/packets.csv (one row per frame per receiver).
"""
from __future__ import annotations

import csv
import heapq
import json
import math
import os
import random

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from uavx_interfaces.msg import LinkState, NetworkState, RadioPacket

from ..common import (EVENT_QOS, RADIO_QOS, SENSOR_QOS, EventBus, dumps, load_scenario,
                      now_s, share_dir, spin)
from ..radio_model import RadioParams, latency_ms, mean_snr_db, pdr_from_snr
from ..world_map import WorldMap


class RadioNet(Node):
    def __init__(self):
        super().__init__("radio_net")
        self.declare_parameter("scenario", "level4")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("seed", 7)
        sc = load_scenario(self.get_parameter("scenario").value)
        with open(os.path.join(share_dir(), "config", "radio.yaml"), encoding="utf-8") as fh:
            base = yaml.safe_load(fh) or {}
        base.update(sc.get("radio", {}))
        self.p = RadioParams.from_dict(base)
        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed if seed >= 0 else 7)
        self.map = WorldMap.for_scenario(sc)          # buildings attenuate radio paths
        self.gcs_id = sc["gcs"].get("id", "gcs")
        self.nodes = [self.gcs_id] + [u["id"] for u in sc["uavs"]]
        self.pos: dict[str, tuple] = {self.gcs_id: tuple(sc["gcs"]["position_m"])}
        self.links: dict[tuple, dict] = {}
        self.outage: dict[str, float] = {}         # node -> until (inf = permanent)
        self.loss: dict[str, tuple] = {}           # node | "*" -> (loss, until)
        self.jams: list[dict] = []
        self.queue: list = []                      # (deliver_t, counter, dst, packet)
        self.counter = 0
        self.stats: dict[str, dict] = {}
        self.latencies: list[float] = []
        self.events = EventBus(self, "radio")

        run_dir = self.get_parameter("run_dir").value
        os.makedirs(run_dir, exist_ok=True)
        self.csv_fh = open(os.path.join(run_dir, "packets.csv"), "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_fh)
        self.csv.writerow(["t", "src", "dst", "kind", "seq", "msg_id", "attempt", "delivered", "latency_ms",
                           "reason", "pdr", "distance_m"])

        self.rx_pub = {n: self.create_publisher(RadioPacket, f"/{n}/radio/rx", RADIO_QOS) for n in self.nodes}
        self.create_subscription(RadioPacket, "/radio/tx", self._on_tx, RADIO_QOS)
        for u in sc["uavs"]:
            self.create_subscription(Odometry, f"/{u['id']}/odometry", lambda m, u=u["id"]: self._on_odom(u, m), SENSOR_QOS)
        self.create_subscription(String, "/scenario/inject", self._on_inject, EVENT_QOS)
        self.net_pub = self.create_publisher(NetworkState, "/radio/network_state", EVENT_QOS)
        self.stats_pub = self.create_publisher(String, "/radio/stats", EVENT_QOS)
        self.create_timer(self.p.update_interval_s, self._update_links)
        self.create_timer(0.01, self._pump)
        self.create_timer(1.0, self._publish_stats)
        self.get_logger().info(f"radio_net: nodes {self.nodes}, range {self.p.max_range_m:.0f} m, "
                               f"tx {self.p.tx_power_dbm} dBm, PL exponent {self.p.path_loss_exponent}")

    # ---------------------------------------------------------------- inputs
    def _on_odom(self, uav: str, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos[uav] = (p.x, p.y, p.z)

    def _on_inject(self, msg: String) -> None:
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        t = now_s(self)
        kind = ev.get("type")
        until = t + float(ev["duration_s"]) if ev.get("duration_s") else math.inf
        if kind == "comm_outage":
            self.outage[ev["target"]] = until
        elif kind == "uav_failure":
            self.outage[ev["target"]] = math.inf
        elif kind == "packet_loss":
            self.loss[ev.get("target", "*")] = (float(ev.get("loss", 0.5)), until)
        elif kind == "jamming":
            self.jams.append({"center": ev["center_m"], "radius": float(ev.get("radius_m", 50.0)),
                              "db": float(ev.get("attenuation_db", 30.0)), "until": until})
        else:
            return
        self.events.emit("RADIO_IMPAIRMENT", kind=kind, target=ev.get("target"),
                         until=None if until == math.inf else round(until, 1))

    # ---------------------------------------------------------------- channel
    def _impairment(self, a: str, b: str, t: float):
        """(channel multiplier, frame survival, extra loss dB, label) for link a-b at time t.

        Outages and jamming change the channel (and so whether the link is up); injected
        packet loss only drops frames at random - a lossy link stays usable, as it would
        for a real routing layer that has nothing better.
        """
        for n in (a, b):
            if self.outage.get(n, -1.0) > t:
                return 0.0, 0.0, 0.0, "outage"
        survive, label = 1.0, ""
        for key in (a, b, "*"):
            loss, until = self.loss.get(key, (0.0, -1.0))
            if until > t:
                survive *= (1.0 - loss)
                label = "packet_loss"
        extra = 0.0
        for j in self.jams:
            if j["until"] <= t:
                continue
            for n in (a, b):
                p = self.pos.get(n)
                if p and math.dist(p[:2], j["center"][:2]) <= j["radius"]:
                    extra = max(extra, j["db"])
                    label = "jamming"
        return 1.0, survive, extra, label

    def _update_links(self) -> None:
        t = now_s(self)
        if t <= 0.0:
            return
        p = self.p
        msg = NetworkState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.nodes = self.nodes
        for i, a in enumerate(self.nodes):
            for b in self.nodes[i + 1:]:
                if a not in self.pos or b not in self.pos:
                    continue
                mult, survive, extra, label = self._impairment(a, b, t)
                if self.map is not None:
                    blocked = self.map.los_loss_db(self.pos[a], self.pos[b])
                    if blocked > 0.0:
                        extra += blocked
                        label = label or "obstructed"
                snr, d = mean_snr_db(p, self.pos[a], self.pos[b], a == self.gcs_id or b == self.gcs_id, extra)
                if p.fading_sigma_db > 0:
                    snr += self.rng.gauss(0.0, p.fading_sigma_db)
                measured = 0.0 if d > p.max_range_m else pdr_from_snr(p, snr) * mult
                prev = self.links.get((a, b))
                if prev is None:
                    chan, was_up = measured, measured >= p.link_up_pdr
                else:
                    chan, was_up = p.pdr_smoothing * measured + (1 - p.pdr_smoothing) * prev["chan"], prev["up"]
                if measured == 0.0:          # out of range / dead radio: no slow fade-out
                    chan = 0.0
                up = chan >= p.link_down_pdr if was_up else chan >= p.link_up_pdr
                pdr = chan * survive         # what a frame actually experiences
                self.links[(a, b)] = {"chan": chan, "pdr": pdr, "up": up, "d": d, "snr": snr, "label": label,
                                      "lat": latency_ms(p, max(pdr, 0.05))}
                ls = LinkState()
                ls.a, ls.b = a, b
                ls.distance_m = float(d)
                ls.snr_db = float(snr)
                ls.pdr = float(pdr)
                ls.latency_ms = float(self.links[(a, b)]["lat"])
                ls.up = bool(up)
                ls.impairment = label
                msg.links.append(ls)
        self.net_pub.publish(msg)

    def _link(self, a: str, b: str):
        return self.links.get((a, b)) or self.links.get((b, a))

    # ---------------------------------------------------------------- frames
    def _on_tx(self, pkt: RadioPacket) -> None:
        t = now_s(self)
        targets = [n for n in self.nodes if n != pkt.src] if pkt.dst == "*" else [pkt.dst]
        for dst in targets:
            link = self._link(pkt.src, dst)
            key = f"{pkt.src}->{dst}|{pkt.kind}"
            st = self.stats.setdefault(key, {"tx": 0, "delivered": 0})
            st["tx"] += 1
            reason, delivered, lat = "", False, 0.0
            if link is None:
                reason = "unknown_node"
            elif self.outage.get(pkt.src, -1.0) > t or self.outage.get(dst, -1.0) > t:
                reason = "outage"
            elif not link["up"]:
                reason = "no_link"
            elif self.rng.random() > link["pdr"]:
                reason = "lost"
            else:
                delivered = True
                lat = link["lat"] * self.rng.uniform(0.7, 1.3)
                self.counter += 1
                heapq.heappush(self.queue, (t + lat / 1000.0, self.counter, dst, pkt))
                st["delivered"] += 1
                self.latencies.append(lat)
            self.csv.writerow([round(t, 3), pkt.src, dst, pkt.kind, pkt.seq, pkt.msg_id, pkt.attempt, int(delivered),
                               round(lat, 2), reason, round(link["pdr"], 3) if link else "",
                               round(link["d"], 1) if link else ""])

    def _pump(self) -> None:
        t = now_s(self)
        while self.queue and self.queue[0][0] <= t:
            _, _, dst, pkt = heapq.heappop(self.queue)
            if self.outage.get(dst, -1.0) > t:
                continue
            pkt.hops += 1
            self.rx_pub[dst].publish(pkt)

    def _publish_stats(self) -> None:
        lat = sorted(self.latencies[-5000:])
        body = {"links": self.stats,
                "latency_ms": {"mean": round(sum(lat) / len(lat), 2) if lat else None,
                               "p95": round(lat[int(0.95 * (len(lat) - 1))], 2) if lat else None,
                               "count": len(self.latencies)}}
        msg = String()
        msg.data = dumps(body)
        self.stats_pub.publish(msg)
        self.csv_fh.flush()


def main(args=None):
    rclpy.init(args=args)
    node = RadioNet()
    spin(node, on_exit=node.csv_fh.close)


if __name__ == "__main__":
    main()
