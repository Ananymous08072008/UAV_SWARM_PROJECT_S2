"""A node's attachment to the emulated mesh radio (radio_net).

    send(dst, kind, payload)                  best effort (heartbeats, status)
    send(dst, kind, payload, reliable=True)   ARQ: retransmitted every retry_s until the
                                              receiver ACKs it - store-and-forward for free
                                              while the link is down

Every frame carries a per-sender sequence number (PDR accounting) and the sim time it
was created (end-to-end latency, measured at the receiver and on the ACK).
"""
from __future__ import annotations

import json
from collections import OrderedDict
from typing import Callable, Optional

from uavx_interfaces.msg import RadioPacket

from .common import RADIO_QOS, dumps, now_s


class RadioEndpoint:
    def __init__(self, node, node_id: str,
                 on_receive: Callable[[RadioPacket, dict], None],
                 on_acked: Optional[Callable[[dict, float], None]] = None,
                 retry_s: float = 1.0, max_retry_s: float = 4.0, callback_group=None):
        self.node = node
        self.id = node_id
        self.on_receive = on_receive
        self.on_acked = on_acked
        self.retry_s = retry_s
        self.max_retry_s = max_retry_s
        self.enabled = True
        self._seq = 0
        self._msg_counter = 0
        self.outbox: "OrderedDict[str, dict]" = OrderedDict()
        self._seen: "OrderedDict[tuple[str, str], float]" = OrderedDict()
        self.stats = {"tx": 0, "rx": 0, "retx": 0, "dup": 0, "acked": 0}
        self._pub = node.create_publisher(RadioPacket, "/radio/tx", RADIO_QOS)
        node.create_subscription(RadioPacket, f"/{node_id}/radio/rx", self._on_rx, RADIO_QOS,
                                 callback_group=callback_group)

    # ------------------------------------------------------------- sending
    def send(self, dst: str, kind: str, payload: dict, reliable: bool = False) -> Optional[str]:
        if not self.enabled:
            return None
        msg_id = ""
        stamp = self.node.get_clock().now().to_msg()
        if reliable:
            self._msg_counter += 1
            msg_id = f"{self.id}-{self._msg_counter}"
            self.outbox[msg_id] = {"dst": dst, "kind": kind, "payload": payload, "stamp": stamp,
                                   "first_t": now_s(self.node), "attempt": 1,
                                   "next_t": now_s(self.node) + self.retry_s}
        self._publish(dst, kind, payload, msg_id, stamp, 1)
        return msg_id or None

    def _publish(self, dst, kind, payload, msg_id, stamp, attempt) -> None:
        self._seq += 1
        pkt = RadioPacket()
        pkt.src = self.id
        pkt.dst = dst
        pkt.seq = self._seq
        pkt.kind = kind
        pkt.msg_id = msg_id
        pkt.payload = payload if isinstance(payload, str) else dumps(payload)
        pkt.stamp = stamp
        pkt.attempt = min(int(attempt), 255)
        pkt.hops = 0
        pkt.route = [self.id]
        self._pub.publish(pkt)
        self.stats["tx"] += 1

    def tick(self) -> None:
        """Retransmit reliable frames whose ACK is overdue (call from a timer)."""
        if not self.enabled:
            return
        t = now_s(self.node)
        for msg_id, e in list(self.outbox.items()):
            if t >= e["next_t"]:
                e["attempt"] += 1
                backoff = min(self.retry_s * (1.5 ** (e["attempt"] - 2)), self.max_retry_s)
                e["next_t"] = t + backoff
                self._publish(e["dst"], e["kind"], e["payload"], msg_id, e["stamp"], e["attempt"])
                self.stats["retx"] += 1

    def flush(self) -> int:
        """Retransmit everything still unacknowledged on the next tick (link just came back)."""
        t = now_s(self.node)
        for e in self.outbox.values():
            e["next_t"] = t
        return len(self.outbox)

    def cancel(self, predicate: Callable[[dict], bool]) -> int:
        dropped = [k for k, e in self.outbox.items() if predicate(e)]
        for k in dropped:
            del self.outbox[k]
        return len(dropped)

    @property
    def pending(self) -> int:
        return len(self.outbox)

    # ------------------------------------------------------------- receiving
    def _on_rx(self, pkt: RadioPacket) -> None:
        if not self.enabled:
            return
        self.stats["rx"] += 1
        try:
            payload = json.loads(pkt.payload) if pkt.payload else {}
        except ValueError:
            payload = {}
        if pkt.kind == "ACK":
            entry = self.outbox.pop(payload.get("msg_id", ""), None)
            if entry is not None:
                self.stats["acked"] += 1
                if self.on_acked:
                    self.on_acked(entry, now_s(self.node) - entry["first_t"])
            return
        if pkt.msg_id:
            self._publish(pkt.src, "ACK", {"msg_id": pkt.msg_id}, "", self.node.get_clock().now().to_msg(), 1)
            key = (pkt.src, pkt.msg_id)
            if key in self._seen:
                self.stats["dup"] += 1
                return
            self._seen[key] = now_s(self.node)
            while len(self._seen) > 2000:
                self._seen.popitem(last=False)
        self.on_receive(pkt, payload)


def packet_age_s(node, pkt: RadioPacket) -> float:
    """Sim seconds since the sender created this frame (end-to-end latency)."""
    sent = pkt.stamp.sec + pkt.stamp.nanosec * 1e-9
    return now_s(node) - sent
