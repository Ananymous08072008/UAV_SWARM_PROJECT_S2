"""Inject a disturbance into a running UAV-X level (the Stage 2 "hidden disturbances").

    ros2 run uavx_swarm inject comm_outage uav2 --duration 20
    ros2 run uavx_swarm inject packet_loss uav1 --loss 0.6 --duration 30
    ros2 run uavx_swarm inject packet_loss '*' --loss 0.3 --duration 60
    ros2 run uavx_swarm inject jamming --center 80 0 --radius 50 --attenuation 30 --duration 60
    ros2 run uavx_swarm inject battery uav1 --pct 22
    ros2 run uavx_swarm inject uav_failure uav2
    ros2 run uavx_swarm inject new_poi --id E9 --pos 70 10 --priority 5

The scenario_manager executes it (and logs it as a DISTURBANCE event) exactly like a
scheduled timeline entry.
"""
import argparse
import json
import time

import rclpy
from std_msgs.msg import String

from ..common import EVENT_QOS

TYPES = ["comm_outage", "packet_loss", "jamming", "battery", "uav_failure", "new_poi"]


def main(args=None):
    ap = argparse.ArgumentParser(prog="ros2 run uavx_swarm inject", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("type", choices=TYPES)
    ap.add_argument("target", nargs="?", default=None, help="uav id (or '*' for packet_loss)")
    ap.add_argument("--duration", type=float, help="seconds of sim time (omit = permanent)")
    ap.add_argument("--loss", type=float, default=0.5)
    ap.add_argument("--center", type=float, nargs=2)
    ap.add_argument("--radius", type=float, default=50.0)
    ap.add_argument("--attenuation", type=float, default=30.0)
    ap.add_argument("--pct", type=float)
    ap.add_argument("--id")
    ap.add_argument("--pos", type=float, nargs=2)
    ap.add_argument("--priority", type=int, default=5)
    ap.add_argument("--survey", type=float, default=20.0)
    opts, ros_args = ap.parse_known_args(args)

    ev = {"type": opts.type}
    if opts.target:
        ev["target"] = opts.target
    if opts.duration:
        ev["duration_s"] = opts.duration
    if opts.type == "packet_loss":
        ev["loss"] = opts.loss
        ev.setdefault("target", "*")
    elif opts.type == "jamming":
        if not opts.center:
            ap.error("jamming needs --center E N")
        ev.update(center_m=opts.center, radius_m=opts.radius, attenuation_db=opts.attenuation)
    elif opts.type == "battery":
        if opts.pct is None or not opts.target:
            ap.error("battery needs a target and --pct")
        ev["pct"] = opts.pct
    elif opts.type == "new_poi":
        if not opts.id or not opts.pos:
            ap.error("new_poi needs --id and --pos E N")
        ev.update(id=opts.id, pos_m=opts.pos, priority=opts.priority, survey_s=opts.survey)
    elif not opts.target:
        ap.error(f"{opts.type} needs a target uav")

    rclpy.init(args=ros_args)
    node = rclpy.create_node("uavx_inject")
    pub = node.create_publisher(String, "/scenario/inject_request", EVENT_QOS)
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    msg = String()
    msg.data = json.dumps(ev)
    pub.publish(msg)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    print(("sent: " if pub.get_subscription_count() else "no scenario_manager listening - sent anyway: ") + msg.data)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
