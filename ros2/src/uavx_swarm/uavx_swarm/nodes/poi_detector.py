"""PoI detector - the companion computer's vision pipeline (Levels 2-4).

Finds the red PoI targets in the nadir survey camera and geolocates them on the ground:

  1. HSV threshold for red, connected components, drop blobs smaller than min_area_px
  2. pixel -> ground with a pinhole model (camera_info K), the camera height from the
     downward rangefinder (EKF altitude as fallback) and the autopilot's yaw:
        forward = -(v - cy) * h / fy     (image "up" is the nose)
        left    = -(u - cx) * h / fx     (image "right" is the right side)
     rotated into the world ENU frame about the UAV's EKF position
  3. publish uavx_interfaces/PoiDetection on /<uav>/poi_detections and save a snapshot
     (the imagery that is reported to the GCS) to <run_dir>/images/

Frames taken while the airframe is tilted more than max_tilt_deg are skipped (the flat
projection assumes a level camera).
"""
from __future__ import annotations

import math
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from uavx_interfaces.msg import PoiDetection

from ..ap_client import ArduPilotClient
from ..common import EVENT_QOS, SENSOR_QOS, load_scenario, now_s, spin, uav_by_id


class PoiDetector(Node):
    def __init__(self):
        super().__init__("poi_detector")
        self.declare_parameter("scenario", "level2")
        self.declare_parameter("run_dir", ".")
        self.declare_parameter("uav_id", "uav1")
        self.declare_parameter("min_area_px", 120)
        self.declare_parameter("max_tilt_deg", 12.0)
        self.declare_parameter("process_period_s", 0.4)
        self.declare_parameter("snapshot_period_s", 4.0)
        sc = load_scenario(self.get_parameter("scenario").value)
        self.uav = uav_by_id(sc, self.get_parameter("uav_id").value)
        self.id = self.uav["id"]
        self.min_area = int(self.get_parameter("min_area_px").value)
        self.max_tilt = math.radians(float(self.get_parameter("max_tilt_deg").value))
        self.period = float(self.get_parameter("process_period_s").value)
        self.snap_period = float(self.get_parameter("snapshot_period_s").value)
        self.img_dir = os.path.join(self.get_parameter("run_dir").value, "images")
        os.makedirs(self.img_dir, exist_ok=True)

        self.ap = ArduPilotClient(self, self.uav["sysid"])
        self.K = None
        self.range_m = None
        self.last_proc = -1e9
        self.last_snap = -1e9
        self.snap_spots: list[tuple[float, float]] = []   # every target photographed so far
        self.frames = 0
        self.create_subscription(Image, f"/{self.id}/camera/image", self._on_image, SENSOR_QOS)
        self.create_subscription(CameraInfo, f"/{self.id}/camera/camera_info", self._on_info, SENSOR_QOS)
        self.create_subscription(LaserScan, f"/{self.id}/rangefinder", self._on_range, SENSOR_QOS)
        self.pub = self.create_publisher(PoiDetection, f"/{self.id}/poi_detections", EVENT_QOS)

    def _on_info(self, msg: CameraInfo) -> None:
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])   # fx, fy, cx, cy

    def _on_range(self, msg: LaserScan) -> None:
        r = msg.ranges[0] if msg.ranges else float("nan")
        self.range_m = float(r) if math.isfinite(r) and 0.3 < r < msg.range_max else None

    def _on_image(self, msg: Image) -> None:
        t = now_s(self)
        if t - self.last_proc < self.period or self.K is None or not self.ap.has_position or self.ap.pose is None:
            return
        self.last_proc = t
        e0, n0, alt = self.ap.position
        height = self.range_m if self.range_m is not None else alt
        if height < 5.0:
            return
        q = self.ap.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            return
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
        fx, fy, cx, cy = self.K
        yaw = self.ap.yaw
        found = []
        for i in range(1, count):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < self.min_area:
                continue
            u, v = centroids[i]
            fwd = -(v - cy) * height / fy
            left = -(u - cx) * height / fx
            east = e0 + fwd * math.cos(yaw) - left * math.sin(yaw)
            north = n0 + fwd * math.sin(yaw) + left * math.cos(yaw)
            found.append((east, north, area, u, v))
        if not found:
            return
        image_path = ""
        new_target = any(all(math.hypot(e - se, n - sn) > 8.0 for se, sn in self.snap_spots)
                         for e, n, *_ in found)
        if new_target or t - self.last_snap >= self.snap_period:
            self.last_snap = t
            self.snap_spots.extend((e, n) for e, n, *_ in found)
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for east, north, area, u, v in found:
                cv2.drawMarker(bgr, (int(u), int(v)), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
                cv2.putText(bgr, f"({east:.1f}, {north:.1f})", (int(u) + 14, int(v) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(bgr, f"{self.id}  t={t:.1f}s  h={height:.1f}m", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            image_path = os.path.join(self.img_dir, f"{self.id}_{t:08.1f}.png")
            cv2.imwrite(image_path, bgr)
        for east, north, area, _, _ in found:
            det = PoiDetection()
            det.header.stamp = msg.header.stamp
            det.header.frame_id = "world_enu"
            det.uav_id = self.id
            det.east_m = float(east)
            det.north_m = float(north)
            det.pixel_area = float(area)
            det.altitude_m = float(height)
            det.image_path = image_path
            self.pub.publish(det)


def main(args=None):
    rclpy.init(args=args)
    spin(PoiDetector())


if __name__ == "__main__":
    main()
