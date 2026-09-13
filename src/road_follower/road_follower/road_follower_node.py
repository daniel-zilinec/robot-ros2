#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RoadFollower(Node):
    def __init__(self):
        super().__init__('road_follower')
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos
        )


        # Optional: publish processed image (for foxglove on your laptop)
        self.pub = self.create_publisher(Image, '/road_follower/debug_image', 10)

        # Only process bottom 50% of frame (road is below horizon)
        self.roi_top_fraction = 0.5

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # ── ROI: only look at the bottom half of the image ──────────────
        roi_y = int(h * self.roi_top_fraction)
        roi = frame[roi_y:h, 0:w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ── Grass mask: green hues ───────────────────────────────────────
        # Tweak these ranges to your environment!
        grass_lower = np.array([25, 40, 40])
        grass_upper = np.array([85, 255, 255])
        grass_mask = cv2.inRange(hsv, grass_lower, grass_upper)

        # ── Road mask: low saturation (gray/gravel/asphalt) ─────────────
        road_lower = np.array([0, 0, 60])
        road_upper = np.array([180, 50, 220])
        road_mask = cv2.inRange(hsv, road_lower, road_upper)

        # ── Overlay on display frame ─────────────────────────────────────
        display = frame.copy()
        display[roi_y:h][grass_mask > 0] = [0, 200, 0]    # green tint on grass
        display[roi_y:h][road_mask > 0]  = [0, 0, 200]    # red tint on road

        # ── Road center estimation ───────────────────────────────────────
        road_cols = np.where(road_mask.sum(axis=0) > 20)[0]  # columns with road
        if len(road_cols) > 0:
            road_center = int(road_cols.mean())
            frame_center = w // 2
            error = road_center - frame_center  # positive = road is right of center

            cv2.line(display, (road_center + 0, roi_y),
                               (road_center, h), (255, 255, 0), 2)
            cv2.line(display, (frame_center, roi_y),
                               (frame_center, h), (255, 0, 255), 1)
            cv2.putText(display, f'Error: {error:+d}px', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            cv2.putText(display, 'ROAD LOST', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # ── Local display ────────────────────────────────────────────────
        cv2.imshow('Road Follower', display)
        cv2.waitKey(1)   # ← MUST have this or window won't update

        # ── Publish processed image (optional, for remote Foxglove) ─────
        self.pub.publish(self.bridge.cv2_to_imgmsg(display, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = RoadFollower()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
