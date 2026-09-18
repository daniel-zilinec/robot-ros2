#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np



class RoadFollower(Node):
    def __init__(self):
        super().__init__('road_follower')
        self.bridge = CvBridge()

        self.declare_parameter('show_local_window', False)  # requires a display (DISPLAY/xcb); off for headless RPi5
        self.show_local_window = self.get_parameter('show_local_window').value

        self.declare_parameter('target_processing_hz', 10.0)  # camera runs faster than this is actually needed
        self.min_frame_interval_s = 1.0 / self.get_parameter('target_processing_hz').value
        self.last_processed_s = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos
        )


        # Optional: publish processed image (for foxglove on your laptop)
        self.pub = self.create_publisher(CompressedImage, '/road_follower/debug_image/compressed', 10)
        self.error_pub = self.create_publisher(Float32, '/road_follower/error', 10)

        # Only process bottom part of frame (road is below horizon)
        self.roi_top_fraction = 0.05

    def image_callback(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self.last_processed_s < self.min_frame_interval_s:
            return  # drop frame — processing every camera frame isn't needed and pegs the CPU
        self.last_processed_s = now_s

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # ── ROI: only look at the bottom part of the image ──────────────
        roi_y = int(h * self.roi_top_fraction)
        roi = frame[roi_y:h, 0:w]

        # Blur to remove small specks (leaves, stones) before color thresholding
        # medianBlur requires an odd kernel size
        roi = cv2.medianBlur(roi, 9)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV separates:
        # Hue — the actual color (0–180 in OpenCV: red→yellow→green→cyan→blue→purple→red)
        # Saturation — how vivid/pure the color is
        # Value — brightness

        # ── Grass mask: green hues ───────────────────────────────────────
        # Tweak these ranges to your environment!
        grass_hue = 60  # approximate hue for green in OpenCV HSV (0-180)
        grass_hue_delta = 35  # allowable deviation from the central green hue
        grass_lower = np.array([grass_hue - grass_hue_delta, 30, 40])
        grass_upper = np.array([grass_hue + grass_hue_delta, 255, 255])
        grass_mask = cv2.inRange(hsv, grass_lower, grass_upper)

        # ── Road mask: low saturation (gray/gravel/asphalt) ─────────────
        road_lower = np.array([0, 0, 50])
        road_upper = np.array([180, 50, 255])
        road_mask = cv2.inRange(hsv, road_lower, road_upper)

        # Morphological opening: erase small isolated blobs (leaves, stones) left in the masks
        noise_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (20, 20))
        grass_mask = cv2.morphologyEx(grass_mask, cv2.MORPH_OPEN, noise_kernel)
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_OPEN, noise_kernel)

        # Exclude pixels classified as grass from the road mask
        road_mask = cv2.bitwise_and(road_mask, cv2.bitwise_not(grass_mask))
        road_mask = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, noise_kernel)

        # ── Road column histogram curve ──────────────────────────────────────────
        # For each column, length of the unbroken run of road pixels from the bottom row upward
        road_from_bottom = road_mask[::-1, :] != 0
        is_zero = ~road_from_bottom
        first_zero_idx = np.argmax(is_zero, axis=0)
        no_zero = ~is_zero.any(axis=0)  # column is road all the way up
        col_sums = np.where(no_zero, road_mask.shape[0], first_zero_idx).astype(np.float32)  # shape: (w,)
        col_sums = col_sums ** 5  # exaggerate longer runs over shorter ones

        # ── Road center estimation (needed for /road_follower/error, always run) ──
        total_road_weight = col_sums.sum()
        road_center = None
        error = None
        if total_road_weight > 0:
            road_center = int(np.average(np.arange(w), weights=col_sums))
            frame_center = w // 2
            error = float(np.clip((road_center - frame_center) / (w / 2), -1.0, 1.0))
            self.error_pub.publish(Float32(data=error))

        # ── Debug overlay image: skip entirely if nobody is viewing it (Foxglove or local window) ──
        # (tinting/curve-drawing/JPEG-encode is pure visualization, not needed for control)
        if self.pub.get_subscription_count() == 0 and not self.show_local_window:
            return

        display = frame.copy()
        display[roi_y:h][grass_mask > 0] = [0, 200, 0]    # green tint on grass
        display[roi_y:h][road_mask > 0]  = [0, 0, 200]    # red tint on road

        if col_sums.max() > 0:
            max_curve_height = 120  # max height of the curve in pixels — tune to taste

            # Normalize to pixel height
            col_sums_norm = (col_sums / col_sums.max() * max_curve_height).astype(int)

            # Build array of (x, y) points — curve sits at the bottom of the frame
            # higher column sum → point goes higher (lower y value)
            points = np.array([
                [x, h - col_sums_norm[x]]
                for x in range(w)
            ], dtype=np.int32)

            # Draw as a connected polyline
            cv2.polylines(display, [points], isClosed=False, color=(0, 255, 255), thickness=2)

            # Optional: fill under the curve for better visibility
            # (draws a vertical line from the bottom up to each curve point)
            for x in range(0, w, 3):   # every 3rd column to save CPU
                if col_sums_norm[x] > 2:
                    cv2.line(display,
                            (x, h),
                            (x, h - col_sums_norm[x]),
                            (0, 180, 180), 1)

        if road_center is not None:
            cv2.line(display, (road_center + 0, roi_y),
                               (road_center, h), (255, 255, 0), 2)
            cv2.line(display, (frame_center, roi_y),
                               (frame_center, h), (255, 0, 255), 1)
            cv2.putText(display, f'Error: {error:+.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            cv2.putText(display, 'ROAD LOST', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # ── Local display (optional — needs a real display attached) ─────
        if self.show_local_window:
            cv2.imshow('Road Follower', display)
            cv2.waitKey(1)   # ← MUST have this or window won't update

        # ── Publish processed image (for remote Foxglove) ─────
        jpeg_quality = 60    # 0-100, lower = smaller file, more artifacts
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
        success, buffer = cv2.imencode('.jpg', display, encode_params)

        if success:
            msg_out = CompressedImage()
            msg_out.header = msg.header   # reuse timestamp + frame_id from input
            msg_out.format = 'jpeg'
            msg_out.data = buffer.tobytes()
            self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = RoadFollower()
    try:
        rclpy.spin(node)
    finally:
        if node.show_local_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
