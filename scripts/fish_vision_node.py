#!/usr/bin/python3
"""
Dual Computer Vision Node: Colored Buoy Guidance + Fish Demand Analysis
=======================================================================
Subscribes to:
  - /camera/image_raw (sensor_msgs/Image)
  - /esp32/sensors_filtered (std_msgs/String JSON, for HUD water status overlay)
  - /brain/state (std_msgs/String, for HUD FSM state overlay)
  - /aqua/target_buoy (std_msgs/String, optional locked buoy color highlight)

Performs two real-time OpenCV pipelines on the forward/downward camera stream:
  1. Colored Floating Buoy Tracker (GREEN, YELLOW, RED, MAGENTA) that drives the robot
     purely via camera visual servoing.
  2. Fish Aggregation, Zone Pad, and Motion Agitation Analysis -> Feeding Demand Score (0-100%).

Publishes:
  - /vision/buoy_detections (std_msgs/String JSON)
  - /vision/feeding_demand (std_msgs/Float32)
  - /vision/debug_image (sensor_msgs/Image) with buoy crosshairs, fish boxes & telemetry HUD.
"""

import json
from typing import Dict, Optional, Tuple

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class FishVisionNode(Node):
    """OpenCV node for colored-buoy visual guidance and fish feeding demand scoring."""

    # (hsv_ranges_list, bgr_draw_color)
    BUOY_COLOR_SPECS: Dict[str, Tuple[list, Tuple[int, int, int]]] = {
        "GREEN": (
            [(np.array([44, 115, 65], np.uint8), np.array([76, 255, 255], np.uint8))],
            (40, 235, 60),
        ),
        "YELLOW": (
            [(np.array([23, 125, 85], np.uint8), np.array([38, 255, 255], np.uint8))],
            (0, 235, 255),
        ),
        "RED": (
            [
                (np.array([0, 140, 70], np.uint8), np.array([6, 255, 255], np.uint8)),
                (np.array([172, 140, 70], np.uint8), np.array([180, 255, 255], np.uint8)),
            ],
            (50, 50, 255),
        ),
        "MAGENTA": (
            [(np.array([142, 125, 70], np.uint8), np.array([170, 255, 255], np.uint8))],
            (235, 60, 235),
        ),
    }

    def __init__(self) -> None:
        super().__init__("fish_vision_node")

        self.bridge = CvBridge()
        self.prev_gray: Optional[np.ndarray] = None
        self.smoothed_demand: float = 0.0
        self.ema_alpha: float = 0.35
        self.locked_buoy_color: str = ""

        # Latest water telemetry & robot state for HUD overlay
        self.water_telemetry = {
            "weight": 5000.0,
            "ph": 7.60,
            "do": 6.80,
            "turbidity": 33.0,
            "turbidity_pct": 22.0,
            "temp": 28.1,
            "valve": 0,
        }
        self.fsm_state: str = "RANDOM_SEARCH"

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.sensor_sub = self.create_subscription(
            String, "/esp32/sensors_filtered", self.sensor_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, "/brain/state", self.state_callback, 10
        )
        self.target_sub = self.create_subscription(
            String, "/aqua/target_buoy", self.target_callback, 10
        )

        # Publishers
        self.buoy_pub = self.create_publisher(String, "/vision/buoy_detections", 10)
        self.demand_pub = self.create_publisher(Float32, "/vision/feeding_demand", 10)
        self.debug_img_pub = self.create_publisher(Image, "/vision/debug_image", 10)

        self.get_logger().info(
            "Vision Guidance & Fish Demand Node started (/vision/buoy_detections, /vision/feeding_demand)."
        )

    def sensor_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.water_telemetry.update(data)
        except (json.JSONDecodeError, TypeError):
            pass

    def state_callback(self, msg: String) -> None:
        self.fsm_state = msg.data

    def target_callback(self, msg: String) -> None:
        self.locked_buoy_color = msg.data.strip().upper()

    def _detect_buoys(
        self, hsv: np.ndarray, debug_frame: np.ndarray, width: int, height: int
    ) -> dict:
        """Detects Green, Yellow, Red, and Magenta spherical buoys in camera view."""
        total_pixels = float(width * height)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        detected_buoys: Dict[str, dict] = {}

        for color_name, (ranges, draw_bgr) in self.BUOY_COLOR_SPECS.items():
            mask = np.zeros((height, width), dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            best_cnt = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(best_cnt))
            if area < 140.0:
                continue

            x, y, w, h = cv2.boundingRect(best_cnt)
            aspect = float(w) / max(1.0, float(h))
            fill_ratio = area / max(1.0, float(w * h))

            # Spherical floating buoys exhibit compact bounding boxes
            if aspect < 0.40 or aspect > 2.30 or fill_ratio < 0.40:
                continue

            m = cv2.moments(best_cnt)
            if m["m00"] > 1e-3:
                cx = float(m["m10"] / m["m00"])
                cy = float(m["m01"] / m["m00"])
            else:
                cx = x + w * 0.5
                cy = y + h * 0.5

            error_x = (cx - (width * 0.5)) / (width * 0.5)
            area_ratio = area / total_pixels

            detected_buoys[color_name] = {
                "error_x": round(error_x, 4),
                "area_ratio": round(area_ratio, 5),
                "cx": int(cx),
                "cy": int(cy),
                "w": int(w),
                "h": int(h),
            }

            is_locked = color_name == self.locked_buoy_color
            box_thickness = 3 if is_locked else 2
            cv2.rectangle(
                debug_frame, (x, y), (x + w, y + h), draw_bgr, box_thickness
            )

            # Draw aiming reticle on locked buoy
            icx, icy = int(cx), int(cy)
            cv2.circle(debug_frame, (icx, icy), 6, draw_bgr, -1)
            if is_locked:
                cv2.line(debug_frame, (icx - 16, icy), (icx + 16, icy), (255, 255, 255), 2)
                cv2.line(debug_frame, (icx, icy - 16), (icx, icy + 16), (255, 255, 255), 2)

            tag = (
                f"{'TARGET ' if is_locked else ''}{color_name} BUOY "
                f"({area_ratio * 100.0:.1f}% | err:{error_x:+.2f})"
            )
            cv2.putText(
                debug_frame,
                tag,
                (x, max(118, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                draw_bgr,
                2 if is_locked else 1,
                cv2.LINE_AA,
            )

        largest_color = None
        largest_error_x = 0.0
        largest_area_ratio = 0.0
        if detected_buoys:
            largest_color = max(
                detected_buoys.keys(),
                key=lambda c: detected_buoys[c]["area_ratio"],
            )
            largest_error_x = detected_buoys[largest_color]["error_x"]
            largest_area_ratio = detected_buoys[largest_color]["area_ratio"]

        return {
            "detected": len(detected_buoys) > 0,
            "buoys": detected_buoys,
            "largest_color": largest_color,
            "largest_error_x": largest_error_x,
            "largest_area_ratio": largest_area_ratio,
        }

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        height, width = frame.shape[:2]
        total_pixels = float(height * width)
        debug_frame = frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # ---------------------------------------------------------------------
        # 1. Detect Colored Floating Buoys for Vision Guidance
        # ---------------------------------------------------------------------
        buoy_payload = self._detect_buoys(hsv, debug_frame, width, height)
        buoy_msg = String()
        buoy_msg.data = json.dumps(buoy_payload)
        self.buoy_pub.publish(buoy_msg)

        # ---------------------------------------------------------------------
        # 2. Detect Active Fish Contours (Amber-Orange HSV, separated from Red & Yellow)
        # ---------------------------------------------------------------------
        lower_fish = np.array([8, 120, 85], dtype=np.uint8)
        upper_fish = np.array([21, 255, 255], dtype=np.uint8)
        fish_mask = cv2.inRange(hsv, lower_fish, upper_fish)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fish_mask = cv2.morphologyEx(fish_mask, cv2.MORPH_OPEN, kernel)
        fish_mask = cv2.dilate(fish_mask, kernel, iterations=1)

        contours, _ = cv2.findContours(
            fish_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        fish_count = 0
        fish_pixel_area = 0.0
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if 40.0 <= area <= 12000.0:
                fish_count += 1
                fish_pixel_area += area
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 200, 255), 1)
                cv2.putText(
                    debug_frame,
                    f"Fish #{fish_count}",
                    (x, max(115, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.38,
                    (0, 230, 255),
                    1,
                    cv2.LINE_AA,
                )

        # ---------------------------------------------------------------------
        # 3. Detect Feeding Zone Pad Signatures
        # ---------------------------------------------------------------------
        lower_depleted_tray = np.array([86, 100, 95], dtype=np.uint8)
        upper_depleted_tray = np.array([104, 255, 255], dtype=np.uint8)
        depleted_mask = cv2.inRange(hsv, lower_depleted_tray, upper_depleted_tray)

        lower_anoxic = np.array([16, 40, 25], dtype=np.uint8)
        upper_anoxic = np.array([36, 150, 85], dtype=np.uint8)
        anoxic_mask = cv2.inRange(hsv, lower_anoxic, upper_anoxic)

        depleted_ratio = float(np.count_nonzero(depleted_mask)) / total_pixels
        anoxic_ratio = float(np.count_nonzero(anoxic_mask)) / total_pixels

        # ---------------------------------------------------------------------
        # 4. Frame-to-Frame Motion / Agitation Score
        # ---------------------------------------------------------------------
        motion_score = 0.0
        if self.prev_gray is not None and self.prev_gray.shape == gray_blur.shape:
            frame_diff = cv2.absdiff(gray_blur, self.prev_gray)
            _, motion_thresh = cv2.threshold(frame_diff, 16, 255, cv2.THRESH_BINARY)
            motion_ratio = float(np.count_nonzero(motion_thresh)) / total_pixels
            motion_score = min(18.0, motion_ratio * 200.0)
        self.prev_gray = gray_blur

        # ---------------------------------------------------------------------
        # 5. Compute Combined Fish Feeding Demand Score (0-100%)
        # ---------------------------------------------------------------------
        fish_count_score = min(58.0, fish_count * 7.8)
        fish_area_ratio = fish_pixel_area / total_pixels
        fish_area_score = min(22.0, fish_area_ratio * 900.0)
        pad_depletion_score = min(20.0, depleted_ratio * 140.0)

        raw_demand = min(
            100.0,
            max(0.0, fish_count_score + fish_area_score + pad_depletion_score + motion_score),
        )
        self.smoothed_demand = (
            self.ema_alpha * raw_demand + (1.0 - self.ema_alpha) * self.smoothed_demand
        )

        demand_msg = Float32()
        demand_msg.data = float(round(self.smoothed_demand, 2))
        self.demand_pub.publish(demand_msg)

        # ---------------------------------------------------------------------
        # 6. Render Annotated Telemetry HUD on /vision/debug_image
        # ---------------------------------------------------------------------
        zone_summary = "SCANNING OPEN WATER"
        if buoy_payload["detected"]:
            col = self.locked_buoy_color or buoy_payload["largest_color"]
            zone_summary = f"BUOY DETECTED: {col}"
        elif anoxic_ratio > 0.04:
            zone_summary = "MURKY / ANOXIC FLOOR"

        self._draw_hud_overlay(
            debug_frame,
            fish_count=fish_count,
            demand_score=self.smoothed_demand,
            zone_label=zone_summary,
        )

        try:
            out_img_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            out_img_msg.header = msg.header
            self.debug_img_pub.publish(out_img_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to publish debug_image: {exc}")

    def _draw_hud_overlay(
        self,
        canvas: np.ndarray,
        fish_count: int,
        demand_score: float,
        zone_label: str,
    ) -> None:
        h, w = canvas.shape[:2]
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (w, 105), (18, 24, 32), -1)
        cv2.addWeighted(overlay, 0.78, canvas, 0.22, 0, canvas)

        # Center camera guide crosshair vertical tick
        cv2.line(canvas, (w // 2, 108), (w // 2, 128), (120, 220, 255), 1)

        cv2.putText(
            canvas,
            f"VISION GUIDANCE | FSM: {self.fsm_state}",
            (12, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 220, 255),
            2,
            cv2.LINE_AA,
        )

        demand_color = (
            (0, 255, 110)
            if demand_score >= 40.0
            else ((0, 215, 255) if demand_score >= 20.0 else (195, 195, 195))
        )
        cv2.putText(
            canvas,
            f"Fish Detected: {fish_count:02d} | Feeding Demand: {demand_score:5.1f}%",
            (12, 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            demand_color,
            1,
            cv2.LINE_AA,
        )

        bar_x, bar_y, bar_w, bar_h = 12, 54, 260, 12
        cv2.rectangle(canvas, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 90, 90), 1)
        fill_w = int((min(100.0, max(0.0, demand_score)) / 100.0) * bar_w)
        cv2.rectangle(canvas, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), demand_color, -1)

        ph = self.water_telemetry.get("ph", 7.6)
        do_val = self.water_telemetry.get("do", 6.8)
        temp = self.water_telemetry.get("temp", 28.1)
        turb_pct = self.water_telemetry.get("turbidity_pct", 22.0)
        weight = self.water_telemetry.get("weight", 5000.0)
        valve = self.water_telemetry.get("valve", 0)

        water_safe = (do_val >= 5.0) and (6.5 <= ph <= 9.0) and (25.0 <= temp <= 30.0)
        status_text = "WATER: STABLE" if water_safe else "WATER: ACTION NEEDED!"
        status_color = (0, 255, 100) if water_safe else (0, 90, 255)

        cv2.putText(
            canvas,
            f"DO:{do_val:.1f}mg/L | pH:{ph:.1f} | Temp:{temp:.1f}C | Turb:{turb_pct:.0f}% | Hopper:{weight:.0f}g",
            (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{status_text} | VALVE: {'OPEN' if valve else 'CLOSED'} | {zone_label}",
            (12, 99),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            status_color,
            1,
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FishVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
