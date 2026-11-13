#!/usr/bin/python3
"""
Camera-Driven & 4x Ultrasonic Autonomous Aquaculture Brain Node (FSM)
=====================================================================
Purely driven by camera vision and 4-side obstacle sensing:
  - Forward/Downward RGB Camera (/vision/buoy_detections & /vision/feeding_demand)
  - 4-Side Waterproof Ultrasonic Sensors (/ultrasonic/front, left, right, rear)
  - ESP32 Water & Weight Telemetry (/esp32/sensors_filtered)

5-State Autonomous FSM:
  1. RANDOM_SEARCH     : Random smooth exploration + 4-side ultrasonic obstacle avoidance
                         until camera spots an active (non-cooldown) colored buoy.
  2. APPROACH_BUOY     : Camera visual servoing toward locked colored buoy, guarded by
                         side & front ultrasonic obstacle checks.
  3. INSPECT_AND_DECIDE: Holds at buoy for 2.5s, reads sensors vs. live safe limits,
                         determines what is missing/needed, and reports to the website.
  4. EXECUTE_FEEDING   : Opens dispenser valve (/feeder_valve_cmd), monitors HX711
                         weight for closed-loop dosing, then closes valve cleanly.
  5. DEPART_BUOY       : Marks visited buoy with cooldown, turns cleanly away using
                         ultrasonic safety, and resumes RANDOM_SEARCH for the next ball.
"""

from enum import Enum
import json
import math
import random
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String


class FSMState(Enum):
    RANDOM_SEARCH = "RANDOM_SEARCH"
    APPROACH_BUOY = "APPROACH_BUOY"
    INSPECT_AND_DECIDE = "INSPECT_AND_DECIDE"
    EXECUTE_FEEDING = "EXECUTE_FEEDING"
    DEPART_BUOY = "DEPART_BUOY"
    PAUSED = "PAUSED"


class AutonomousBrainNode(Node):
    """Pure Camera Visual-Servoing + 4-Side Ultrasonic Obstacle Avoidance FSM."""

    def __init__(self) -> None:
        super().__init__("autonomous_brain_node")

        # Parameters
        self.declare_parameter("target_feed_dose_g", 250.0)
        self.declare_parameter("low_feed_threshold_g", 300.0)
        self.declare_parameter("feeding_demand_threshold_pct", 35.0)
        self.declare_parameter("inspection_duration_sec", 2.5)
        self.declare_parameter("buoy_cooldown_sec", 32.0)

        self.default_dose_g: float = float(
            self.get_parameter("target_feed_dose_g").value
        )
        self.low_feed_threshold_g: float = float(
            self.get_parameter("low_feed_threshold_g").value
        )
        self.demand_threshold_pct: float = float(
            self.get_parameter("feeding_demand_threshold_pct").value
        )
        self.inspect_duration_sec: float = float(
            self.get_parameter("inspection_duration_sec").value
        )
        self.buoy_cooldown_sec: float = float(
            self.get_parameter("buoy_cooldown_sec").value
        )

        # Editable Safe Limits (default Nile Tilapia bounds, live-updatable via /aqua/safe_limits)
        self.safe_limits: Dict[str, float] = {
            "min_do": 5.0,
            "min_ph": 6.5,
            "max_ph": 9.0,
            "min_temp": 25.0,
            "max_temp": 30.0,
            "max_turb_pct": 40.0,
        }

        # State tracking
        self.state: FSMState = FSMState.RANDOM_SEARCH
        self.state_enter_mono: float = time.monotonic()
        self.valve_is_open: bool = False
        self.feeding_start_weight_g: float = 5000.0
        self.planned_dose_g: float = self.default_dose_g
        self.aerators_on: bool = False
        self.phone_alert_active: bool = False

        # Target & cooldown tracking
        self.target_buoy_color: str = ""
        self.requested_seek_color: str = ""
        self.buoy_cooldowns: Dict[str, float] = {}
        self.last_target_seen_mono: float = 0.0
        self.last_visited_buoy: str = "None yet"
        self.buoy_detections: dict = {"detected": False, "buoys": {}}

        # Random wander & ultrasonic avoidance state
        self.wander_angular: float = 0.22
        self.next_wander_change_mono: float = time.monotonic() + 2.5
        self.avoid_turn_dir: float = 1.0
        self.avoid_lock_until_mono: float = 0.0
        self.depart_turn_dir: float = 1.0
        self.post_inspect_wait_until: float = 0.0
        self.decision_completed_in_inspect: bool = False

        # 4-Side Ultrasonic distances (meters)
        self.us_ranges: Dict[str, float] = {
            "front": 4.5,
            "left": 4.5,
            "right": 4.5,
            "rear": 4.5,
        }

        # Live ESP32 sensors & Vision demand
        self.sensors: Dict[str, float] = {
            "weight": 5000.0,
            "ph": 7.60,
            "do": 6.80,
            "turbidity": 33.0,
            "turbidity_pct": 22.0,
            "temp": 28.1,
            "valve": 0,
        }
        self.sensors_received: bool = False
        self.feeding_demand_pct: float = 0.0

        # Structured Action Decision Report (rendered directly on Live Water Quality Web Dashboard)
        self.report_seq: int = 0
        self.latest_report: dict = {
            "seq": 0,
            "fsm_state": self.state.value,
            "overall_status": "STABLE",
            "status_badge": "Stable",
            "banner_title": "All parameters are stable",
            "banner_subtitle": "Robot cruising randomly using camera vision & 4x ultrasonic obstacle avoidance. Aerators stay off to save energy.",
            "action_title": "No action needed",
            "action_steps": [
                "Robot is autonomously searching the pond for colored zone buoys.",
                "Camera vision & all 4 side ultrasonic sensors are active for obstacle avoidance.",
                "Aerators stay off while dissolved oxygen remains above safe limit.",
            ],
            "aerator_state": "OFF · AUTO",
            "aerator_sub": "Start automatically when oxygen is low.",
            "feeder_state": "Scanning",
            "feeder_sub": "Waiting for next colored buoy inspection.",
            "alert_state": "None",
            "alert_sub": "Nothing to report.",
            "target_buoy": "",
            "last_buoy": "Open Pond",
            "dispensed_g": 0.0,
            "planned_dose_g": 0.0,
            "deficiencies": [],
            "event_log": "",
        }
        self.latest_mission_text: str = (
            "Random Pond Search (Camera + 4x Ultrasonic Obstacle Avoidance)"
        )

        # Subscribers
        self.create_subscription(String, "/vision/buoy_detections", self._buoy_cb, 10)
        self.create_subscription(Float32, "/vision/feeding_demand", self._demand_cb, 10)
        self.create_subscription(String, "/esp32/sensors_filtered", self._sensors_cb, 10)
        self.create_subscription(Range, "/ultrasonic/front", lambda m: self._us_cb("front", m), 10)
        self.create_subscription(Range, "/ultrasonic/left", lambda m: self._us_cb("left", m), 10)
        self.create_subscription(Range, "/ultrasonic/right", lambda m: self._us_cb("right", m), 10)
        self.create_subscription(Range, "/ultrasonic/rear", lambda m: self._us_cb("rear", m), 10)
        self.create_subscription(String, "/aqua/safe_limits", self._limits_cb, 10)
        self.create_subscription(String, "/aqua/dispatch_cmd", self._dispatch_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.valve_pub = self.create_publisher(Bool, "/feeder_valve_cmd", 10)
        self.state_pub = self.create_publisher(String, "/brain/state", 10)
        self.target_pub = self.create_publisher(String, "/aqua/target_buoy", 10)
        self.mission_pub = self.create_publisher(String, "/aqua/mission_status", 10)
        self.report_pub = self.create_publisher(String, "/aqua/decision_report", 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            "Camera Vision + 4x Ultrasonic Autonomous Brain initialized. Starting RANDOM_SEARCH."
        )

    @staticmethod
    def _clean_range(msg: Range) -> float:
        r = float(msg.range)
        if math.isnan(r) or math.isinf(r):
            return float(msg.max_range) if msg.max_range > 0.0 else 4.5
        return max(0.05, min(float(msg.max_range) if msg.max_range > 0.0 else 4.5, r))

    def _us_cb(self, side: str, msg: Range) -> None:
        self.us_ranges[side] = self._clean_range(msg)

    def _buoy_cb(self, msg: String) -> None:
        try:
            self.buoy_detections = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def _demand_cb(self, msg: Float32) -> None:
        self.feeding_demand_pct = float(msg.data)

    def _sensors_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            for k in ["weight", "ph", "do", "turbidity", "turbidity_pct", "temp", "valve"]:
                if k in data:
                    self.sensors[k] = float(data[k])
            if "turbidity_pct" not in data and "turbidity" in data:
                self.sensors["turbidity_pct"] = round(
                    min(100.0, max(0.0, self.sensors["turbidity"] / 1.5)), 1
                )
            self.sensors_received = True
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _limits_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            for k in ["min_do", "min_ph", "max_ph", "min_temp", "max_temp", "max_turb_pct"]:
                if k in data:
                    self.safe_limits[k] = float(data[k])
            self.get_logger().info(f"Updated Safe Limits from Website: {self.safe_limits}")
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

    def _dispatch_cb(self, msg: String) -> None:
        cmd = msg.data.strip().upper()
        if cmd == "AUTO":
            self.requested_seek_color = ""
            self.buoy_cooldowns.clear()
            self.set_feeder_valve(False)
            self._transition(FSMState.RANDOM_SEARCH, "Resumed Autonomous Vision Search")
        elif cmd == "STOP":
            self.set_feeder_valve(False)
            self.stop_robot()
            self._transition(FSMState.PAUSED, "Robot paused by operator")
        elif cmd.startswith("SEEK_"):
            color = cmd.replace("SEEK_", "")
            if color in ("GREEN", "YELLOW", "RED", "MAGENTA"):
                self.buoy_cooldowns.pop(color, None)
                self.requested_seek_color = color
                self.set_feeder_valve(False)
                self._transition(
                    FSMState.RANDOM_SEARCH, f"Operator requested seeking {color} buoy"
                )

    def set_feeder_valve(self, open_valve: bool) -> None:
        if self.valve_is_open != open_valve:
            self.valve_is_open = open_valve
            msg = Bool()
            msg.data = open_valve
            self.valve_pub.publish(msg)

    def stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _transition(self, new_state: FSMState, reason: str = "") -> None:
        if reason:
            self.latest_mission_text = reason
        if self.state != new_state:
            self.get_logger().info(f"[FSM] {self.state.value} -> {new_state.value} | {reason}")
            self.state = new_state
            self.state_enter_mono = time.monotonic()
            self.decision_completed_in_inspect = False

    def _publish_report(
        self,
        overall_status: str,
        status_badge: str,
        banner_title: str,
        banner_subtitle: str,
        action_title: str,
        action_steps: List[str],
        aerator_state: str,
        aerator_sub: str,
        feeder_state: str,
        feeder_sub: str,
        alert_state: str,
        alert_sub: str,
        dispensed_g: float = 0.0,
        planned_dose_g: float = 0.0,
        deficiencies: Optional[List[str]] = None,
        event_log: str = "",
    ) -> None:
        if event_log:
            self.report_seq += 1
        self.latest_report = {
            "seq": self.report_seq,
            "fsm_state": self.state.value,
            "overall_status": overall_status,
            "status_badge": status_badge,
            "banner_title": banner_title,
            "banner_subtitle": banner_subtitle,
            "action_title": action_title,
            "action_steps": action_steps,
            "aerator_state": aerator_state,
            "aerator_sub": aerator_sub,
            "feeder_state": feeder_state,
            "feeder_sub": feeder_sub,
            "alert_state": alert_state,
            "alert_sub": alert_sub,
            "target_buoy": self.target_buoy_color,
            "last_buoy": self.last_visited_buoy,
            "dispensed_g": round(dispensed_g, 1),
            "planned_dose_g": round(planned_dose_g, 1),
            "deficiencies": deficiencies or [],
            "event_log": event_log,
            "ultrasonic": {k: round(v, 2) for k, v in self.us_ranges.items()},
        }

    def _evaluate_sensors_against_limits(self) -> Tuple[List[str], bool]:
        """Checks DO, pH, Temp, Turbidity against editable limits. Returns (issues, critical_for_feeding)."""
        issues: List[str] = []
        critical_skip_feed = False

        do_val = self.sensors["do"]
        ph_val = self.sensors["ph"]
        temp_val = self.sensors["temp"]
        turb_pct = self.sensors.get("turbidity_pct", 22.0)

        if do_val < self.safe_limits["min_do"]:
            issues.append(
                f"Low Dissolved Oxygen: {do_val:.1f} mg/L (below safe {self.safe_limits['min_do']:.1f} mg/L)"
            )
            critical_skip_feed = True

        if ph_val < self.safe_limits["min_ph"] or ph_val > self.safe_limits["max_ph"]:
            issues.append(
                f"Out-of-range pH: {ph_val:.1f} (safe {self.safe_limits['min_ph']:.1f} to {self.safe_limits['max_ph']:.1f})"
            )
            critical_skip_feed = True

        if temp_val < self.safe_limits["min_temp"] or temp_val > self.safe_limits["max_temp"]:
            issues.append(
                f"Temperature out of range: {temp_val:.1f}°C (safe {self.safe_limits['min_temp']:.0f} to {self.safe_limits['max_temp']:.0f}°C)"
            )

        if turb_pct > self.safe_limits["max_turb_pct"]:
            issues.append(
                f"High Turbidity: {turb_pct:.0f}% (above safe {self.safe_limits['max_turb_pct']:.0f}%)"
            )

        return issues, critical_skip_feed

    def _pick_visible_buoy(self, now_mono: float) -> Optional[Tuple[str, dict]]:
        buoys = self.buoy_detections.get("buoys", {})
        if not buoys:
            return None

        # Priority if operator asked for a specific buoy color
        if self.requested_seek_color and self.requested_seek_color in buoys:
            return self.requested_seek_color, buoys[self.requested_seek_color]

        candidates = []
        for color, info in buoys.items():
            cooldown_until = self.buoy_cooldowns.get(color, 0.0)
            if now_mono >= cooldown_until:
                candidates.append((color, info))

        if not candidates:
            return None

        # Pick largest apparent buoy in frame
        return max(candidates, key=lambda item: item[1].get("area_ratio", 0.0))

    def _compute_ultrasonic_avoidance_twist(self, now_mono: float) -> Twist:
        """
        Pure 4-side ultrasonic obstacle avoidance + smooth random pond exploration.
        Prevents wall/pillar/buoy collisions on Front, Left, Right, and Rear.
        """
        twist = Twist()
        f = self.us_ranges["front"]
        l = self.us_ranges["left"]
        r = self.us_ranges["right"]
        b = self.us_ranges["rear"]

        # 1. Very close front obstacle -> slow reverse (if rear clear) & rotate away
        if f < 0.65:
            if now_mono > self.avoid_lock_until_mono:
                self.avoid_turn_dir = 1.0 if l >= r else -1.0
                self.avoid_lock_until_mono = now_mono + 1.4
            twist.linear.x = -0.22 if b > 0.60 else 0.0
            twist.angular.z = self.avoid_turn_dir * 0.95
            return twist

        # 2. Obstacle approaching in front -> steer cleanly around it
        if f < 1.35:
            if now_mono > self.avoid_lock_until_mono:
                self.avoid_turn_dir = 1.0 if l >= r else -1.0
                self.avoid_lock_until_mono = now_mono + 1.2
            twist.linear.x = 0.20 if f > 0.95 else 0.0
            twist.angular.z = self.avoid_turn_dir * 0.90
            return twist

        # 3. Free forward path -> smooth random wander with side ultrasonic guardrails
        if now_mono >= self.next_wander_change_mono:
            self.wander_angular = random.uniform(-0.48, 0.48)
            self.next_wander_change_mono = now_mono + random.uniform(2.0, 3.8)

        side_repulsion = 0.0
        if l < 0.85:
            side_repulsion -= (0.85 - l) * 1.15  # steer right away from left wall
        if r < 0.85:
            side_repulsion += (0.85 - r) * 1.15  # steer left away from right wall

        twist.linear.x = 0.60
        twist.angular.z = max(-0.95, min(0.95, self.wander_angular + side_repulsion))
        return twist

    def control_loop(self) -> None:
        now_mono = time.monotonic()

        # Always publish state, target, mission, and live structured report for web server
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

        target_msg = String()
        target_msg.data = self.target_buoy_color
        self.target_pub.publish(target_msg)

        mission_msg = String()
        mission_msg.data = self.latest_mission_text
        self.mission_pub.publish(mission_msg)

        # Keep ultrasonic snapshot fresh inside latest_report
        self.latest_report["fsm_state"] = self.state.value
        self.latest_report["ultrasonic"] = {
            k: round(v, 2) for k, v in self.us_ranges.items()
        }
        report_msg = String()
        report_msg.data = json.dumps(self.latest_report)
        self.report_pub.publish(report_msg)

        # =====================================================================
        # STATE: PAUSED
        # =====================================================================
        if self.state == FSMState.PAUSED:
            self.set_feeder_valve(False)
            self.stop_robot()
            return

        # =====================================================================
        # STATE 1: RANDOM_SEARCH (4x Ultrasonic Avoidance + Camera Buoy Scan)
        # =====================================================================
        if self.state == FSMState.RANDOM_SEARCH:
            self.set_feeder_valve(False)
            self.target_buoy_color = ""

            picked = self._pick_visible_buoy(now_mono)
            if picked is not None:
                color, info = picked
                self.target_buoy_color = color
                self.last_target_seen_mono = now_mono
                self._transition(
                    FSMState.APPROACH_BUOY,
                    f"Camera locked onto {color} Buoy! Guiding robot via visual servoing.",
                )
                return

            twist = self._compute_ultrasonic_avoidance_twist(now_mono)
            self.cmd_vel_pub.publish(twist)

            # Update ambient pond status while cruising cleanly between buoys
            issues, _ = self._evaluate_sensors_against_limits()
            f = self.us_ranges["front"]
            if f < 1.35:
                self.latest_mission_text = (
                    f"Random Search · Ultrasonic dodging obstacle ahead ({f:.2f}m)"
                )
            else:
                self.latest_mission_text = (
                    "Random Search · Scanning pond via camera for next colored buoy"
                )

            if not self.valve_is_open and not issues and (now_mono - self.state_enter_mono) > 3.5:
                self._publish_report(
                    overall_status="STABLE",
                    status_badge="Stable",
                    banner_title="All parameters are stable",
                    banner_subtitle="Aerators stay off to save energy. Robot cruising pond looking for next colored zone buoy.",
                    action_title="No action needed",
                    action_steps=[
                        f"No action needed — current pond water parameters within safe limits (Last: {self.last_visited_buoy}).",
                        f"Keep feeding on the normal schedule (Hopper: {self.sensors['weight']:.0f}g · Active scan via Camera + 4x Ultrasonics).",
                        "Aerators run only when dissolved oxygen falls below its safe limit.",
                    ],
                    aerator_state="OFF · AUTO",
                    aerator_sub="Start automatically when oxygen is low.",
                    feeder_state="Standby",
                    feeder_sub=f"Valve CLOSED · Searching buoys ({self.sensors['weight']:.0f}g reserve)",
                    alert_state="None",
                    alert_sub="Nothing to report.",
                )

        # =====================================================================
        # STATE 2: APPROACH_BUOY (Pure Camera Visual Servoing + Ultrasonic Safety)
        # =====================================================================
        elif self.state == FSMState.APPROACH_BUOY:
            self.set_feeder_valve(False)
            buoys = self.buoy_detections.get("buoys", {})
            target_info = buoys.get(self.target_buoy_color)

            if target_info is not None:
                self.last_target_seen_mono = now_mono
                err_x = float(target_info["error_x"])
                area_ratio = float(target_info["area_ratio"])
                f_dist = self.us_ranges["front"]
                l_dist = self.us_ranges["left"]
                r_dist = self.us_ranges["right"]

                # Arrival Condition: camera shows buoy close up OR front ultrasonic locks distance
                arrived_by_vision = area_ratio >= 0.044 and abs(err_x) < 0.38
                arrived_by_us = (f_dist <= 0.95) and (area_ratio >= 0.018) and (abs(err_x) < 0.42)

                if arrived_by_vision or arrived_by_us:
                    self.stop_robot()
                    self._transition(
                        FSMState.INSPECT_AND_DECIDE,
                        f"Arrived at {self.target_buoy_color} Buoy! Inspecting water sensors & fish vision...",
                    )
                    return

                # Side/front obstacle mid-path before buoy is close
                if f_dist < 0.80 and area_ratio < 0.016:
                    twist = Twist()
                    turn_dir = 1.0 if l_dist >= r_dist else -1.0
                    twist.linear.x = 0.08
                    twist.angular.z = turn_dir * 0.85
                    self.cmd_vel_pub.publish(twist)
                    self.latest_mission_text = (
                        f"Avoiding mid-pond obstacle ({f_dist:.2f}m) while tracking {self.target_buoy_color} Buoy"
                    )
                    return

                # Visual Servoing Proportional Controller
                omega_z = -1.65 * err_x
                # Side ultrasonic collision guard
                if l_dist < 0.55:
                    omega_z -= (0.55 - l_dist) * 1.2
                if r_dist < 0.55:
                    omega_z += (0.55 - r_dist) * 1.2
                omega_z = max(-1.10, min(1.10, omega_z))

                align_factor = max(0.25, 1.0 - min(0.75, abs(err_x)))
                base_speed = 0.58 if area_ratio < 0.020 else 0.36
                if f_dist < 1.40:
                    base_speed = min(base_speed, 0.32)

                twist = Twist()
                twist.linear.x = base_speed * align_factor
                twist.angular.z = omega_z
                self.cmd_vel_pub.publish(twist)
                self.latest_mission_text = (
                    f"Visual Servoing -> {self.target_buoy_color} Buoy (size {area_ratio*100:.1f}%, front {f_dist:.2f}m)"
                )
            else:
                # Buoy temporarily out of view
                if (now_mono - self.last_target_seen_mono) > 2.2:
                    self.stop_robot()
                    self._transition(
                        FSMState.RANDOM_SEARCH,
                        f"Lost sight of {self.target_buoy_color} Buoy; resuming Random Search.",
                    )
                else:
                    # Slow search spin slightly in last direction
                    twist = Twist()
                    twist.linear.x = 0.12
                    twist.angular.z = 0.35
                    self.cmd_vel_pub.publish(twist)

        # =====================================================================
        # STATE 3: INSPECT_AND_DECIDE (Sensor Evaluation & Missing Needs Report)
        # =====================================================================
        elif self.state == FSMState.INSPECT_AND_DECIDE:
            self.stop_robot()
            self.set_feeder_valve(False)
            elapsed = now_mono - self.state_enter_mono

            if not self.decision_completed_in_inspect:
                if elapsed < self.inspect_duration_sec:
                    self.latest_mission_text = (
                        f"Inspecting {self.target_buoy_color} Buoy water & fish demand ({elapsed:.1f}/{self.inspect_duration_sec:.1f}s)"
                    )
                    return

                self.decision_completed_in_inspect = True
                buoy_label = f"{self.target_buoy_color} Buoy"
                self.last_visited_buoy = buoy_label
                if self.requested_seek_color == self.target_buoy_color:
                    self.requested_seek_color = ""

                issues, critical_skip = self._evaluate_sensors_against_limits()
                do_val = self.sensors["do"]
                ph_val = self.sensors["ph"]
                temp_val = self.sensors["temp"]
                turb_pct = self.sensors.get("turbidity_pct", 22.0)
                weight_val = self.sensors["weight"]
                turb_ntu = self.sensors.get("turbidity", 33.0)

                effective_demand = max(
                    self.feeding_demand_pct,
                    min(80.0, (turb_ntu - 42.0) * 1.4) if turb_ntu > 42.0 else 0.0,
                )

                # CASE A: Critical Water Quality Deficit (Low DO or Bad pH) -> Valve stays CLOSED!
                if critical_skip:
                    self.aerators_on = (do_val < self.safe_limits["min_do"])
                    self.phone_alert_active = True
                    missing_summary = " | ".join(issues)

                    self._publish_report(
                        overall_status="CRITICAL",
                        status_badge="Action needed",
                        banner_title=f"Water anomaly detected at {buoy_label}",
                        banner_subtitle=f"{missing_summary}. Aerators switched ON · Feeder valve kept CLOSED.",
                        action_title="Action taken — Feeder Valve CLOSED",
                        action_steps=[
                            f"Sensor deficit at {buoy_label}: {missing_summary}.",
                            "Feeder valve kept CLOSED — prevents wasted feed from sinking and worsening water quality.",
                            f"Aerators switched {'ON (AUTO)' if self.aerators_on else 'STANDBY'} & operator phone alert active.",
                        ],
                        aerator_state="ON · AUTO" if self.aerators_on else "STANDBY",
                        aerator_sub=f"Running automatically — DO dropped to {do_val:.1f} mg/L." if self.aerators_on else "Monitoring pH/Temp alert.",
                        feeder_state="Paused (Safety)",
                        feeder_sub=f"Valve CLOSED at {buoy_label} due to water quality deficit.",
                        alert_state="Alert Sent",
                        alert_sub=f"{issues[0]}",
                        deficiencies=issues,
                        event_log=f"[{buoy_label}] SKIPPED FEEDING · {missing_summary} · Valve CLOSED · Aerators {'ON' if self.aerators_on else 'STANDBY'}",
                    )
                    self.post_inspect_wait_until = now_mono + 2.5
                    return

                # CASE B: Safe Water & Fish Demand High -> OPEN FEEDER VALVE!
                if effective_demand >= self.demand_threshold_pct:
                    if weight_val < self.low_feed_threshold_g:
                        self.phone_alert_active = True
                        self._publish_report(
                            overall_status="WARNING",
                            status_badge="Hopper low",
                            banner_title=f"Feed needed at {buoy_label} — Hopper Low",
                            banner_subtitle=f"Hopper reserve is {weight_val:.0f}g (< {self.low_feed_threshold_g:.0f}g). Please refill feed tank.",
                            action_title="Action needed — Refill Hopper",
                            action_steps=[
                                f"Fish demand at {buoy_label} is high ({effective_demand:.0f}%), water is safe (DO {do_val:.1f} mg/L, pH {ph_val:.1f}).",
                                f"Valve kept CLOSED because HX711 weight is low ({weight_val:.0f}g).",
                                "Refill hopper or click Refill in the dashboard to resume dispensing.",
                            ],
                            aerator_state="OFF · AUTO",
                            aerator_sub="Start automatically when oxygen is low.",
                            feeder_state="Low Feed",
                            feeder_sub=f"Hopper only {weight_val:.0f}g remaining.",
                            alert_state="Alert Sent",
                            alert_sub="Hopper refill required.",
                            deficiencies=["Low Feed Hopper Reserve"],
                            event_log=f"[{buoy_label}] HOPPER LOW ({weight_val:.0f}g) — Cannot dispense {effective_demand:.0f}% demand.",
                        )
                        self.post_inspect_wait_until = now_mono + 2.5
                        return

                    # Determine feed dose based on camera/sensor demand
                    self.planned_dose_g = 250.0 if effective_demand >= 60.0 else 180.0
                    self.feeding_start_weight_g = weight_val
                    self.aerators_on = False
                    self.phone_alert_active = False

                    self._publish_report(
                        overall_status="FEEDING",
                        status_badge="Valve OPEN",
                        banner_title=f"Feeder Valve OPENED at {buoy_label}",
                        banner_subtitle=f"Water stable (DO {do_val:.1f} mg/L, pH {ph_val:.1f}, {temp_val:.1f}°C). Vision measured {effective_demand:.0f}% fish demand -> Opening valve for {self.planned_dose_g:.0f}g feed.",
                        action_title=f"Valve OPEN · Dispensing {self.planned_dose_g:.0f}g",
                        action_steps=[
                            f"Feed shortage detected at {buoy_label}: Camera counted hungry fish school ({effective_demand:.0f}% demand).",
                            f"All water sensors SAFE (DO {do_val:.1f} mg/L, pH {ph_val:.1f}, Temp {temp_val:.1f}°C, Turbidity {turb_pct:.0f}%).",
                            f"Feeder Valve OPENED automatically to dispense {self.planned_dose_g:.0f}g (HX711 monitoring live).",
                        ],
                        aerator_state="OFF · AUTO",
                        aerator_sub="Start automatically when oxygen is low.",
                        feeder_state="Valve OPEN",
                        feeder_sub=f"Dispensing {self.planned_dose_g:.0f}g at {buoy_label}",
                        alert_state="None",
                        alert_sub="Water quality stable during feeding.",
                        dispensed_g=0.0,
                        planned_dose_g=self.planned_dose_g,
                        deficiencies=[f"Fish Feeding Demand ({effective_demand:.0f}%)"],
                        event_log=f"[{buoy_label}] VALVE OPENED ({self.planned_dose_g:.0f}g) · DO={do_val:.1f} pH={ph_val:.1f} Demand={effective_demand:.0f}%",
                    )

                    self._transition(
                        FSMState.EXECUTE_FEEDING,
                        f"Opening Valve at {buoy_label} -> Dispensing {self.planned_dose_g:.0f}g",
                    )
                    return

                # CASE C: Low Hunger (check if minor non-critical Temp/Turbidity warnings exist)
                self.aerators_on = False
                if issues:
                    self.phone_alert_active = True
                    issue_str = " | ".join(issues)
                    self._publish_report(
                        overall_status="WARNING",
                        status_badge="Monitor",
                        banner_title=f"Parameter drift observed at {buoy_label}",
                        banner_subtitle=f"{issue_str}. Feeder valve stayed CLOSED (low fish demand: {effective_demand:.0f}%).",
                        action_title="Monitoring needed",
                        action_steps=[
                            f"Sensor note at {buoy_label}: {issue_str}.",
                            f"Fish hunger low ({effective_demand:.0f}%), so Feeder Valve stayed CLOSED.",
                            "Aerators remain on standby while dissolved oxygen is adequate.",
                        ],
                        aerator_state="OFF · AUTO",
                        aerator_sub="Oxygen above minimum threshold.",
                        feeder_state="Standby",
                        feeder_sub=f"Valve CLOSED at {buoy_label}.",
                        alert_state="Warning",
                        alert_sub=issues[0],
                        deficiencies=issues,
                        event_log=f"[{buoy_label}] MONITOR WARNING · {issue_str} · Valve CLOSED",
                    )
                else:
                    self.phone_alert_active = False
                    self._publish_report(
                        overall_status="STABLE",
                        status_badge="Stable",
                        banner_title="All parameters are stable",
                        banner_subtitle=f"Inspected {buoy_label}: water safe and fish satiated ({effective_demand:.0f}% demand). Aerators stay off to save energy.",
                        action_title="No action needed",
                        action_steps=[
                            f"No action needed — water quality at {buoy_label} is optimal.",
                            f"Keep feeding on the normal schedule (Valve stayed CLOSED: DO {do_val:.1f} mg/L, pH {ph_val:.1f}, Temp {temp_val:.1f}°C, Turb {turb_pct:.0f}%).",
                            "Aerators run only when oxygen falls below its limit.",
                        ],
                        aerator_state="OFF · AUTO",
                        aerator_sub="Start automatically when oxygen is low.",
                        feeder_state="Running",
                        feeder_sub="On schedule — moving to explore next zone.",
                        alert_state="None",
                        alert_sub="Nothing to report.",
                        event_log=f"[{buoy_label}] ALL STABLE · DO={do_val:.1f} pH={ph_val:.1f} Temp={temp_val:.1f}C Turb={turb_pct:.0f}% · Valve CLOSED",
                    )
                self.post_inspect_wait_until = now_mono + 2.0
            else:
                if now_mono >= self.post_inspect_wait_until:
                    self._start_depart()

        # =====================================================================
        # STATE 4: EXECUTE_FEEDING (Closed-Loop HX711 Dosing)
        # =====================================================================
        elif self.state == FSMState.EXECUTE_FEEDING:
            self.stop_robot()
            self.set_feeder_valve(True)

            current_w = self.sensors["weight"]
            dispensed_g = max(0.0, self.feeding_start_weight_g - current_w)
            buoy_label = f"{self.target_buoy_color} Buoy"
            elapsed = now_mono - self.state_enter_mono

            # Update live dispensed progress on dashboard
            self.latest_report["dispensed_g"] = round(dispensed_g, 1)
            self.latest_report["feeder_sub"] = (
                f"Valve OPEN · {dispensed_g:.0f}g / {self.planned_dose_g:.0f}g at {buoy_label}"
            )
            self.latest_mission_text = (
                f"Valve OPEN at {buoy_label}: {dispensed_g:.0f}g / {self.planned_dose_g:.0f}g dispensed"
            )

            # Emergency safety check while feeding
            issues, critical_skip = self._evaluate_sensors_against_limits()
            if critical_skip:
                self.set_feeder_valve(False)
                self._publish_report(
                    overall_status="CRITICAL",
                    status_badge="Emergency Stop",
                    banner_title=f"Emergency valve shutoff at {buoy_label}",
                    banner_subtitle=f"Water dropped out of safe limits mid-feeding ({issues[0]}). Valve closed immediately.",
                    action_title="Valve CLOSED (Safety Shutoff)",
                    action_steps=[
                        f"Sensor alert mid-feeding at {buoy_label}: {issues[0]}.",
                        f"Closed valve early after {dispensed_g:.0f}g.",
                        "Resuming pond search and activating alerts.",
                    ],
                    aerator_state="ON · AUTO",
                    aerator_sub="Activated on sudden oxygen drop.",
                    feeder_state="Closed (Alert)",
                    feeder_sub="Emergency shutoff triggered.",
                    alert_state="Alert Sent",
                    alert_sub=issues[0],
                    dispensed_g=dispensed_g,
                    planned_dose_g=self.planned_dose_g,
                    deficiencies=issues,
                    event_log=f"[{buoy_label}] EMERGENCY VALVE SHUTOFF at {dispensed_g:.0f}g ({issues[0]})",
                )
                self._start_depart()
                return

            if dispensed_g >= self.planned_dose_g or elapsed >= 8.5 or current_w < 50.0:
                self.set_feeder_valve(False)
                self._publish_report(
                    overall_status="STABLE",
                    status_badge="Dose Complete",
                    banner_title=f"Feeding completed at {buoy_label} ({dispensed_g:.0f}g dispensed)",
                    banner_subtitle="Valve closed cleanly. Moving away to resume random pond exploration.",
                    action_title=f"Completed · Dispensed {dispensed_g:.0f}g",
                    action_steps=[
                        f"Opened valve at {buoy_label} based on high fish demand & safe water telemetry.",
                        f"Closed valve cleanly after dispensing {dispensed_g:.0f}g (Remaining hopper: {current_w:.0f}g).",
                        "Switching back to random patrol with 4-side ultrasonic obstacle avoidance.",
                    ],
                    aerator_state="OFF · AUTO",
                    aerator_sub="Start automatically when oxygen is low.",
                    feeder_state="Running",
                    feeder_sub=f"Last dose: {dispensed_g:.0f}g at {buoy_label} · Next scan starting",
                    alert_state="None",
                    alert_sub="Nothing to report.",
                    dispensed_g=dispensed_g,
                    planned_dose_g=self.planned_dose_g,
                    event_log=f"[{buoy_label}] VALVE CLOSED · Dispensed {dispensed_g:.0f}g (Hopper left: {current_w:.0f}g)",
                )
                self._start_depart()

        # =====================================================================
        # STATE 5: DEPART_BUOY (Cooldown + Ultrasonic-Safe Turn Away)
        # =====================================================================
        elif self.state == FSMState.DEPART_BUOY:
            self.set_feeder_valve(False)
            elapsed = now_mono - self.state_enter_mono
            twist = Twist()

            if elapsed < 0.75:
                # Reverse slightly away from buoy only if rear ultrasonic is clear
                if self.us_ranges["rear"] > 0.65:
                    twist.linear.x = -0.22
                twist.angular.z = self.depart_turn_dir * 0.45
            elif elapsed < 2.25:
                # Rotate ~120-140 deg away toward open water
                twist.linear.x = 0.05
                twist.angular.z = self.depart_turn_dir * 1.05
            else:
                self._transition(
                    FSMState.RANDOM_SEARCH,
                    "Resuming random exploration with camera & 4x ultrasonic avoidance.",
                )
                return

            self.cmd_vel_pub.publish(twist)

    def _start_depart(self) -> None:
        if self.target_buoy_color:
            self.buoy_cooldowns[self.target_buoy_color] = (
                time.monotonic() + self.buoy_cooldown_sec
            )
        self.depart_turn_dir = 1.0 if self.us_ranges["left"] >= self.us_ranges["right"] else -1.0
        old_color = self.target_buoy_color
        self.target_buoy_color = ""
        self._transition(
            FSMState.DEPART_BUOY,
            f"Departing {old_color} Buoy (cooldown {self.buoy_cooldown_sec:.0f}s) -> Turning to explore pond.",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_feeder_valve(False)
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
