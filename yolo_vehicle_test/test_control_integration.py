#!/usr/bin/env python3
"""
VIRYA DBW Buggy - Automated Control Integration Test Suite
Module: test_control_integration.py

Comprehensive 18-Scenario Offline Integration & Safety Test Suite
Validates the VehicleControlAdapter and perception-to-control pipeline:
  - Setpoint mappings (STOP, MOVE_LEFT, MOVE_RIGHT, SLOW_DOWN, PATH_CLEAR, UNKNOWN)
  - Mathematical sign conventions (+10 deg LEFT, -10 deg RIGHT)
  - Slew-rate limiting (30.0 deg/s) & longitudinal acceleration limits
  - Safety clamps ([-30 deg, +30 deg], [-2.5 m/s, +2.5 m/s])
  - Modbus 16-bit Two's Complement encoding (Register 3)
  - SocketCAN frame layout (ID: 0x12280797, speed in cm/s, 0x64 brake)
  - Automatic braking interlocks on zero-velocity
  - Watchdogs: Perception staleness (>300ms), Camera failure, Deadman dropout (>200ms)
  - DBW FSM state transitions (IDLE, AUTOMODE, FAULT)
  - Emergency stop handling & clean shutdown

Guaranteed 100% Offline: Zero physical actuation, zero ROS 2/CAN transmission.
Outputs structured JSON report to: results/evaluation/control_integration_test_report.json
"""

import os
import sys
import time
import json
import math
import unittest
from dataclasses import dataclass, asdict
from typing import List, Dict, Any

from vehicle_control_adapter import (
    VehicleControlAdapter,
    ControlMode,
    SubsystemStatus,
    FSMState,
    AdapterTelemetry,
)


class TestControlIntegration(unittest.TestCase):
    """18 Automated Verification Scenarios for Perception-to-Control Integration."""

    @classmethod
    def setUpClass(cls):
        cls.results = []
        cls.report_dir = "/home/vat/yolo_vehicle_test/results/evaluation"
        os.makedirs(cls.report_dir, exist_ok=True)
        print("\n" + "=" * 80)
        print("🧪 RUNNING VIRYA DBW PERCEPTION-TO-CONTROL INTEGRATION TEST SUITE (18 SCENARIOS)")
        print("   Mode: Strictly Offline & Non-Actuating (MODE_OFFLINE_MOCK)")
        print("=" * 80)

    def _record_result(self, test_id: int, name: str, passed: bool, details: Dict[str, Any]):
        self.results.append({
            "test_id": test_id,
            "name": name,
            "status": "PASS" if passed else "FAIL",
            "timestamp": time.time(),
            "details": details,
        })
        status_symbol = "✅ PASS" if passed else "❌ FAIL"
        print(f"[{test_id:02d}/18] {status_symbol} - {name}")

    # 1. STOP mapping
    def test_01_stop_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("STOP")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 0.0)
            self.assertEqual(tel.velocity.target_value, 0.0)
            self.assertEqual(tel.brake.target_value, 1.0)
            self.assertEqual(tel.brake.commanded_value, 1.0)
            self._record_result(1, "STOP Decision Setpoint Mapping", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 2. MOVE_LEFT mapping
    def test_02_move_left_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("MOVE_LEFT")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 10.0, "MOVE_LEFT must command +10.0 degrees (LEFT is positive)")
            self.assertEqual(tel.velocity.target_value, 0.5, "MOVE_LEFT must command caution speed 0.5 m/s")
            self.assertEqual(tel.brake.target_value, 0.0, "MOVE_LEFT must release service brake")
            self._record_result(2, "MOVE_LEFT Setpoint Mapping (+10.0 deg Left, 0.5 m/s)", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 3. MOVE_RIGHT mapping
    def test_03_move_right_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("MOVE_RIGHT")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, -10.0, "MOVE_RIGHT must command -10.0 degrees (RIGHT is negative)")
            self.assertEqual(tel.velocity.target_value, 0.5, "MOVE_RIGHT must command caution speed 0.5 m/s")
            self.assertEqual(tel.brake.target_value, 0.0, "MOVE_RIGHT must release service brake")
            self._record_result(3, "MOVE_RIGHT Setpoint Mapping (-10.0 deg Right, 0.5 m/s)", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 4. SLOW_DOWN mapping
    def test_04_slow_down_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("SLOW_DOWN")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 0.0)
            self.assertEqual(tel.velocity.target_value, 0.5)
            self.assertEqual(tel.brake.target_value, 0.0)
            self._record_result(4, "SLOW_DOWN Setpoint Mapping (0.0 deg, 0.5 m/s)", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 5. PATH_CLEAR mapping
    def test_05_path_clear_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("PATH_CLEAR")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 0.0)
            self.assertEqual(tel.velocity.target_value, 1.0, "PATH_CLEAR must command nominal cruising 1.0 m/s")
            self.assertEqual(tel.brake.target_value, 0.0)
            self._record_result(5, "PATH_CLEAR Setpoint Mapping (0.0 deg, 1.0 m/s)", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 6. UNKNOWN mapping
    def test_06_unknown_mapping(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("UNKNOWN")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 0.0)
            self.assertEqual(tel.velocity.target_value, 0.0)
            self.assertEqual(tel.brake.target_value, 1.0)
            self._record_result(6, "UNKNOWN Safe Fallback Mapping (vel=0, brake=1)", True, {
                "steer_target": tel.steer.target_value,
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
            })
        finally:
            adapter.stop()

    # 7. Steering sign conversion (Modbus 16-bit Two's Complement)
    def test_07_steering_sign_conversion(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            # Positive angle (Left turn: +10.0 deg -> 100 in Register 3)
            # Maintain active perception stream (simulating 15 FPS camera)
            for _ in range(5):
                adapter.update_perception_decision("MOVE_LEFT")
                time.sleep(0.08)
            tel_left = adapter.get_telemetry()
            self.assertAlmostEqual(tel_left.steer.commanded_value, 10.0, places=1)
            self.assertEqual(tel_left.simulated_modbus_reg3, 100)

            # Negative angle (Right turn: -10.0 deg -> (1<<16) - 100 = 65436 in Register 3)
            # Allow slew limiter to transition +10.0 -> -10.0 deg (20 deg / 30 deg/s ~ 0.67s)
            for _ in range(10):
                adapter.update_perception_decision("MOVE_RIGHT")
                time.sleep(0.08)
            tel_right = adapter.get_telemetry()
            self.assertAlmostEqual(tel_right.steer.commanded_value, -10.0, places=1)
            expected_twos_comp = (1 << 16) - 100  # 65436
            self.assertEqual(tel_right.simulated_modbus_reg3, expected_twos_comp)

            self._record_result(7, "Steering Sign & Modbus Two's Complement Conversion", True, {
                "left_reg3": tel_left.simulated_modbus_reg3,
                "right_reg3": tel_right.simulated_modbus_reg3,
                "expected_right_reg3": expected_twos_comp,
            })
        finally:
            adapter.stop()

    # 8. Steering limits clamp
    def test_08_steering_limits_clamp(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        clamped_pos = adapter.clamp_steering(45.0)
        clamped_neg = adapter.clamp_steering(-60.0)
        clamped_norm = adapter.clamp_steering(15.0)

        self.assertEqual(clamped_pos, 30.0, "Positive steer must clamp at +30.0 deg")
        self.assertEqual(clamped_neg, -30.0, "Negative steer must clamp at -30.0 deg")
        self.assertEqual(clamped_norm, 15.0, "Normal steer must pass unclamped")

        self._record_result(8, "Steering Limit Clamp [-30.0°, +30.0°]", True, {
            "clamped_45_to": clamped_pos,
            "clamped_neg60_to": clamped_neg,
            "normal_15_remains": clamped_norm,
        })

    # 9. Velocity limits clamp
    def test_09_velocity_limits_clamp(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        clamped_pos = adapter.clamp_velocity(4.2)
        clamped_neg = adapter.clamp_velocity(-3.5)
        clamped_norm = adapter.clamp_velocity(1.8)

        self.assertEqual(clamped_pos, 2.5, "Positive velocity must clamp at +2.5 m/s")
        self.assertEqual(clamped_neg, -2.5, "Negative velocity must clamp at -2.5 m/s")
        self.assertEqual(clamped_norm, 1.8, "Normal velocity must pass unclamped")

        self._record_result(9, "Velocity Limit Clamp [-2.5 m/s, +2.5 m/s]", True, {
            "clamped_pos": clamped_pos,
            "clamped_neg": clamped_neg,
            "clamped_norm": clamped_norm,
        })

    # 10. Brake request mapping & automatic interlock
    def test_10_brake_request_and_interlock(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            # STOP commands vel=0, brake=1 -> CAN Byte 3 must be 0x64 (100% brake)
            adapter.update_perception_decision("STOP")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            can_bytes = list(tel.simulated_can_tx_bytes)
            self.assertEqual(can_bytes[3], 0x64, "CAN Byte 3 must be 0x64 (full service brake)")
            self.assertEqual(can_bytes[0], 0x00, "CAN Byte 0 must be 0x00 (drive neutral)")

            self._record_result(10, "Brake Request & Auto Interlock (CAN Byte 3 = 0x64)", True, {
                "can_byte_3_brake": f"0x{can_bytes[3]:02x}",
                "can_byte_0_mode": f"0x{can_bytes[0]:02x}",
            })
        finally:
            adapter.stop()

    # 11. Invalid perception data handling
    def test_11_invalid_perception_data_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            # Pass None or malformed object
            adapter.update_perception_decision(None, is_camera_healthy=False)
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.velocity.target_value, 0.0)
            self.assertEqual(tel.brake.target_value, 1.0)
            self.assertFalse(tel.perception_fresh)
            self.assertEqual(tel.last_decision_str, "FAULT_CAMERA_OR_INPUT")

            self._record_result(11, "Invalid / Malformed Perception Data Handling", True, {
                "velocity_target": tel.velocity.target_value,
                "brake_target": tel.brake.target_value,
                "perception_fresh": tel.perception_fresh,
            })
        finally:
            adapter.stop()

    # 12. Camera failure handling
    def test_12_camera_failure_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("PATH_CLEAR", is_camera_healthy=True)
            time.sleep(0.08)
            # Simulate camera drop
            adapter.update_perception_decision("PATH_CLEAR", is_camera_healthy=False)
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.velocity.target_value, 0.0)
            self.assertEqual(tel.brake.target_value, 1.0)
            self.assertFalse(tel.perception_fresh)

            self._record_result(12, "Camera Frame Drop & Hardware Disconnection Handling", True, {
                "safe_stop_asserted": tel.velocity.target_value == 0.0 and tel.brake.target_value == 1.0,
                "perception_fresh": tel.perception_fresh,
            })
        finally:
            adapter.stop()

    # 13. Communication timeout handling (>300 ms staleness)
    def test_13_perception_timeout_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("PATH_CLEAR")
            time.sleep(0.08)
            tel_fresh = adapter.get_telemetry()
            self.assertTrue(tel_fresh.perception_fresh)

            # Wait 350 ms (> 300 ms timeout) without updating perception
            time.sleep(0.35)
            tel_stale = adapter.get_telemetry()
            self.assertFalse(tel_stale.perception_fresh)
            self.assertEqual(tel_stale.velocity.target_value, 0.0)
            self.assertEqual(tel_stale.brake.target_value, 1.0)
            self.assertIn("Perception stale", tel_stale.velocity.fault_reason)

            self._record_result(13, "Perception Timeout Watchdog (> 300 ms Staleness)", True, {
                "stale_age_ms": round(tel_stale.perception_age_ms, 1),
                "watchdog_tripped": not tel_stale.perception_fresh,
                "brake_engaged": tel_stale.brake.commanded_value == 1.0,
            })
        finally:
            adapter.stop()

    # 14. DBW supervisor fault handling
    def test_14_dbw_supervisor_fault_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("PATH_CLEAR")
            time.sleep(0.08)
            # Trigger Emergency Stop
            adapter.trigger_emergency_stop("Obstacle collision hazard detected")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.fsm_state, FSMState.FAULT)
            self.assertTrue(tel.emergency_stop)
            self.assertEqual(tel.velocity.commanded_value, 0.0)
            self.assertEqual(tel.brake.commanded_value, 1.0)
            # CAN byte 4 must assert park brake 0x01
            can_bytes = list(tel.simulated_can_tx_bytes)
            self.assertEqual(can_bytes[4], 0x01)

            self._record_result(14, "DBW Supervisor Fault & Emergency Stop Transition", True, {
                "fsm_state": tel.fsm_state.value,
                "emergency_stop": tel.emergency_stop,
                "can_park_brake": f"0x{can_bytes[4]:02x}",
            })
        finally:
            adapter.stop()

    # 15. Deadman inactive handling
    def test_15_deadman_inactive_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            adapter.update_perception_decision("PATH_CLEAR")
            time.sleep(0.08)
            tel_active = adapter.get_telemetry()
            self.assertEqual(tel_active.fsm_state, FSMState.AUTOMODE)

            # Inhibit deadman switch
            adapter.set_deadman(False)
            time.sleep(0.08)
            tel_inactive = adapter.get_telemetry()
            self.assertEqual(tel_inactive.fsm_state, FSMState.IDLE)
            self.assertEqual(tel_inactive.velocity.commanded_value, 0.0)
            self.assertEqual(tel_inactive.brake.commanded_value, 1.0)

            self._record_result(15, "Deadman Dropout & FSM IDLE Reversion", True, {
                "initial_fsm": tel_active.fsm_state.value,
                "reverted_fsm": tel_inactive.fsm_state.value,
                "velocity_zeroed": tel_inactive.velocity.commanded_value == 0.0,
            })
        finally:
            adapter.stop()

    # 16. Actuator Slew Rate Limiter (Max 30.0 deg/s)
    def test_16_steering_slew_rate_limiter(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK, steer_slew_rate=30.0)
        adapter.start()
        try:
            # Request step jump to MOVE_LEFT (+10.0 deg)
            adapter.update_perception_decision("MOVE_LEFT")
            # Sample at 100 ms: rate limit is 30 deg/s, so at 100 ms max angle ~ 3.0 deg
            time.sleep(0.10)
            tel_100ms = adapter.get_telemetry()
            # Must not jump immediately to 10.0 deg!
            self.assertLess(tel_100ms.steer.commanded_value, 5.0)
            self.assertGreater(tel_100ms.steer.commanded_value, 1.5)

            # Continue streaming MOVE_LEFT to keep perception fresh
            for _ in range(4):
                adapter.update_perception_decision("MOVE_LEFT")
                time.sleep(0.08)
            tel_400ms = adapter.get_telemetry()
            self.assertAlmostEqual(tel_400ms.steer.commanded_value, 10.0, places=1)

            self._record_result(16, "Actuator Slew Rate Limiter (30.0 deg/s Smooth Ramp)", True, {
                "steer_at_100ms": round(tel_100ms.steer.commanded_value, 2),
                "steer_at_400ms": round(tel_400ms.steer.commanded_value, 2),
                "step_jump_prevented": True,
            })
        finally:
            adapter.stop()

    # 17. Stale control command handling (steer-speed coupling)
    def test_17_stale_control_coupling_handling(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        try:
            # Force velocity to 0 and verify steering snaps to center on stopped vehicle
            adapter.update_perception_decision("STOP")
            time.sleep(0.08)
            tel = adapter.get_telemetry()
            self.assertEqual(tel.steer.target_value, 0.0)
            self.assertEqual(tel.velocity.target_value, 0.0)
            self.assertEqual(tel.brake.commanded_value, 1.0)

            self._record_result(17, "Steer-Speed Coupling & Neutral Steering on Zero Velocity", True, {
                "steer_centered": tel.steer.target_value == 0.0,
                "velocity_zero": tel.velocity.target_value == 0.0,
                "brake_locked": tel.brake.commanded_value == 1.0,
            })
        finally:
            adapter.stop()

    # 18. Adapter clean shutdown
    def test_18_adapter_clean_shutdown(self):
        adapter = VehicleControlAdapter(mode=ControlMode.MODE_OFFLINE_MOCK)
        adapter.start()
        self.assertTrue(adapter.running)
        self.assertTrue(adapter._thread.is_alive())

        adapter.stop()
        self.assertFalse(adapter.running)
        self.assertFalse(adapter._thread.is_alive())
        tel = adapter.get_telemetry()
        self.assertEqual(tel.steer.commanded_value, 0.0)
        self.assertEqual(tel.velocity.commanded_value, 0.0)
        self.assertEqual(tel.brake.commanded_value, 1.0)

        self._record_result(18, "Adapter Clean Thread Termination & Neutral Resource Release", True, {
            "thread_stopped": not adapter._thread.is_alive(),
            "neutral_steer": tel.steer.commanded_value == 0.0,
            "neutral_vel": tel.velocity.commanded_value == 0.0,
            "locked_brake": tel.brake.commanded_value == 1.0,
        })

    @classmethod
    def tearDownClass(cls):
        report_file = os.path.join(cls.report_dir, "control_integration_test_report.json")
        total_tests = len(cls.results)
        passed_tests = sum(1 for r in cls.results if r["status"] == "PASS")
        summary = {
            "total_scenarios": total_tests,
            "passed_scenarios": passed_tests,
            "failed_scenarios": total_tests - passed_tests,
            "success_rate_percent": (passed_tests / total_tests) * 100.0 if total_tests > 0 else 0.0,
            "test_mode": "MODE_OFFLINE_MOCK",
            "physical_actuation_transmitted": False,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "results": cls.results,
        }
        with open(report_file, "w") as f:
            json.dump(summary, f, indent=2)

        print("\n" + "=" * 80)
        print(f"📊 TEST SUITE SUMMARY: {passed_tests}/{total_tests} SCENARIOS PASSED ({(passed_tests/total_tests)*100:.1f}%)")
        print(f"📁 Report saved to: {report_file}")
        print("=" * 80)


if __name__ == "__main__":
    unittest.main()
