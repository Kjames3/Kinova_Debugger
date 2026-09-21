import json
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from kinova_debugger import calibration
from kinova_debugger import kinematics as K
from kinova_debugger.models import Status
from kinova_debugger.offsets import (FAULT_THRESHOLD_DEG, UNDETERMINED_SIGMA_DEG, Pose, estimate_offsets,
                                     fit_imu_mounting)

FIXTURE = Path(__file__).parent / "fixtures" / "real1_fk_samples.json"
HOME = [0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0]


def load_fk_samples():
    return json.loads(FIXTURE.read_text())["samples"]


def synthetic_poses(physical_rotations, physical_joints, fault, seed=0, noise_deg=0.2):
    """Wrist/base IMU readings for an arm whose joints are physically ``physical_joints``."""
    rng = np.random.default_rng(seed)
    mounting = K.rotation_from_vector(rng.normal(size=3))
    gravity = K.rot_x(math.radians(0.8)) @ K.rot_y(math.radians(-0.5)) @ np.array([0.0, 0.0, -1.0])
    poses = []
    for rotation, joints in zip(physical_rotations, physical_joints):
        wrist = mounting @ rotation.T @ gravity
        wrist = K.rotation_from_vector(rng.normal(size=3) * math.radians(noise_deg)) @ wrist
        reported = [a - fault.get(i + 1, 0.0) for i, a in enumerate(joints)]
        poses.append(Pose(reported, list(wrist * 9.81), list(gravity * 9.81), noise_deg))
    return poses


def fixture_poses(indices, fault, seed=0):
    samples = [load_fk_samples()[i] for i in indices]
    rotations = [K.kortex_euler_matrix(*s["e"]) for s in samples]
    return synthetic_poses(rotations, [s["q"] for s in samples], fault, seed)


class KinematicsTests(unittest.TestCase):
    def test_matches_the_arm_controller_forward_kinematics(self) -> None:
        position_mm, orientation_deg = [], []
        for sample in load_fk_samples():
            position, rotation = K.tool_pose(sample["q"], 0.200)
            position_mm.append(np.linalg.norm(position - sample["p"]) * 1000.0)
            arm = K.kortex_euler_matrix(*sample["e"])
            orientation_deg.append(math.degrees(np.linalg.norm(K.rotation_to_vector(arm.T @ rotation))))
        # The nominal URDF model differs from Kinova's per-unit calibrated model by ~1 deg.
        # A convention error (sign, zero, Euler order) would be tens of degrees.
        self.assertLess(np.median(orientation_deg), 2.0)
        self.assertLess(max(orientation_deg), 3.0)
        self.assertLess(np.median(position_mm), 15.0)

    def test_rotation_vector_round_trip_including_half_turn(self) -> None:
        for vector in ([0.1, -0.2, 0.3], [math.pi, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 2.0, -0.5]):
            rotation = K.rotation_from_vector(vector)
            np.testing.assert_allclose(K.rotation_from_vector(K.rotation_to_vector(rotation)), rotation, atol=1e-9)


class OffsetEstimationTests(unittest.TestCase):
    POSES = list(range(0, 100, 10))

    def assert_offsets(self, estimate, expected, tolerance=1.5):
        for joint in estimate.joints:
            self.assertAlmostEqual(estimate.offsets_deg[joint], expected.get(joint, 0.0), delta=tolerance,
                                   msg=f"joint {joint}: {estimate.offsets_deg}")

    def test_healthy_arm_shows_no_offset_above_fault_threshold(self) -> None:
        estimate = estimate_offsets(fixture_poses(self.POSES, {}))
        for joint in estimate.joints:
            self.assertLess(abs(estimate.offsets_deg[joint]), FAULT_THRESHOLD_DEG)

    def test_pitch_fault_is_attributed_to_the_correct_joint(self) -> None:
        for joint in (2, 4, 6):
            with self.subTest(joint=joint):
                estimate = estimate_offsets(fixture_poses(self.POSES, {joint: 18.0}))
                self.assert_offsets(estimate, {joint: 18.0})

    def test_two_simultaneous_faults_are_separated(self) -> None:
        estimate = estimate_offsets(fixture_poses(self.POSES, {2: -20.0, 6: 15.0}))
        self.assert_offsets(estimate, {2: -20.0, 6: 15.0})

    def test_baseline_mounting_makes_joint_7_observable(self) -> None:
        baseline = fit_imu_mounting(fixture_poses(range(5, 105, 10), {}, seed=0))
        estimate = estimate_offsets(fixture_poses(self.POSES, {7: -12.0}, seed=0), baseline=baseline)
        self.assertIn(7, estimate.joints)
        self.assert_offsets(estimate, {7: -12.0})

    def test_joint_7_is_not_estimated_without_baseline(self) -> None:
        estimate = estimate_offsets(fixture_poses(self.POSES, {}))
        self.assertNotIn(7, estimate.joints)
        self.assertNotIn(1, estimate.joints)

    def test_poses_that_never_rotate_joints_3_and_5_are_flagged_undetermined(self) -> None:
        weak = [HOME, *[[*HOME[:6], a] for a in (0.0, 180.0, 270.0)],
                [*HOME[:5], 25.0, 90.0], [*HOME[:5], 40.0, 90.0],
                [0.0, 5.0, *HOME[2:]], [0.0, 10.0, *HOME[2:]]]
        poses = synthetic_poses([K.tool_rotation(q) for q in weak], weak, {6: 18.0})
        estimate = estimate_offsets(poses)
        self.assertGreater(max(estimate.sigma_deg[j] for j in (2, 4, 6)), UNDETERMINED_SIGMA_DEG)


class AnalysisReportTests(unittest.TestCase):
    def write_pose_file(self, directory, poses, serial):
        path = os.path.join(directory, f"{serial}.json")
        calibration.save_pose_file(path, {
            "format": calibration.POSE_FILE_FORMAT, "arm": {"base_serial": serial},
            "poses": [{"joints": p.joints, "wrist_accel": p.wrist_accel, "base_accel": p.base_accel,
                       "wrist_noise_deg": p.wrist_noise_deg} for p in poses]})
        return path

    def test_report_fails_the_faulty_joint_and_explains_gripper_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_pose_file(directory, fixture_poses(range(0, 100, 10), {6: 18.0}), "SUSPECT")
            results = {r.name: r for r in calibration.run_offset_analysis(path)}
        self.assertEqual(results["joint-6-offset"].status, Status.FAIL)
        for joint in (2, 3, 4, 5):
            self.assertNotEqual(results[f"joint-{joint}-offset"].status, Status.FAIL)
        self.assertEqual(results["gripper-at-home"].status, Status.FAIL)
        self.assertEqual(results["offset-model-fit"].status, Status.PASS)

    def test_report_with_baseline_estimates_joint_7(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            good = self.write_pose_file(directory, fixture_poses(range(5, 105, 10), {}), "GOOD")
            bad = self.write_pose_file(directory, fixture_poses(range(0, 100, 10), {7: 10.0}), "SUSPECT")
            results = {r.name: r for r in calibration.run_offset_analysis(bad, good)}
        self.assertEqual(results["baseline"].status, Status.PASS)
        self.assertEqual(results["joint-7-offset"].status, Status.FAIL)

    def test_too_few_poses_is_a_failure_not_a_guess(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_pose_file(directory, fixture_poses(range(3), {}), "SUSPECT")
            results = {r.name: r for r in calibration.run_offset_analysis(path)}
        self.assertEqual(results["pose-count"].status, Status.FAIL)
        self.assertNotIn("joint-6-offset", results)


class CaptureSessionTests(unittest.TestCase):
    def test_refuses_to_append_poses_from_a_different_arm(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "poses.json")
            calibration.save_pose_file(path, {"format": calibration.POSE_FILE_FORMAT,
                                              "arm": {"base_serial": "WO547041-2"}, "poses": []})
            messages = []
            identity = {"host": "192.168.1.10", "base_serial": "WO546670-0", "tool_transform_z": 0.2}
            with mock.patch.object(calibration, "read_arm_identity", return_value=identity), \
                    mock.patch.object(calibration, "capture_pose") as capture:
                code = calibration.run_capture_session("192.168.1.10", 10000, path,
                                                       ask=lambda _: "q", say=messages.append)
            self.assertEqual(code, 1)
            capture.assert_not_called()
            self.assertIn("Refusing to append", messages[0])

    def test_saves_after_each_pose(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "poses.json")
            pose = {"joints": HOME, "wrist_accel": [0, 9.8, 0], "base_accel": [0, 0, -9.8],
                    "wrist_noise_deg": 0.1, "samples": 50, "captured_at": "t"}
            answers = iter(["", "q"])
            identity = {"host": "h", "base_serial": "X", "tool_transform_z": 0.2}
            with mock.patch.object(calibration, "read_arm_identity", return_value=identity), \
                    mock.patch.object(calibration, "capture_pose", return_value=pose):
                calibration.run_capture_session("h", 10000, path, ask=lambda _: next(answers), say=lambda _: None)
            self.assertEqual(len(calibration.load_pose_file(path)["poses"]), 1)


if __name__ == "__main__":
    unittest.main()
