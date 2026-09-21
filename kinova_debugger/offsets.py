"""Estimate per-joint zero offsets from wrist-IMU gravity readings.

At rest the wrist IMU measures gravity in its own frame. If joint ``k`` is
physically at ``reported_k + offset_k``, then for every pose ``i``::

    wrist_i  ~  s * Q * R_tool(reported_i + offsets)^T * base_i

where ``base_i`` is gravity in the base frame (from the base IMU), ``R_tool`` is
the nominal Gen3 forward kinematics, ``Q`` is how the IMU is mounted inside the
wrist, and ``s`` is the accelerometer's sign convention. One pose cannot separate
joints whose axes are parallel (2, 4 and 6 at Home), but several poses that
rotate the intervening joints can.

Not everything is observable:

* Joint 1 turns about the vertical, which never changes gravity. Never estimated.
* Joint 7 turns about the tool axis, directly before the IMU mounting. Without a
  known ``Q`` its offset is indistinguishable from how the IMU is mounted, so it
  is only estimated when ``Q`` comes from a baseline arm.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from .kinematics import angle_between, rotation_from_vector, rotation_to_vector, tool_rotation

# The nominal URDF kinematics differ from Kinova's per-unit calibrated model by
# up to ~1 deg, so no fit can be tighter than this regardless of IMU noise.
MODEL_FLOOR_DEG = 0.75
FAULT_THRESHOLD_DEG = 3.0       # smallest offset reported as a calibration fault
UNDETERMINED_SIGMA_DEG = 2.0    # 1-sigma above which an offset is "not pinned down"
FIT_RESIDUAL_LIMIT_DEG = 2.0    # RMS residual above which the offset model does not explain the data
HOME_JOINTS = (0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0)


@dataclass
class Pose:
    joints: list[float]
    wrist_accel: list[float]
    base_accel: list[float]
    wrist_noise_deg: float = 0.0

    @classmethod
    def from_dict(cls, data: dict) -> "Pose":
        return cls(list(data["joints"]), list(data["wrist_accel"]), list(data["base_accel"]),
                   float(data.get("wrist_noise_deg", 0.0)))


@dataclass
class ImuMounting:
    rotation: np.ndarray
    sign: float
    residuals_deg: list[float]

    def to_dict(self) -> dict:
        return {"rotation_vector": rotation_to_vector(self.rotation).tolist(), "sign": self.sign,
                "rms_residual_deg": _rms(self.residuals_deg)}


@dataclass
class OffsetEstimate:
    joints: list[int]                      # 1-based joint numbers that were estimated
    offsets_deg: dict[int, float]
    sigma_deg: dict[int, float]
    mounting: ImuMounting
    residuals_before_deg: list[float]
    residuals_after_deg: list[float]
    noise_deg: float
    used_baseline: bool
    notes: list[str] = field(default_factory=list)


def _unit(vector) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def _rms(values) -> float:
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def _predicted(pose: Pose, offsets_deg, rotation: np.ndarray, sign: float) -> np.ndarray:
    joints = [j + o for j, o in zip(pose.joints, offsets_deg)]
    return sign * rotation @ tool_rotation(joints).T @ _unit(pose.base_accel)


def fit_imu_mounting(poses: list[Pose], offsets_deg=(0.0,) * 7) -> ImuMounting:
    """Best IMU mounting rotation for fixed joint offsets (Kabsch), trying both sign conventions."""
    best = None
    for sign in (1.0, -1.0):
        model = np.array([sign * tool_rotation([j + o for j, o in zip(p.joints, offsets_deg)]).T
                          @ _unit(p.base_accel) for p in poses])
        measured = np.array([_unit(p.wrist_accel) for p in poses])
        u, _, vt = np.linalg.svd(measured.T @ model)
        correction = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
        rotation = u @ correction @ vt
        residuals = [angle_between(m, rotation @ v) for m, v in zip(measured, model)]
        if best is None or _rms(residuals) < _rms(best.residuals_deg):
            best = ImuMounting(rotation, sign, residuals)
    return best


def _residual_vector(poses, offsets_deg, rotation, sign) -> np.ndarray:
    return np.concatenate([_unit(p.wrist_accel) - _predicted(p, offsets_deg, rotation, sign) for p in poses])


def estimate_offsets(poses: list[Pose], baseline: ImuMounting | None = None) -> OffsetEstimate:
    """Fit joint zero offsets (and the IMU mounting, unless a baseline supplies it)."""
    if baseline is None:
        joints = [2, 3, 4, 5, 6]
        start = fit_imu_mounting(poses)
    else:
        joints = [2, 3, 4, 5, 6, 7]
        start = baseline
    rotation_params = 0 if baseline is not None else 3
    parameter_count = len(joints) + rotation_params

    def unpack(theta):
        offsets = [0.0] * 7
        for index, joint in enumerate(joints):
            offsets[joint - 1] = math.degrees(theta[index])
        rotation = start.rotation
        if rotation_params:
            rotation = start.rotation @ rotation_from_vector(theta[len(joints):])
        return offsets, rotation

    def residual(theta):
        offsets, rotation = unpack(theta)
        return _residual_vector(poses, offsets, rotation, start.sign)

    theta = np.zeros(parameter_count)
    current = residual(theta)
    cost = float(current @ current)
    damping = 1e-3
    jacobian = np.zeros((current.size, parameter_count))
    for _ in range(200):
        for k in range(parameter_count):
            step = np.zeros(parameter_count)
            step[k] = 1e-6
            jacobian[:, k] = (residual(theta + step) - residual(theta - step)) / 2e-6
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ current
        improved = False
        while damping < 1e8:
            delta = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-9), -gradient)
            trial = residual(theta + delta)
            trial_cost = float(trial @ trial)
            if trial_cost < cost:
                theta, current, cost = theta + delta, trial, trial_cost
                damping = max(damping * 0.3, 1e-9)
                improved = True
                break
            damping *= 10.0
        if not improved or np.linalg.norm(delta) < 1e-9:
            break

    offsets, rotation = unpack(theta)
    measured_noise = float(np.median([p.wrist_noise_deg for p in poses])) if poses else 0.0
    noise_deg = max(measured_noise, MODEL_FLOOR_DEG)

    # Each unit-vector residual carries 2 degrees of freedom.
    dof = 2 * len(poses) - parameter_count
    fit_variance = cost / dof if dof > 0 else float("inf")
    variance = max(fit_variance, math.radians(noise_deg) ** 2)
    normal = jacobian.T @ jacobian
    covariance = variance * np.linalg.inv(normal + 1e-12 * np.trace(normal) * np.eye(parameter_count))
    sigma = {joint: math.degrees(math.sqrt(max(covariance[i, i], 0.0))) for i, joint in enumerate(joints)}

    before = [angle_between(_unit(p.wrist_accel), _predicted(p, [0.0] * 7, start.rotation, start.sign))
              for p in poses]
    after = [angle_between(_unit(p.wrist_accel), _predicted(p, offsets, rotation, start.sign)) for p in poses]

    notes = ["Joint 1 is not estimated: it turns about the vertical, which never changes gravity."]
    if baseline is None:
        notes.append("Joint 7 is not estimated: without a baseline arm its offset is indistinguishable "
                     "from how the IMU is mounted in the wrist.")
    return OffsetEstimate(
        joints=joints,
        offsets_deg={j: offsets[j - 1] for j in joints},
        sigma_deg=sigma,
        mounting=ImuMounting(rotation, start.sign, after),
        residuals_before_deg=before,
        residuals_after_deg=after,
        noise_deg=noise_deg,
        used_baseline=baseline is not None,
        notes=notes,
    )


def gripper_error_at(joints_deg, offsets_deg: dict[int, float]) -> float:
    """Angle between where the controller thinks the gripper points and where it physically points."""
    corrected = [j + offsets_deg.get(i + 1, 0.0) for i, j in enumerate(joints_deg)]
    return angle_between(tool_rotation(joints_deg)[:, 2], tool_rotation(corrected)[:, 2])
