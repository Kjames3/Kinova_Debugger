"""Compare the arm's kinematic model against physically measured orientation.

The wrist interconnect module carries an IMU. With the arm stationary the IMU
reads gravity, which is a direct physical measurement of how the wrist is
actually oriented — independent of the joint encoders. If the encoders are
correctly zeroed, two arms at the same joint angles must read the same gravity
vector. A persistent difference means the joint calibration is wrong: the
controller believes the arm is somewhere it physically is not.
"""

import json
import math

from .models import CheckResult, Status
from .session import kortex_session

STATIC_VELOCITY = 0.5    # deg/s; above this the IMU also sees real acceleration
GRAVITY_TOLERANCE = 0.6  # m/s^2 that |accel| may deviate from 9.81 while static
SAMPLES = 8


def _tilt(ax, ay, az):
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    return math.degrees(math.acos(max(-1.0, min(1.0, az / norm)))), norm


def _angle_between(u, v):
    nu = math.sqrt(sum(c * c for c in u))
    nv = math.sqrt(sum(c * c for c in v))
    dot = sum(a * b for a, b in zip(u, v)) / (nu * nv)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def sample_orientation(host: str, port: int = 10000) -> dict:
    """Average several static samples of joint angles and wrist gravity."""
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

    with kortex_session(host, port) as router:
        cyclic = BaseCyclicClient(router)
        accels, joints, moving = [], None, False
        for _ in range(SAMPLES):
            feedback = cyclic.RefreshFeedback()
            if max(abs(a.velocity) for a in feedback.actuators) > STATIC_VELOCITY:
                moving = True
            imu = feedback.interconnect
            accels.append((imu.imu_acceleration_x, imu.imu_acceleration_y, imu.imu_acceleration_z))
            joints = [a.position for a in feedback.actuators]

    mean = [sum(c[i] for c in accels) / len(accels) for i in range(3)]
    tilt, norm = _tilt(*mean)
    return {"host": host, "joints": joints, "accel": mean, "tilt_deg": tilt,
            "accel_norm": norm, "moving": moving}


def run_calibration_checks(host: str, port: int = 10000, reference_path: str | None = None,
                           save_path: str | None = None) -> list[CheckResult]:
    results: list[CheckResult] = []
    try:
        current = sample_orientation(host, port)
    except Exception as error:
        return [CheckResult("wrist-imu", Status.FAIL, f"Could not sample the wrist IMU: {error}",
                            "Check the connection and that the arm is powered.")]

    if current["moving"]:
        results.append(CheckResult("arm-static", Status.FAIL,
                                   "The arm moved while sampling; the IMU reading is not just gravity.",
                                   "Let the arm come to rest and run again."))
        return results
    results.append(CheckResult("arm-static", Status.PASS, "Arm is stationary."))

    if abs(current["accel_norm"] - 9.81) > GRAVITY_TOLERANCE:
        results.append(CheckResult(
            "wrist-imu", Status.WARN,
            f"Wrist IMU magnitude is {current['accel_norm']:.2f} m/s^2, expected ~9.81.",
            "The IMU may be faulty; treat the tilt below as unreliable."))
    else:
        results.append(CheckResult("wrist-imu", Status.PASS,
                                   f"Wrist IMU reads {current['accel_norm']:.2f} m/s^2 (gravity)."))

    joints = ", ".join(f"{j:.2f}" for j in current["joints"])
    results.append(CheckResult("wrist-tilt", Status.PASS,
                               f"Wrist tilt is {current['tilt_deg']:.1f} deg at joints [{joints}].",
                               details={k: current[k] for k in ("joints", "accel", "tilt_deg")}))

    if save_path:
        with open(save_path, "w") as handle:
            json.dump(current, handle, indent=2)
        results.append(CheckResult("reference-saved", Status.PASS, f"Wrote reference to {save_path}."))

    if reference_path:
        with open(reference_path) as handle:
            reference = json.load(handle)
        drift = [abs((a - b + 180.0) % 360.0 - 180.0) for a, b in zip(current["joints"], reference["joints"])]
        worst_joint = max(range(len(drift)), key=lambda i: drift[i])

        if drift[worst_joint] > 1.0:
            results.append(CheckResult(
                "reference-pose", Status.WARN,
                f"Reference was taken at a different pose (joint {worst_joint + 1} differs by "
                f"{drift[worst_joint]:.2f} deg); the comparison below is only valid at a matched pose.",
                "Drive both arms to the same stored action before comparing."))
        else:
            results.append(CheckResult("reference-pose", Status.PASS,
                                       "Reference was taken at the same joint angles."))

        physical = _angle_between(current["accel"], reference["accel"])
        name = f"calibration-vs-{reference.get('host', 'reference')}"
        if physical > 5.0:
            results.append(CheckResult(
                name, Status.FAIL,
                f"At the same joint angles the wrist is physically {physical:.1f} deg away from the "
                f"reference arm ({current['tilt_deg']:.1f} deg tilt vs {reference['tilt_deg']:.1f} deg).",
                "Joint zero calibration is off. The controller's model does not match the real arm, "
                "so collision and protection-zone checks cannot protect it. Re-run the Kinova "
                "actuator calibration or contact Kinova support.",
                details={"physical_difference_deg": physical}))
        else:
            results.append(CheckResult(name, Status.PASS,
                                       f"Wrist orientation matches the reference within {physical:.1f} deg."))
    return results
