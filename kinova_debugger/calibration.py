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
import os
from datetime import datetime, timezone

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


# ---------------------------------------------------------------------------
# Multi-pose capture and joint offset analysis
# ---------------------------------------------------------------------------

POSE_FILE_FORMAT = "kinova-calibration-poses/1"
SETTLE_VELOCITY = 0.3      # deg/s every joint must stay under before and during a capture
SETTLE_SECONDS = 0.5
CAPTURE_SECONDS = 1.0
POLL_PERIOD = 0.02
SETTLE_TIMEOUT = 20.0
MAX_WRIST_NOISE_DEG = 1.0
MIN_POSES = 6

SUGGESTED_POSES = (
    "Home: hold the Home button until the arm stops.",
    "From Home, jog joint 7 to about 0 deg.",
    "Jog joint 7 to about 180 deg.",
    "Return to Home, then jog joint 5 to about +60 deg.",
    "Jog joint 5 to about -60 deg (300).",
    "Return to Home, then jog joint 3 to about 240 deg.",
    "Jog joint 3 to about 120 deg.",
    "Jog joint 3 to about 240 deg AND joint 5 to about +60 deg.",
    "Return to Home, then change joint 6 by ~30 deg, pointing the gripper UP.",
    "Return to Home, then change joint 2 by ~15 deg, lifting the arm AWAY from the table.",
)


class CaptureError(Exception):
    """A pose could not be captured cleanly; the message says why."""


def _circular_mean_deg(values) -> float:
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360.0


def _wrapped_difference(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def read_arm_identity(host: str, port: int = 10000) -> dict:
    """Base serial and tool transform, so pose files from different arms cannot be mixed."""
    from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
    from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient

    identity = {"host": host, "base_serial": None, "tool_transform_z": None}
    with kortex_session(host, port) as router:
        try:
            identity["base_serial"] = DeviceConfigClient(router).GetSerialNumber(0).serial_number
        except Exception:
            pass
        try:
            identity["tool_transform_z"] = ControlConfigClient(router).GetToolConfiguration().tool_transform.z
        except Exception:
            pass
    return identity


def capture_pose(host: str, port: int = 10000, settle_timeout: float = SETTLE_TIMEOUT) -> dict:
    """Wait for the arm to be still, then average wrist IMU, base IMU and joints over a short window."""
    import time

    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

    with kortex_session(host, port) as router:
        cyclic = BaseCyclicClient(router)
        deadline = time.time() + settle_timeout
        still_since = None
        samples = []
        while True:
            if time.time() > deadline:
                raise CaptureError(f"the arm did not stay still for {settle_timeout:.0f} s")
            feedback = cyclic.RefreshFeedback()
            now = time.time()
            if max(abs(a.velocity) for a in feedback.actuators) > SETTLE_VELOCITY:
                still_since = None
                samples = []
            else:
                if still_since is None:
                    still_since = now
                if now - still_since >= SETTLE_SECONDS:
                    samples.append(feedback)
                if now - still_since >= SETTLE_SECONDS + CAPTURE_SECONDS:
                    break
            time.sleep(POLL_PERIOD)

    wrist = [(f.interconnect.imu_acceleration_x, f.interconnect.imu_acceleration_y,
              f.interconnect.imu_acceleration_z) for f in samples]
    base = [(f.base.imu_acceleration_x, f.base.imu_acceleration_y, f.base.imu_acceleration_z) for f in samples]
    wrist_mean = [sum(v[i] for v in wrist) / len(wrist) for i in range(3)]
    base_mean = [sum(v[i] for v in base) / len(base) for i in range(3)]
    joints = [_circular_mean_deg([f.actuators[j].position for f in samples]) for j in range(len(samples[0].actuators))]
    noise = math.sqrt(sum(_angle_between(v, wrist_mean) ** 2 for v in wrist) / len(wrist))

    wrist_norm = math.sqrt(sum(c * c for c in wrist_mean))
    base_norm = math.sqrt(sum(c * c for c in base_mean))
    if abs(wrist_norm - 9.81) > GRAVITY_TOLERANCE:
        raise CaptureError(f"wrist IMU magnitude is {wrist_norm:.2f} m/s^2, expected ~9.81")
    if abs(base_norm - 9.81) > GRAVITY_TOLERANCE:
        raise CaptureError(f"base IMU magnitude is {base_norm:.2f} m/s^2, expected ~9.81")
    if noise > MAX_WRIST_NOISE_DEG:
        raise CaptureError(f"wrist IMU direction wandered {noise:.2f} deg during capture; something was vibrating")

    return {"joints": joints, "wrist_accel": wrist_mean, "base_accel": base_mean,
            "wrist_noise_deg": noise, "samples": len(samples),
            "captured_at": datetime.now(timezone.utc).isoformat(timespec="seconds")}


def load_pose_file(path: str) -> dict:
    with open(path) as handle:
        data = json.load(handle)
    if data.get("format") != POSE_FILE_FORMAT:
        raise ValueError(f"{path} is not a calibration pose file (format={data.get('format')!r})")
    return data


def save_pose_file(path: str, data: dict) -> None:
    temporary = f"{path}.tmp"
    with open(temporary, "w") as handle:
        json.dump(data, handle, indent=2)
    os.replace(temporary, path)


def joint_coverage(poses: list[dict], joint: int) -> float:
    """Largest wrapped spread, in degrees, between any two captured values of ``joint``."""
    values = [p["joints"][joint - 1] for p in poses]
    return max((_wrapped_difference(a, b) for a in values for b in values), default=0.0)


def run_capture_session(host: str, port: int, path: str, start_new: bool = False,
                        ask=input, say=print) -> int:
    """Interactively capture poses into ``path``, saving after every pose."""
    identity = read_arm_identity(host, port)
    if os.path.exists(path) and not start_new:
        data = load_pose_file(path)
        recorded = data.get("arm", {}).get("base_serial")
        if recorded and identity["base_serial"] and recorded != identity["base_serial"]:
            say(f"Refusing to append: {path} was captured on arm {recorded}, "
                f"but {host} is arm {identity['base_serial']}. Use a different --out, or --new.")
            return 1
        say(f"Appending to {path} ({len(data['poses'])} poses already captured).")
    else:
        data = {"format": POSE_FILE_FORMAT, "arm": identity,
                "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"), "poses": []}

    say(f"Arm {identity['base_serial'] or '(serial unavailable)'} at {host}.")
    say("")
    say("SAFETY: this tool never moves the arm. You jog it with the controller in Joint mode.")
    say("If this arm's calibration is suspect, its protection zones may not protect it:")
    say("jog slowly and keep the gripper well clear of the base and table. Exact angles do not")
    say("matter; the arm records wherever it actually is. Diverse poses are what count.")
    say("")

    while True:
        index = len(data["poses"])
        hint = SUGGESTED_POSES[index] if index < len(SUGGESTED_POSES) else "Any new, clearly different pose."
        say(f"Pose {index + 1}: {hint}")
        try:
            answer = ask("  Settle the arm there, then press Enter to capture (q to finish): ").strip().lower()
        except EOFError:
            answer = "q"
        if answer == "q":
            break
        say("  Waiting for the arm to be still...")
        try:
            pose = capture_pose(host, port)
        except CaptureError as error:
            say(f"  Not captured: {error}. Try again.")
            continue
        except Exception as error:
            say(f"  Not captured: could not talk to the arm ({error}). Try again.")
            continue

        for number, existing in enumerate(data["poses"], start=1):
            if max(_wrapped_difference(a, b) for a, b in zip(pose["joints"], existing["joints"])) < 5.0:
                say(f"  Note: nearly identical to pose {number}; it adds little information.")
                break
        data["poses"].append(pose)
        save_pose_file(path, data)
        joints = ", ".join(f"{j:.1f}" for j in pose["joints"])
        say(f"  Captured pose {len(data['poses'])}: joints [{joints}], IMU noise {pose['wrist_noise_deg']:.2f} deg.")
        say("")

    poses = data["poses"]
    say(f"Saved {len(poses)} poses to {path}.")
    if poses:
        spreads = ", ".join(f"joint {j} {joint_coverage(poses, j):.0f} deg" for j in (3, 5, 7))
        say(f"Coverage: {spreads}. Rotating each by 60 deg or more separates the pitch joints.")
    if len(poses) < MIN_POSES:
        say(f"At least {MIN_POSES} poses are needed for analysis; run capture again to add more.")
    else:
        say(f"Analyse with: python diagnose_calibration.py analyze {path}")
    return 0


def run_offset_analysis(path: str, baseline_path: str | None = None) -> list[CheckResult]:
    from .offsets import (FAULT_THRESHOLD_DEG, FIT_RESIDUAL_LIMIT_DEG, HOME_JOINTS, UNDETERMINED_SIGMA_DEG,
                          Pose, estimate_offsets, fit_imu_mounting, gripper_error_at)

    results: list[CheckResult] = []
    data = load_pose_file(path)
    poses = [Pose.from_dict(p) for p in data["poses"]]
    serial = data.get("arm", {}).get("base_serial") or "unknown"
    results.append(CheckResult("pose-file", Status.PASS, f"{len(poses)} poses from arm {serial}."))
    if len(poses) < MIN_POSES:
        results.append(CheckResult("pose-count", Status.FAIL,
                                   f"Only {len(poses)} poses; at least {MIN_POSES} are needed.",
                                   "Run capture again on the same --out file to add poses."))
        return results

    baseline = None
    if baseline_path:
        baseline_data = load_pose_file(baseline_path)
        baseline_poses = [Pose.from_dict(p) for p in baseline_data["poses"]]
        baseline_serial = baseline_data.get("arm", {}).get("base_serial") or "unknown"
        if baseline_serial != "unknown" and baseline_serial == serial:
            results.append(CheckResult("baseline", Status.WARN,
                                       "The baseline was captured on the same arm being analysed.",
                                       "A baseline should come from a different, known-good arm."))
        if len(baseline_poses) < MIN_POSES:
            results.append(CheckResult("baseline", Status.FAIL,
                                       f"Baseline has only {len(baseline_poses)} poses; {MIN_POSES} needed."))
            return results
        baseline = fit_imu_mounting(baseline_poses)
        rms = math.sqrt(sum(r * r for r in baseline.residuals_deg) / len(baseline.residuals_deg))
        if rms > FIT_RESIDUAL_LIMIT_DEG:
            results.append(CheckResult(
                "baseline", Status.WARN,
                f"Baseline arm {baseline_serial} does not fit a zero-offset model (RMS {rms:.2f} deg).",
                "It may not be correctly calibrated itself; results below are less trustworthy."))
        else:
            results.append(CheckResult("baseline", Status.PASS,
                                       f"Baseline arm {baseline_serial} fits a zero-offset model (RMS {rms:.2f} deg); "
                                       "using its IMU mounting."))

    estimate = estimate_offsets(poses, baseline)
    for note in estimate.notes:
        results.append(CheckResult("not-estimated", Status.SKIP, note))

    faulty = {}
    for joint in estimate.joints:
        offset = estimate.offsets_deg[joint]
        sigma = estimate.sigma_deg[joint]
        name = f"joint-{joint}-offset"
        details = {"offset_deg": round(offset, 3), "sigma_deg": round(sigma, 3)}
        if sigma > UNDETERMINED_SIGMA_DEG:
            results.append(CheckResult(
                name, Status.WARN,
                f"Joint {joint}: could not be pinned down ({offset:+.1f} +/- {sigma:.1f} deg).",
                "Capture more poses that rotate joints 3, 5 and 7 by 60-90 deg away from Home.",
                details=details))
        elif abs(offset) >= max(FAULT_THRESHOLD_DEG, 3.0 * sigma):
            faulty[joint] = offset
            results.append(CheckResult(
                name, Status.FAIL,
                f"Joint {joint} is physically {offset:+.1f} +/- {sigma:.1f} deg from the angle it reports.",
                "Joint zero calibration fault. Re-zero this actuator (Kinova calibration or support).",
                details=details))
        else:
            results.append(CheckResult(name, Status.PASS,
                                       f"Joint {joint} is within {abs(offset):.1f} +/- {sigma:.1f} deg of its reported angle.",
                                       details=details))

    before = math.sqrt(sum(r * r for r in estimate.residuals_before_deg) / len(poses))
    after = math.sqrt(sum(r * r for r in estimate.residuals_after_deg) / len(poses))
    if after > FIT_RESIDUAL_LIMIT_DEG:
        results.append(CheckResult(
            "offset-model-fit", Status.WARN,
            f"Constant joint offsets leave {after:.2f} deg RMS unexplained (from {before:.2f}).",
            "The error is not a simple zero offset: suspect slipping, a faulty IMU, or poses captured while moving.",
            details={"per_pose_residual_deg": [round(r, 2) for r in estimate.residuals_after_deg]}))
    else:
        results.append(CheckResult("offset-model-fit", Status.PASS,
                                   f"Offsets explain the IMU readings: RMS {before:.2f} -> {after:.2f} deg.",
                                   details={"per_pose_residual_deg": [round(r, 2) for r in estimate.residuals_after_deg]}))

    if faulty:
        error = gripper_error_at(HOME_JOINTS, faulty)
        joints = ", ".join(str(j) for j in faulty)
        results.append(CheckResult(
            "gripper-at-home", Status.FAIL,
            f"At Home, joint(s) {joints} put the gripper {error:.1f} deg from where the controller believes it points.",
            "Protection zones and collision checks use the controller's belief, so they cannot protect this arm."))
    return results
