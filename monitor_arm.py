"""Watch the arm while you drive it, and report why a move faulted or stopped short.

Read-only: it never commands motion. Start it, press the controller button you
are investigating, and it records every fault bit, the settled joint angles, and
how those compare with the stored action targets.

    .venv/bin/python monitor_arm.py --host 192.168.1.10 --csv run.csv
"""

import argparse
import csv
import math
import time

from kinova_debugger.session import kortex_session
from kinova_debugger.zones import penetration

SETTLE_SPEED = 0.5     # deg/s below which every joint counts as stopped
SETTLE_SAMPLES = 8     # consecutive stopped samples before a pose is "settled"
MATCH_TOLERANCE = 1.0  # deg; a settled pose within this of a target counts as reached


def _angle_delta(a: float, b: float) -> float:
    """Shortest signed difference between two angles in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


def _decode(bank: int, enum=None) -> str:
    """Name every set bit, using a Kortex enum when one is supplied."""
    if not bank:
        return "none"
    names = []
    for i in range(32):
        value = 1 << i
        if not bank & value:
            continue
        label = f"bit{i}"
        if enum is not None:
            try:
                label = enum.Name(value)
            except ValueError:
                pass
        names.append(label)
    return ",".join(names)


def main() -> int:
    parser = argparse.ArgumentParser(description="Live read-only fault and motion monitor for a Kinova Gen3.")
    parser.add_argument("--host", default="192.168.1.10")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--rate", type=float, default=10.0, help="Samples per second (default: 10).")
    parser.add_argument("--csv", help="Also append every sample to this CSV file.")
    args = parser.parse_args()

    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import ActuatorConfig_pb2, Base_pb2

    actuator_safety = ActuatorConfig_pb2.SafetyIdentifierBankA

    with kortex_session(args.host, args.port) as router:
        base = BaseClient(router)
        cyclic = BaseCyclicClient(router)

        zones = [z for z in base.ReadAllProtectionZones().protection_zones if z.is_enabled]
        targets = {}
        for action in base.ReadAllActions(Base_pb2.RequestedActionType()).action_list:
            if action.HasField("reach_joint_angles"):
                targets[action.name] = [a.value for a in action.reach_joint_angles.joint_angles.joint_angles]

        print(f"Monitoring {args.host} at {args.rate:g} Hz. Enabled zones: "
              f"{', '.join(z.name for z in zones) or 'none'}")
        print(f"Known action targets: {', '.join(targets) or 'none'}")
        print("Press the controller button under test. Ctrl-C to stop.\n")

        writer = None
        handle = None
        if args.csv:
            handle = open(args.csv, "a", newline="")
            writer = csv.writer(handle)
            writer.writerow(["t", "arm_state", "base_fault_a", "base_fault_b"]
                            + [f"j{i}" for i in range(1, 8)]
                            + [f"fault_a{i}" for i in range(1, 8)]
                            + [f"warn_a{i}" for i in range(1, 8)]
                            + [f"torque{i}" for i in range(1, 8)]
                            + [f"current{i}" for i in range(1, 8)]
                            + [f"vel{i}" for i in range(1, 8)]
                            + [f"tmot{i}" for i in range(1, 8)]
                            + ["tool_x", "tool_y", "tool_z", "worst_zone", "penetration_mm"])

        period = 1.0 / args.rate
        start = time.time()
        prev_faults = None
        prev_state = None
        still = 0
        announced = True
        try:
            while True:
                loop_start = time.time()
                feedback = cyclic.RefreshFeedback()
                now = time.time() - start
                angles = [a.position for a in feedback.actuators]
                speeds = [abs(a.velocity) for a in feedback.actuators]
                pose = (feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z)

                hits = [penetration(pose, z) for z in zones]
                worst = max(hits, key=lambda h: h.depth, default=None)

                faults = [feedback.base.fault_bank_a, feedback.base.fault_bank_b] + \
                         [a.fault_bank_a for a in feedback.actuators]
                state = Base_pb2.ArmState.Name(base.GetArmState().active_state)

                if state != prev_state:
                    print(f"[{now:7.2f}s] arm state -> {state}")
                    prev_state = state

                if faults != prev_faults and any(faults):
                    safety = Base_pb2.SafetyIdentifier
                    print(f"[{now:7.2f}s] FAULT base_a={faults[0]:#010x} ({_decode(faults[0], safety)}) "
                          f"base_b={faults[1]:#010x} ({_decode(faults[1])})")
                    for i, bank in enumerate(faults[2:], start=1):
                        if bank:
                            a = feedback.actuators[i - 1]
                            print(f"           actuator {i}: {bank:#010x} ({_decode(bank, actuator_safety)})")
                            print(f"             pos={a.position:.2f} vel={a.velocity:+.2f} "
                                  f"torque={a.torque:+.2f} Nm current={a.current_motor:+.2f} A "
                                  f"Tmot={a.temperature_motor:.1f}C")
                    if worst is not None:
                        print(f"           tool ({pose[0]:+.3f},{pose[1]:+.3f},{pose[2]:+.3f}) vs "
                              f"'{worst.zone_name}': {worst.depth * 1000:+.0f} mm "
                              f"({'INSIDE' if worst.violates else 'clear'})")
                prev_faults = faults

                moving = any(s > SETTLE_SPEED for s in speeds)
                if moving:
                    still = 0
                    announced = False
                else:
                    still += 1

                if still == SETTLE_SAMPLES and not announced:
                    announced = True
                    print(f"[{now:7.2f}s] settled at "
                          f"{[f'{a:.1f}' for a in angles]}")
                    print(f"           tool ({pose[0]:+.3f},{pose[1]:+.3f},{pose[2]:+.3f}) m")
                    best = None
                    for name, target in targets.items():
                        deltas = [_angle_delta(a, t) for a, t in zip(angles, target)]
                        worst_joint = max(range(len(deltas)), key=lambda i: abs(deltas[i]))
                        error = abs(deltas[worst_joint])
                        if best is None or error < best[1]:
                            best = (name, error, deltas, worst_joint)
                    if best:
                        name, error, deltas, worst_joint = best
                        if error <= MATCH_TOLERANCE:
                            print(f"           reached '{name}' (max joint error {error:.2f} deg)")
                        else:
                            print(f"           closest stored action is '{name}' but it STOPPED SHORT: "
                                  f"max error {error:.2f} deg on joint {worst_joint + 1}")
                            print(f"           per-joint error: {[f'{d:+.2f}' for d in deltas]}")
                    if worst is not None:
                        print(f"           '{worst.zone_name}' margin: {-worst.depth * 1000:+.0f} mm\n")

                if writer:
                    acts = feedback.actuators
                    writer.writerow([f"{now:.3f}", state, faults[0], faults[1]]
                                    + [f"{a:.3f}" for a in angles]
                                    + [a.fault_bank_a for a in acts]
                                    + [a.warning_bank_a for a in acts]
                                    + [f"{a.torque:.3f}" for a in acts]
                                    + [f"{a.current_motor:.3f}" for a in acts]
                                    + [f"{a.velocity:.3f}" for a in acts]
                                    + [f"{a.temperature_motor:.1f}" for a in acts]
                                    + [f"{c:.4f}" for c in pose]
                                    + [worst.zone_name if worst else "",
                                       f"{worst.depth * 1000:.1f}" if worst else ""])
                    handle.flush()

                time.sleep(max(0.0, period - (time.time() - loop_start)))
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            if handle:
                handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
