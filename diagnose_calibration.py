import argparse

from diagnose_arm import print_results
from kinova_debugger.calibration import run_calibration_checks, run_capture_session, run_offset_analysis


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check Kinova Gen3 joint calibration against the wrist IMU. Never moves the arm.")
    commands = parser.add_subparsers(dest="command", required=True)

    compare = commands.add_parser(
        "compare", help="Quick single-pose check of wrist tilt against a known-good arm's reading.")
    compare.add_argument("--host", default="192.168.1.10", help="Arm IP address (default: 192.168.1.10).")
    compare.add_argument("--port", type=int, default=10000, help="Kortex TCP port (default: 10000).")
    compare.add_argument("--save-reference", metavar="FILE",
                         help="Write this arm's reading to FILE, to compare another arm against.")
    compare.add_argument("--reference", metavar="FILE", help="Compare against a reading saved by --save-reference.")
    compare.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")

    capture = commands.add_parser(
        "capture", help="Interactively record static poses (you jog the arm) for per-joint offset analysis.")
    capture.add_argument("--host", default="192.168.1.10", help="Arm IP address (default: 192.168.1.10).")
    capture.add_argument("--port", type=int, default=10000, help="Kortex TCP port (default: 10000).")
    capture.add_argument("--out", required=True, metavar="FILE",
                         help="Pose file to write. An existing file from the same arm is appended to.")
    capture.add_argument("--new", action="store_true", help="Start a fresh file instead of appending.")

    analyze = commands.add_parser("analyze", help="Estimate each joint's zero offset from a captured pose file.")
    analyze.add_argument("poses", metavar="FILE", help="Pose file written by capture.")
    analyze.add_argument("--baseline", metavar="FILE",
                         help="Pose file from a known-good arm. Pins down the IMU mounting and enables joint 7.")
    analyze.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")

    args = parser.parse_args()
    if args.command == "compare":
        return print_results(
            run_calibration_checks(args.host, args.port, args.reference, args.save_reference), args.json)
    if args.command == "capture":
        return run_capture_session(args.host, args.port, args.out, args.new)
    return print_results(run_offset_analysis(args.poses, args.baseline), args.json)


if __name__ == "__main__":
    raise SystemExit(main())
