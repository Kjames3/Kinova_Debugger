import argparse

from diagnose_arm import print_results
from kinova_debugger.calibration import run_calibration_checks


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check joint calibration by comparing the wrist IMU against a known-good arm.")
    parser.add_argument("--host", default="192.168.1.10", help="Arm IP address (default: 192.168.1.10).")
    parser.add_argument("--port", type=int, default=10000, help="Kortex TCP port (default: 10000).")
    parser.add_argument("--save-reference", metavar="FILE",
                        help="Write this arm's reading to FILE, to compare another arm against.")
    parser.add_argument("--reference", metavar="FILE",
                        help="Compare against a reading saved by --save-reference.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")
    args = parser.parse_args()
    return print_results(
        run_calibration_checks(args.host, args.port, args.reference, args.save_reference), args.json)


if __name__ == "__main__":
    raise SystemExit(main())
