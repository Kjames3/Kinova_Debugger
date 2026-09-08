import argparse

from diagnose_arm import print_results
from kinova_debugger.hardware import run_hardware_checks


def main() -> int:
    parser = argparse.ArgumentParser(description="Run read-only Kortex checks against a Kinova Gen3 arm.")
    parser.add_argument("host", help="Arm IP address.")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    return print_results(run_hardware_checks(args.host, args.port, args.timeout), False)


if __name__ == "__main__":
    raise SystemExit(main())
