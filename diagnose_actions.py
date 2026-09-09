import argparse

from diagnose_arm import print_results
from kinova_debugger.actions import run_action_checks


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check each stored arm action (Home, Retract, ...) against the enabled protection zones.")
    parser.add_argument("--host", default="192.168.1.10", help="Arm IP address (default: 192.168.1.10).")
    parser.add_argument("--port", type=int, default=10000, help="Kortex TCP port (default: 10000).")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")
    args = parser.parse_args()
    return print_results(run_action_checks(args.host, args.port), args.json)


if __name__ == "__main__":
    raise SystemExit(main())
