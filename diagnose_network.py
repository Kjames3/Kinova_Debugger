import argparse

from kinova_debugger.network import run_network_checks
from diagnose_arm import print_results


def main() -> int:
    parser = argparse.ArgumentParser(description="Check network reachability to a Kinova Gen3 arm.")
    parser.add_argument("host", help="Arm IP address.")
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    return print_results(run_network_checks(args.host, args.port, args.timeout), False)


if __name__ == "__main__":
    raise SystemExit(main())
