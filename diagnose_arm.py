import argparse
import json
from dataclasses import asdict

from kinova_debugger.hardware import run_hardware_checks
from kinova_debugger.models import CheckResult, Status
from kinova_debugger.network import run_network_checks
from kinova_debugger.software import run_software_checks


def print_results(results: list[CheckResult], as_json: bool) -> int:
    if as_json:
        print(json.dumps([asdict(result) for result in results], indent=2, default=str))
    else:
        for result in results:
            print(f"[{result.status.value:4}] {result.name}: {result.message}")
            if result.remediation:
                print(f"       Fix: {result.remediation}")
    return 1 if any(result.status == Status.FAIL for result in results) else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only diagnostics for a Kinova Gen3 7-DoF arm.")
    parser.add_argument("--host", default="192.168.1.10", help="Arm IP address (default: 192.168.1.10).")
    parser.add_argument("--port", type=int, default=10000, help="Kortex TCP port (default: 10000).")
    parser.add_argument("--timeout", type=float, default=3.0, help="Network timeout in seconds.")
    parser.add_argument("--software", action="store_true", help="Check Python and installed dependencies.")
    parser.add_argument("--network", action="store_true", help="Check TCP reachability to the arm.")
    parser.add_argument("--hardware", action="store_true", help="Create a read-only Kortex session and query the arm.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON output.")
    args = parser.parse_args()

    run_all = not any((args.software, args.network, args.hardware))
    results: list[CheckResult] = []
    if run_all or args.software:
        results.extend(run_software_checks())
    if run_all or args.network:
        results.extend(run_network_checks(args.host, args.port, args.timeout))
    if run_all or args.hardware:
        results.extend(run_hardware_checks(args.host, args.port, max(args.timeout, 5.0)))
    return print_results(results, args.json)


if __name__ == "__main__":
    raise SystemExit(main())
