from diagnose_arm import print_results
from kinova_debugger.software import run_software_checks


if __name__ == "__main__":
    raise SystemExit(print_results(run_software_checks(), False))
