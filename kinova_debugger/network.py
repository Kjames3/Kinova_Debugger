import socket

from .models import CheckResult, Status


def check_tcp(host: str, port: int, timeout: float = 3.0) -> CheckResult:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return CheckResult(
                f"tcp-{port}",
                Status.PASS,
                f"{host}:{port} accepted a TCP connection.",
            )
    except socket.timeout:
        return CheckResult(
            f"tcp-{port}",
            Status.FAIL,
            f"Connection to {host}:{port} timed out.",
            "Check the arm IP address, Ethernet/Wi-Fi route, and firewall rules.",
        )
    except OSError as error:
        return CheckResult(
            f"tcp-{port}",
            Status.FAIL,
            f"Could not connect to {host}:{port}: {error}",
            "Confirm the arm is powered on, reachable from this computer, and using the expected IP address.",
        )


def run_network_checks(host: str, port: int = 10000, timeout: float = 3.0) -> list[CheckResult]:
    return [check_tcp(host, port, timeout)]
