import importlib.util
import platform
import sys
from pathlib import Path

from .models import CheckResult, Status


def run_software_checks() -> list[CheckResult]:
    checks = [
        CheckResult(
            "python-version",
            Status.PASS if sys.version_info >= (3, 9) else Status.FAIL,
            f"Python {platform.python_version()} detected.",
            "Install Python 3.9 or newer and select it in VS Code." if sys.version_info < (3, 9) else "",
        ),
        CheckResult(
            "platform",
            Status.PASS,
            f"Running on {platform.system()} {platform.release()} ({platform.machine()}).",
        ),
    ]

    kortex_available = importlib.util.find_spec("kortex_api") is not None
    checks.append(
        CheckResult(
            "kortex-api",
            Status.PASS if kortex_available else Status.FAIL,
            "kortex_api is installed." if kortex_available else "kortex_api is not installed.",
            "Install the Kortex wheel by path; see the Setup section of README.md." if not kortex_available else "",
        )
    )

    workspace = Path.cwd()
    checks.append(
        CheckResult(
            "workspace",
            Status.PASS if workspace.exists() else Status.FAIL,
            f"Working directory exists: {workspace}",
        )
    )
    return checks
