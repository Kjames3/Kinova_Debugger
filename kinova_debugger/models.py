from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Status(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class CheckResult:
    name: str
    status: Status
    message: str
    remediation: str = ""
    details: dict[str, Any] = field(default_factory=dict)
