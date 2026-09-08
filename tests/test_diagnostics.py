import unittest

from kinova_debugger.models import Status
from kinova_debugger.network import check_tcp


class DiagnosticTests(unittest.TestCase):
    def test_unreachable_tcp_port_returns_failure(self) -> None:
        result = check_tcp("127.0.0.1", 1, timeout=0.1)
        self.assertEqual(result.status, Status.FAIL)


if __name__ == "__main__":
    unittest.main()
