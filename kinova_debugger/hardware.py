from typing import Any

from .models import CheckResult, Status


def _client_call(client: Any, method_name: str) -> Any:
    method = getattr(client, method_name, None)
    if method is None:
        raise AttributeError(f"Kortex client does not expose {method_name}()")
    return method()


def run_hardware_checks(host: str, port: int = 10000, timeout: float = 5.0) -> list[CheckResult]:
    try:
        from kortex_api.TCPTransport import TCPTransport
        from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
        from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
        from kortex_api.autogen.messages import Session_pb2
    except ImportError as error:
        return [
            CheckResult(
                "kortex-import",
                Status.FAIL,
                f"Kortex SDK imports are unavailable: {error}",
                "Install the dependencies with: python -m pip install -r requirements.txt",
            )
        ]

    transport = TCPTransport()
    router = None
    try:
        transport.connect(host, port)
        router = transport.create_router()
        session_manager = router.create_client_session_manager()
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = "admin"
        session_info.password = "admin"
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000
        session_manager.create_session(session_info)

        base_client = BaseClient(router)
        device_manager = DeviceManagerClient(router)
        results = [
            CheckResult("kortex-connection", Status.PASS, f"Connected to the arm at {host}:{port}."),
        ]
        for name, client, method in (
            ("product-configuration", base_client, "GetProductConfiguration"),
            ("device-inventory", device_manager, "ReadAllDevices"),
        ):
            try:
                response = _client_call(client, method)
                results.append(CheckResult(name, Status.PASS, f"{method}() succeeded.", details={"response": str(response)}))
            except Exception as error:
                results.append(
                    CheckResult(
                        name,
                        Status.FAIL,
                        f"{method}() failed: {error}",
                        "Check the arm firmware, session credentials, and Kortex API compatibility.",
                    )
                )
        return results
    except Exception as error:
        return [
            CheckResult(
                "kortex-connection",
                Status.FAIL,
                f"Could not create a Kortex session: {error}",
                "Check the IP address, port, credentials, network route, and arm operating state.",
            )
        ]
    finally:
        if router is not None:
            try:
                router.destroy()
            except Exception:
                pass
        try:
            transport.disconnect()
        except Exception:
            pass
