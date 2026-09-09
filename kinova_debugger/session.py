"""Shared read-only Kortex session handling."""

from contextlib import contextmanager

DEFAULT_USERNAME = "admin"
DEFAULT_PASSWORD = "admin"


@contextmanager
def kortex_session(host: str, port: int = 10000, username: str = DEFAULT_USERNAME, password: str = DEFAULT_PASSWORD):
    """Yield a connected RouterClient, closing the session and transport on exit."""
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.autogen.messages import Session_pb2

    transport = TCPTransport()
    session_manager = None
    try:
        transport.connect(host, port)
        router = RouterClient(transport, RouterClient.basicErrorCallback)
        session_manager = SessionManager(router)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = username
        session_info.password = password
        session_info.session_inactivity_timeout = 60000
        session_info.connection_inactivity_timeout = 2000
        session_manager.CreateSession(session_info)
        yield router
    finally:
        if session_manager is not None:
            try:
                session_manager.CloseSession()
            except Exception:
                pass
        try:
            transport.disconnect()
        except Exception:
            pass
