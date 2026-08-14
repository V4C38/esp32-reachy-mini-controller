"""
Motion Controller Reachy Mini app entry point (protocol v4).

FastAPI/uvicorn HTTP on TCP 8766, UDP datagrams on UDP 8766,
mDNS advertise _reachyctl._tcp, clutch + safety bridge.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import socket
import sys
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from esp32_motion_controller.protocol import PROTOCOL_VERSION
from esp32_motion_controller.robot_control import RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub
from esp32_motion_controller import __version__

logger = logging.getLogger(__name__)

LINK_PORT = 8766
STATIC_DIR = Path(__file__).parent / "static"
MDNS_SERVICE_TYPE = "_reachyctl._tcp.local."
SERVER_START_TIMEOUT_S = 10.0
UDP_PROBE = b"RMC2?"


def get_local_ips() -> list[str]:
    ips: list[str] = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addr = info[4][0]
            if not addr.startswith("127."):
                ips.append(addr)
    except (socket.gaierror, OSError):
        pass
    return list(dict.fromkeys(ips))


class ControllerProtocol(asyncio.DatagramProtocol):
    """One UDP socket: discovery probes + protocol v4 datagrams."""

    def __init__(self, session: SessionHub) -> None:
        self.session = session
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]
        self.session.bind_transport(transport)

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        if data.startswith(UDP_PROBE):
            self._reply_probe(addr)
            return
        asyncio.create_task(self.session.handle_datagram(data, addr))

    def error_received(self, exc: Exception) -> None:
        logger.warning("UDP error: %s", exc)

    def _reply_probe(self, addr: tuple[str, int]) -> None:
        if self.transport is None:
            return
        ips = get_local_ips()
        if not ips:
            return
        try:
            self.transport.sendto(f"RMC2 {ips[0]} {LINK_PORT}".encode(), addr)
        except OSError:
            pass


class MdnsAdvertiser:
    def __init__(self, port: int = LINK_PORT) -> None:
        self.port = port
        self._zc = None
        self._info = None

    def start(self) -> None:
        """Advertise over mDNS. Never raises — discovery is a convenience."""
        try:
            from zeroconf import ServiceInfo, Zeroconf
        except ImportError:
            logger.warning("zeroconf not installed; mDNS advertise skipped")
            return

        zc = None
        try:
            ips = get_local_ips()
            if not ips:
                logger.warning("No local IP for mDNS advertise")
                return
            info = ServiceInfo(
                MDNS_SERVICE_TYPE,
                f"esp32-motion-controller.{MDNS_SERVICE_TYPE}",
                addresses=[socket.inet_aton(ip) for ip in ips],
                port=self.port,
                properties={"protocol": b"3", "transport": b"udp"},
                server="esp32-motion.local.",
            )
            zc = Zeroconf()
            zc.register_service(info, allow_name_change=True)
        except Exception:
            logger.warning(
                "mDNS advertise failed; controller must use a configured host",
                exc_info=True,
            )
            if zc is not None:
                try:
                    zc.close()
                except Exception:
                    pass
            return

        self._zc = zc
        self._info = info
        logger.info("mDNS advertised %s on %s:%d", info.name, ips[0], self.port)

    def stop(self) -> None:
        if self._zc is not None:
            try:
                if self._info is not None:
                    self._zc.unregister_service(self._info)
            except Exception:
                pass
            try:
                self._zc.close()
            except Exception:
                pass
        self._zc = None
        self._info = None


def create_app(
    reachy_mini,
    stop_event: threading.Event,
    *,
    log_only: bool = False,
) -> FastAPI:
    app = FastAPI(title="ESP32 Motion Controller")
    robot_available = reachy_mini is not None and not log_only
    session = SessionHub(robot_available=robot_available or log_only)
    gateway = RobotGateway(reachy_mini, log_only=log_only)
    control = RobotControl(
        session,
        gateway,
        robot_available=robot_available,
        log_only=log_only,
    )
    session.on_hello = control.on_controller_hello
    app.state.session = session
    app.state.control = control

    @app.on_event("startup")
    async def _startup() -> None:
        loop = asyncio.get_running_loop()
        transport, _protocol = await loop.create_datagram_endpoint(
            lambda: ControllerProtocol(session),
            local_addr=("0.0.0.0", LINK_PORT),
        )
        app.state.udp_transport = transport
        logger.info("UDP link listening on 0.0.0.0:%d", LINK_PORT)
        control.start()

    @app.on_event("shutdown")
    async def _shutdown() -> None:
        await control.stop()
        transport = getattr(app.state, "udp_transport", None)
        if transport is not None:
            transport.close()

    @app.get("/api/info")
    async def info() -> JSONResponse:
        ips = get_local_ips()
        return JSONResponse(
            {
                "ips": ips,
                "port": LINK_PORT,
                "udp": f"{ips[0]}:{LINK_PORT}" if ips else None,
                "mdns": MDNS_SERVICE_TYPE,
                "protocol_version": PROTOCOL_VERSION,
            }
        )

    @app.get("/api/status")
    async def status() -> JSONResponse:
        snap = await session.snapshot_status()
        return JSONResponse(
            {
                "status": "ok",
                "robot": robot_available or log_only,
                "busy": snap["busy"],
                "mode": snap["mode"],
                "connected": snap["connected"],
                "log_only": log_only,
                "protocol_version": PROTOCOL_VERSION,
                "app_version": __version__,
                "boot_id": snap["boot_id"],
                "peer": snap["peer"],
                "presents": snap["presents"],
                "absents": snap["absents"],
                "last_seq": snap["last_seq"],
                "seq_skips": snap["seq_skips"],
                "sample_gaps": snap["sample_gaps"],
                "last_rx_age_ms": snap["last_rx_age_ms"],
                "last_diag": snap["last_diag"],
                "max_tick_lag_ms": snap["max_tick_lag_ms"],
                "last_sdk_ms": snap["last_sdk_ms"],
            }
        )

    @app.get("/")
    async def root() -> FileResponse:
        return FileResponse(STATIC_DIR / "index.html")

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    return app


def _run_server(reachy_mini, stop_event: threading.Event, *, log_only: bool) -> None:
    ips = get_local_ips()
    logger.info("=" * 50)
    logger.info("ESP32 Motion Controller (protocol v4) app_version=%s", __version__)
    logger.info("=" * 50)
    for ip in ips:
        logger.info("  UDP: %s:%d", ip, LINK_PORT)
    if log_only:
        logger.info("  Mode: --log-only (no robot SDK calls)")
    logger.info("=" * 50)

    app = create_app(reachy_mini, stop_event, log_only=log_only)
    config = uvicorn.Config(
        app,
        host="0.0.0.0",
        port=LINK_PORT,
        log_level="info",
    )
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()

    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    while not server.started and thread.is_alive() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not server.started:
        raise RuntimeError(
            f"Could not serve on port {LINK_PORT} — another Motion Controller "
            f"instance is probably already running"
        )

    mdns = MdnsAdvertiser(LINK_PORT)
    mdns.start()

    stop_event.wait()
    server.should_exit = True
    thread.join(timeout=5)
    mdns.stop()
    logger.info("Motion Controller stopped")


try:
    from reachy_mini import ReachyMini, ReachyMiniApp

    class Esp32MotionController(ReachyMiniApp):
        """Reachy Mini App that bridges an ESP32 motion controller."""

        name = "ESP32 Motion Controller"
        emoji = "🕹️"
        custom_app_url: str | None = None

        def run(self, reachy_mini: ReachyMini, stop_event: threading.Event) -> None:
            _run_server(reachy_mini, stop_event, log_only=False)

except ImportError:
    class Esp32MotionController:  # type: ignore[no-redef]
        name = "ESP32 Motion Controller"
        emoji = "🕹️"
        custom_app_url = None

        def run(self, reachy_mini, stop_event: threading.Event) -> None:
            _run_server(reachy_mini, stop_event, log_only=False)

        def wrapped_run(self) -> None:
            raise RuntimeError("reachy-mini SDK is required unless using --log-only")

        def stop(self) -> None:
            pass


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    parser = argparse.ArgumentParser(description="ESP32 Motion Controller for Reachy Mini")
    parser.add_argument(
        "--log-only",
        action="store_true",
        help="Run without a ReachyMini SDK instance; log decoded controller state",
    )
    args = parser.parse_args(argv)
    stop = threading.Event()

    if args.log_only:
        try:
            _run_server(None, stop, log_only=True)
        except KeyboardInterrupt:
            stop.set()
        return 0

    app = Esp32MotionController()
    try:
        app.wrapped_run()
    except KeyboardInterrupt:
        app.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
