"""
Motion Controller Reachy Mini app entry point (protocol v2).

FastAPI/uvicorn on port 8766, mDNS advertise _reachyctl._tcp, clutch + safety bridge.
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from esp32_motion_controller.robot_control import RobotControl, RobotGateway
from esp32_motion_controller.session import SessionHub

logger = logging.getLogger(__name__)

WS_PORT = 8766
STATIC_DIR = Path(__file__).parent / "static"
MDNS_SERVICE_TYPE = "_reachyctl._tcp.local."
SERVER_START_TIMEOUT_S = 10.0


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


class UdpAdvertiser:
    """Answer ESP32 subnet-broadcast probes on UDP WS_PORT.

    Dual-band APs commonly forward unicast/broadcast while dropping mDNS
    multicast between 2.4 GHz (ESP32-S3) and 5 GHz (desktop).
    """

    def __init__(self, port: int = WS_PORT) -> None:
        self.port = port
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind(("0.0.0.0", self.port))
            sock.settimeout(0.5)
        except OSError:
            logger.warning("UDP discovery bind failed on port %d", self.port, exc_info=True)
            return
        self._sock = sock
        self._thread = threading.Thread(target=self._loop, name="udp-discovery", daemon=True)
        self._thread.start()
        logger.info("UDP discovery listening on 0.0.0.0:%d", self.port)

    def _loop(self) -> None:
        sock = self._sock
        if sock is None:
            return
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(64)
            except TimeoutError:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            if not data.startswith(b"RMC2?"):
                continue
            ips = get_local_ips()
            if not ips:
                continue
            reply = f"RMC2 {ips[0]} {self.port}".encode()
            try:
                sock.sendto(reply, addr)
            except OSError:
                pass

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._sock = None
        self._thread = None


class MdnsAdvertiser:
    def __init__(self, port: int = WS_PORT) -> None:
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
                properties={"path": b"/ws", "protocol": b"2"},
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
    app.state.session = session
    app.state.control = control

    @app.on_event("startup")
    async def _startup() -> None:
        control.start()

    @app.on_event("shutdown")
    async def _shutdown() -> None:
        await control.stop()

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        generation = await session.on_connect(websocket)
        await control.on_controller_connected()
        try:
            while not stop_event.is_set():
                raw = await websocket.receive_text()
                await session.handle_message(websocket, generation, raw)
        except WebSocketDisconnect:
            logger.info("WebSocket disconnect")
        except Exception as exc:
            logger.error("WebSocket error: %s", exc)
        finally:
            await session.cleanup(websocket, generation)
            await control.on_controller_disconnected()

    @app.get("/api/info")
    async def info() -> JSONResponse:
        ips = get_local_ips()
        return JSONResponse(
            {
                "ips": ips,
                "port": WS_PORT,
                "ws_url": f"ws://{ips[0]}:{WS_PORT}/ws" if ips else None,
                "mdns": MDNS_SERVICE_TYPE,
                "protocol_version": 2,
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
                "protocol_version": 2,
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
    logger.info("ESP32 Motion Controller (protocol v2)")
    logger.info("=" * 50)
    for ip in ips:
        logger.info("  WebSocket: ws://%s:%d/ws", ip, WS_PORT)
    if log_only:
        logger.info("  Mode: --log-only (no robot SDK calls)")
    logger.info("=" * 50)

    app = create_app(reachy_mini, stop_event, log_only=log_only)
    config = uvicorn.Config(app, host="0.0.0.0", port=WS_PORT, log_level="info")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()

    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    while not server.started and thread.is_alive() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not server.started:
        raise RuntimeError(
            f"Could not serve on port {WS_PORT} — another Motion Controller "
            f"instance is probably already running"
        )

    mdns = MdnsAdvertiser(WS_PORT)
    mdns.start()
    udp = UdpAdvertiser(WS_PORT)
    udp.start()

    stop_event.wait()
    # Shutdown path: uvicorn will fire FastAPI shutdown hooks.
    server.should_exit = True
    thread.join(timeout=5)
    udp.stop()
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
