"""
Motion Controller Reachy Mini app entry point.

FastAPI/uvicorn on port 8766, mDNS advertise _reachyctl._tcp, clutch + behavior bridge.
"""

from __future__ import annotations

import argparse
import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from esp32_motion_controller.behavior import Behavior
from esp32_motion_controller.controller_state import ControllerState
from esp32_motion_controller.movement_handler import MovementHandler
from esp32_motion_controller.ws_handler import WebSocketHandler

logger = logging.getLogger(__name__)

WS_PORT = 8766
STATIC_DIR = Path(__file__).parent / "static"
MDNS_SERVICE_TYPE = "_reachyctl._tcp.local."
ENV_SEND_RATE_HZ = "REACHY_MOTION_SEND_RATE_HZ"
DEFAULT_SEND_RATE_HZ = 20.0
SERVER_START_TIMEOUT_S = 10.0


def _get_send_rate_hz() -> float:
    raw = os.environ.get(ENV_SEND_RATE_HZ)
    if raw is None or raw.strip() == "":
        return DEFAULT_SEND_RATE_HZ
    try:
        return max(5.0, min(50.0, float(raw.strip())))
    except ValueError:
        return DEFAULT_SEND_RATE_HZ


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
                properties={"path": b"/ws"},
                server="esp32-motion.local.",
            )
            zc = Zeroconf()
            # An advertisement from a previous run lingers in the mDNS cache for
            # the record TTL (75 min), so a restart collides with itself.
            # Renaming is safe: the controller browses the service type, never
            # the instance name.
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
    send_rate = _get_send_rate_hz()
    robot_available = reachy_mini is not None and not log_only
    movement = MovementHandler(reachy_mini, send_rate_hz=send_rate)
    controller = ControllerState()
    behavior = Behavior()
    ws_handler = WebSocketHandler(
        movement,
        controller,
        behavior,
        robot_available=robot_available or log_only,
        log_only=log_only,
    )
    app.state.ws_handler = ws_handler

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        admitted = await ws_handler.on_connect(websocket)
        if not admitted:
            return
        try:
            while not stop_event.is_set():
                raw = await websocket.receive_text()
                await ws_handler.handle_message(websocket, raw)
        except WebSocketDisconnect:
            logger.info("WebSocket disconnect")
        except Exception as exc:
            logger.error("WebSocket error: %s", exc)
        finally:
            ws_handler.cleanup(websocket)

    @app.get("/api/info")
    async def info() -> JSONResponse:
        ips = get_local_ips()
        return JSONResponse(
            {
                "ips": ips,
                "port": WS_PORT,
                "ws_url": f"ws://{ips[0]}:{WS_PORT}/ws" if ips else None,
                "mdns": MDNS_SERVICE_TYPE,
            }
        )

    @app.get("/api/status")
    async def status() -> JSONResponse:
        return JSONResponse(
            {
                "status": "ok",
                "robot": robot_available or log_only,
                "busy": ws_handler.busy,
                "log_only": log_only,
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
    logger.info("ESP32 Motion Controller")
    logger.info("=" * 50)
    for ip in ips:
        logger.info("  WebSocket: ws://%s:%d/ws", ip, WS_PORT)
    if log_only:
        logger.info("  Mode: --log-only (no robot SDK calls)")
    logger.info("=" * 50)

    app = create_app(reachy_mini, stop_event, log_only=log_only)
    # Stash for shutdown — create_app closes over the handler.
    ws_handler: WebSocketHandler = app.state.ws_handler  # type: ignore[attr-defined]
    config = uvicorn.Config(app, host="0.0.0.0", port=WS_PORT, log_level="info")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()

    # A failed bind only kills the server thread, so without this the app would
    # sit here advertising a port nothing listens on.
    deadline = time.monotonic() + SERVER_START_TIMEOUT_S
    while not server.started and thread.is_alive() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not server.started:
        raise RuntimeError(
            f"Could not serve on port {WS_PORT} — another Motion Controller "
            f"instance is probably already running"
        )

    # Advertise only once we are actually listening.
    mdns = MdnsAdvertiser(WS_PORT)
    mdns.start()

    stop_event.wait()
    ws_handler.shutdown()
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
