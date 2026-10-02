"""HTTP server: JSON API plus the mobile web GUI (open it on your phone)."""

import hmac
import json
import mimetypes
import os
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_BODY = 1 << 20


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def _int(value, name):
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ApiError(400, f"'{name}' must be an integer")


def make_handler(controller, pin=None):
    routes = {
        "/api/mode": lambda b: controller.set_mode(b.get("mode")),
        "/api/drive": lambda b: controller.drive(b.get("direction")),
        "/api/goto": lambda b: controller.goto(_int(b.get("x"), "x"), _int(b.get("y"), "y")),
        "/api/cells": lambda b: controller.set_cells(_cells(b.get("cells")), b.get("state")),
        "/api/dock": lambda b: controller.set_dock(_int(b.get("x"), "x"), _int(b.get("y"), "y")),
        "/api/robot": lambda b: controller.set_pose(
            _int(b.get("x"), "x"), _int(b.get("y"), "y"), b.get("heading")),
        "/api/reset": lambda b: controller.reset(b.get("scope")),
        "/api/resize": lambda b: controller.resize(
            _int(b.get("width"), "width"), _int(b.get("height"), "height")),
        "/api/settings": lambda b: controller.update_settings(b),
    }

    def _cells(value):
        if not isinstance(value, list) or len(value) > 40000:
            raise ApiError(400, "'cells' must be a list of [x, y] pairs")
        out = []
        for item in value:
            if not (isinstance(item, (list, tuple)) and len(item) == 2):
                raise ApiError(400, "'cells' must be a list of [x, y] pairs")
            out.append((_int(item[0], "x"), _int(item[1], "y")))
        return out

    class Handler(BaseHTTPRequestHandler):
        server_version = "RoombaMapper/1.0"

        def log_message(self, fmt, *args):  # keep the console quiet
            pass

        def _send(self, status, body, content_type="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _authorised(self):
            if not pin:
                return True
            given = self.headers.get("X-Pin", "")
            return hmac.compare_digest(given.encode(), pin.encode())

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/state":
                if not self._authorised():
                    return self._send(401, {"error": "PIN required", "pin_required": True})
                return self._send(200, controller.snapshot())
            if path == "/api/info":
                return self._send(200, {"pin_required": bool(pin)})
            if path == "/":
                path = "/index.html"
            name = os.path.normpath(path.lstrip("/"))
            full = os.path.join(STATIC_DIR, name)
            if name.startswith("..") or os.path.isabs(name) or not os.path.isfile(full):
                return self._send(404, {"error": "not found"})
            ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
            if name.endswith(".webmanifest"):
                ctype = "application/manifest+json"
            with open(full, "rb") as fh:
                self._send(200, fh.read(), ctype)

        def do_POST(self):
            path = urlparse(self.path).path
            if path not in routes:
                return self._send(404, {"error": "not found"})
            if not self._authorised():
                return self._send(401, {"error": "PIN required", "pin_required": True})
            try:
                length = int(self.headers.get("Content-Length") or 0)
                if length > MAX_BODY:
                    raise ApiError(413, "request too large")
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    body = json.loads(raw or b"{}")
                except ValueError:
                    raise ApiError(400, "body must be JSON")
                if not isinstance(body, dict):
                    raise ApiError(400, "body must be a JSON object")
                routes[path](body)
            except ApiError as exc:
                return self._send(exc.status, {"error": str(exc)})
            except (ValueError, TypeError, IndexError) as exc:
                return self._send(400, {"error": str(exc)})
            self._send(200, controller.snapshot())

    return Handler


def make_server(controller, host="0.0.0.0", port=8080, pin=None):
    return ThreadingHTTPServer((host, port), make_handler(controller, pin))


def lan_address():
    """Best guess at this machine's address on the local network."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packets are sent
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
