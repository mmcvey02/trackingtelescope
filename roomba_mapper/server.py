"""HTTP server: JSON API plus the mobile web GUI (open it on your phone)."""

import hmac
import json
import mimetypes
import os
import socket
import ssl
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MAX_BODY = 1 << 20


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def _int(value, name):
    if isinstance(value, bool):
        raise ApiError(400, f"'{name}' must be an integer")
    try:
        return int(value)
    except (TypeError, ValueError):
        raise ApiError(400, f"'{name}' must be an integer")


def geojson(controller):
    with controller.lock:
        elements = [dict(el) for el in controller.map.elements]
        locked = controller.map.locked
    features = [{
        "type": "Feature",
        "properties": {"id": el["id"], "kind": el["kind"], "source": el["source"], "name": el["name"]},
        "geometry": {"type": "Polygon", "coordinates": [el["points"] + [el["points"][0]]]},
    } for el in elements]
    return {"type": "FeatureCollection", "properties": {"units": "metres", "origin": "dock",
                                                         "locked": locked},
            "features": features}


def make_handler(controller, pin=None):
    def map_action(body):
        action = body.get("action")
        actions = {
            "rebuild": controller.rebuild,
            "lock": lambda: controller.set_locked(True),
            "unlock": lambda: controller.set_locked(False),
            "reset_coverage": controller.reset_coverage,
            "erase": controller.erase_map,
        }
        if action not in actions:
            raise ApiError(400, f"action must be one of {sorted(actions)}")
        actions[action]()

    routes = {
        "/api/command": lambda b: controller.command(b.get("command"), b.get("mode")),
        "/api/shapes": lambda b: controller.add_shape(b.get("kind"), b.get("points"), b.get("name")),
        "/api/shapes/update": lambda b: controller.update_shape(
            _int(b.get("id"), "id"), b.get("points"), b.get("kind"), b.get("name")),
        "/api/shapes/delete": lambda b: controller.delete_shape(_int(b.get("id"), "id")),
        "/api/map": map_action,
        "/api/settings": lambda b: controller.update_settings(b),
    }

    class Handler(BaseHTTPRequestHandler):
        server_version = "RoombaMapper/2.0"

        def log_message(self, fmt, *args):  # keep the console quiet
            pass

        def _send(self, status, body, content_type="application/json", extra=None):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _authorised(self):
            if not pin:
                return True
            given = self.headers.get("X-Pin", "")
            return hmac.compare_digest(given.encode(), pin.encode())

        def do_GET(self):
            url = urlparse(self.path)
            path = url.path
            if path.startswith("/api/") and path != "/api/info" and not self._authorised():
                return self._send(401, {"error": "PIN required", "pin_required": True})
            if path == "/api/state":
                q = parse_qs(url.query)
                return self._send(200, controller.snapshot(
                    have_map=(q.get("map") or [None])[0], have_cov=(q.get("cov") or [None])[0]))
            if path == "/api/export":
                return self._send(200, json.dumps(geojson(controller), indent=1).encode(),
                                  "application/geo+json",
                                  {"Content-Disposition": 'attachment; filename="roomba-map.geojson"'})
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
                result = routes[path](body)
            except ApiError as exc:
                return self._send(exc.status, {"error": str(exc)})
            except (ValueError, TypeError, KeyError, IndexError) as exc:
                return self._send(400, {"error": str(exc)})
            state = controller.snapshot()
            if isinstance(result, int) and not isinstance(result, bool):
                state["created_id"] = result
            self._send(200, state)

    return Handler


def make_server(controller, host="0.0.0.0", port=8080, pin=None, ssl_context=None):
    server = ThreadingHTTPServer((host, port), make_handler(controller, pin))
    if ssl_context is not None:
        server.socket = ssl_context.wrap_socket(server.socket, server_side=True)
    return server


def https_context(cert_path, key_path, days=3650):
    """TLS context for serving the app over https.

    Phones only let a web page read their motion sensors (used by
    walk-to-map) over https. A self-signed certificate is made once with
    the `openssl` tool (built into macOS and Linux) and reused; the phone
    shows a one-time warning that has to be accepted.
    """
    if not (os.path.exists(cert_path) and os.path.exists(key_path)):
        try:
            subprocess.run(
                ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", str(days),
                 "-subj", "/CN=roomba-mapper", "-keyout", key_path, "-out", cert_path],
                check=True, capture_output=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            raise RuntimeError("could not create a certificate - https needs the `openssl` "
                               f"command ({exc})") from exc
        try:
            os.chmod(key_path, 0o600)
        except OSError:
            pass
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert_path, key_path)
    return ctx


def lan_address():
    """Best guess at this machine's address on the local network."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))  # no packets are sent
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
