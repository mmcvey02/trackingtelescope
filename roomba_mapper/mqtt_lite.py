"""A small MQTT 3.1.1 client (and test broker) using only the standard library.

Roombas talk MQTT over TLS on their own Wi-Fi radio. This client implements
just what that needs - CONNECT with username/password, SUBSCRIBE, QoS 0/1
PUBLISH, keep-alive - so the app has no third-party dependencies.
"""

import socket
import ssl
import struct
import threading

CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, PINGREQ, PINGRESP, DISCONNECT = 8, 9, 12, 13, 14

CONNACK_ERRORS = {
    1: "unacceptable protocol version",
    2: "client identifier rejected",
    3: "server unavailable",
    4: "bad username or password",
    5: "not authorised",
}


class MQTTError(Exception):
    pass


def _str(s):
    b = s.encode() if isinstance(s, str) else bytes(s)
    return struct.pack(">H", len(b)) + b


def _remaining_length(n):
    out = bytearray()
    while True:
        byte = n % 128
        n //= 128
        if n:
            byte |= 0x80
        out.append(byte)
        if not n:
            return bytes(out)


def packet(ptype, flags, body):
    return bytes([(ptype << 4) | flags]) + _remaining_length(len(body)) + body


def read_packet(sock):
    """Read one packet; returns (type, flags, body). Raises on EOF."""
    head = _recv_exact(sock, 1)
    mult, length = 1, 0
    for _ in range(4):
        b = _recv_exact(sock, 1)[0]
        length += (b & 0x7F) * mult
        if not b & 0x80:
            break
        mult *= 128
    else:
        raise MQTTError("malformed remaining length")
    body = _recv_exact(sock, length) if length else b""
    return head[0] >> 4, head[0] & 0x0F, body


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf.extend(chunk)
    return bytes(buf)


def parse_publish(flags, body):
    tlen = struct.unpack(">H", body[:2])[0]
    topic = body[2: 2 + tlen].decode("utf-8", "replace")
    pos = 2 + tlen
    qos = (flags >> 1) & 3
    pid = None
    if qos:
        pid = struct.unpack(">H", body[pos: pos + 2])[0]
        pos += 2
    return topic, body[pos:], qos, pid


def roomba_tls_context():
    """TLS settings that Roomba firmware accepts.

    Robots use a self-signed certificate and old cipher suites, so
    verification is off and the security level is lowered. The link only
    ever goes to the robot's address on your own network.
    """
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    try:
        ctx.set_ciphers("DEFAULT:@SECLEVEL=0")
    except ssl.SSLError:  # pragma: no cover - depends on OpenSSL build
        pass
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.options |= getattr(ssl, "OP_LEGACY_SERVER_CONNECT", 0x4)
    return ctx


class MQTTClient:
    def __init__(self, host, port, client_id, username=None, password=None,
                 tls=True, keepalive=30, timeout=10.0, ssl_context=None):
        self.host, self.port = host, port
        self.client_id = client_id
        self.username, self.password = username, password
        self.tls = tls
        self.keepalive = keepalive
        self.timeout = timeout
        self.ssl_context = ssl_context
        self.on_message = None     # callable(topic, payload_bytes)
        self.on_disconnect = None  # callable(exception_or_None)
        self._sock = None
        self._send_lock = threading.Lock()
        self._pid = 0
        self._thread = None
        self._closing = False
        self.connected = False

    def _next_pid(self):
        self._pid = self._pid % 65535 + 1
        return self._pid

    def _send(self, data):
        with self._send_lock:
            if not self._sock:
                raise MQTTError("not connected")
            self._sock.sendall(data)

    def connect(self):
        raw = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock = raw
        try:
            if self.tls:
                ctx = self.ssl_context or roomba_tls_context()
                sock = ctx.wrap_socket(raw, server_hostname=None)
            else:
                sock = raw
            flags = 0x02
            payload = _str(self.client_id)
            if self.username is not None:
                flags |= 0x80
                payload += _str(self.username)
            if self.password is not None:
                flags |= 0x40
                payload += _str(self.password)
            body = _str("MQTT") + bytes([4, flags]) + struct.pack(">H", self.keepalive) + payload
            sock.sendall(packet(CONNECT, 0, body))
            ptype, _, resp = read_packet(sock)
            if ptype != CONNACK or len(resp) < 2:
                raise MQTTError("unexpected reply to CONNECT")
            if resp[1]:
                raise MQTTError(f"connection refused: {CONNACK_ERRORS.get(resp[1], resp[1])}")
        except BaseException:
            sock.close()
            if sock is not raw:
                raw.close()
            raise
        sock.settimeout(None)
        self._sock = sock
        self._closing = False
        self._stop_ping = threading.Event()
        self.connected = True
        self._thread = threading.Thread(target=self._loop, name="mqtt-reader", daemon=True)
        self._thread.start()
        threading.Thread(target=self._pinger, name="mqtt-ping", daemon=True).start()

    def _pinger(self):
        while not self._stop_ping.wait(max(1.0, self.keepalive / 2)):
            try:
                self._send(packet(PINGREQ, 0, b""))
            except (OSError, MQTTError):
                return

    def subscribe(self, *topics):
        body = struct.pack(">H", self._next_pid())
        for t in topics:
            body += _str(t) + b"\x00"
        self._send(packet(SUBSCRIBE, 0x02, body))

    def publish(self, topic, payload, qos=0):
        if isinstance(payload, str):
            payload = payload.encode()
        body = _str(topic)
        if qos:
            body += struct.pack(">H", self._next_pid())
        self._send(packet(PUBLISH, qos << 1, body + payload))

    def close(self):
        self._closing = True
        if self._sock:
            self._stop_ping.set()
            try:
                self._send(packet(DISCONNECT, 0, b""))
            except (OSError, MQTTError):
                pass
            for fn in (lambda: self._sock.shutdown(socket.SHUT_RDWR), self._sock.close):
                try:
                    fn()
                except OSError:
                    pass
        self.connected = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3)

    def _loop(self):
        error = None
        try:
            while not self._closing:
                ptype, flags, body = read_packet(self._sock)
                if ptype == PUBLISH:
                    topic, payload, qos, pid = parse_publish(flags, body)
                    if qos == 1:
                        self._send(packet(PUBACK, 0, struct.pack(">H", pid)))
                    if self.on_message:
                        try:
                            self.on_message(topic, payload)
                        except Exception:  # never let a handler kill the link
                            pass
        except (OSError, ConnectionError, MQTTError) as exc:
            error = exc
        self.connected = False
        self._stop_ping.set()
        if not self._closing and self.on_disconnect:
            self.on_disconnect(error)


def topic_matches(pattern, topic):
    p, t = pattern.split("/"), topic.split("/")
    for k, part in enumerate(p):
        if part == "#":
            return True
        if k >= len(t) or (part != "+" and part != t[k]):
            return False
    return len(p) == len(t)


class MiniBroker:
    """Single-purpose MQTT server, used by the robot emulator and the tests.

    on_publish(topic, payload) is called for every message a client sends;
    send(topic, payload) pushes a message to subscribed clients.
    """

    def __init__(self, host="127.0.0.1", port=0, username=None, password=None,
                 ssl_context=None, max_clients=1):
        self.username, self.password = username, password
        self.ssl_context = ssl_context
        self.max_clients = max_clients
        self.on_publish = None
        self._clients = {}  # sock -> list of subscription patterns
        self._lock = threading.Lock()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((host, port))
        self._srv.listen(4)
        self.port = self._srv.getsockname()[1]
        self._stop = False
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while not self._stop:
            try:
                conn, _ = self._srv.accept()
            except OSError:
                return
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        try:
            if self.ssl_context:
                conn = self.ssl_context.wrap_socket(conn, server_side=True)
            ptype, _, body = read_packet(conn)
            if ptype != CONNECT:
                return
            rc = self._check_connect(body)
            with self._lock:
                if rc == 0 and len(self._clients) >= self.max_clients:
                    rc = 3  # Roombas accept a single local connection
                conn.sendall(packet(CONNACK, 0, bytes([0, rc])))
                if rc:
                    return
                self._clients[conn] = []
            while True:
                ptype, flags, body = read_packet(conn)
                if ptype == SUBSCRIBE:
                    pid = body[:2]
                    pos, pats = 2, []
                    while pos < len(body):
                        n = struct.unpack(">H", body[pos: pos + 2])[0]
                        pats.append(body[pos + 2: pos + 2 + n].decode())
                        pos += 3 + n
                    with self._lock:
                        self._clients[conn].extend(pats)
                    conn.sendall(packet(SUBACK, 0, pid + bytes(len(pats))))
                elif ptype == PUBLISH:
                    topic, payload, qos, pid = parse_publish(flags, body)
                    if qos == 1:
                        conn.sendall(packet(PUBACK, 0, struct.pack(">H", pid)))
                    if self.on_publish:
                        self.on_publish(topic, payload)
                elif ptype == PINGREQ:
                    conn.sendall(packet(PINGRESP, 0, b""))
                elif ptype == DISCONNECT:
                    return
        except (OSError, ConnectionError, MQTTError, ssl.SSLError):
            pass
        finally:
            with self._lock:
                self._clients.pop(conn, None)
            try:
                conn.close()
            except OSError:
                pass

    def _check_connect(self, body):
        try:
            pos = 2 + struct.unpack(">H", body[:2])[0]
            level, flags = body[pos], body[pos + 1]
            pos += 4
            fields = []
            while pos < len(body):
                n = struct.unpack(">H", body[pos: pos + 2])[0]
                fields.append(body[pos + 2: pos + 2 + n].decode())
                pos += 2 + n
        except (IndexError, struct.error, UnicodeDecodeError):
            return 2
        if level != 4:
            return 1
        user = fields[1] if flags & 0x80 and len(fields) > 1 else None
        pw = fields[2] if flags & 0x40 and len(fields) > 2 else None
        if self.username is not None and (user != self.username or pw != self.password):
            return 4
        return 0

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    def send(self, topic, payload):
        if isinstance(payload, str):
            payload = payload.encode()
        data = packet(PUBLISH, 0, _str(topic) + payload)
        with self._lock:
            targets = [c for c, pats in self._clients.items() if any(topic_matches(p, topic) for p in pats)]
        for conn in targets:
            try:
                conn.sendall(data)
            except OSError:
                pass

    def close(self):
        self._stop = True
        try:
            self._srv.close()
        except OSError:
            pass
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c.close()
            except OSError:
                pass
