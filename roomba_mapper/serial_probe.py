"""Listen to the serial line inside the robot with a USB-serial adapter.

Inside a Wi-Fi Roomba, the Wi-Fi module (an ESP32 on the Combo Essential)
passes commands and status to the main controller over a UART. Tapping that
line with a 3.3 V USB-serial adapter shows what the two chips say to each
other. That can include things the Wi-Fi side never reports, such as
positions, sensors or drive commands.

* ``capture()`` records one or both directions with timestamps, finds the
  baud rate, and lets you mark moments ("pressed Clean") while it records,
* ``analyze()`` works out the message format: start bytes, length field,
  checksum, message types, the bytes that change, and what appeared after
  each mark,
* ``send()`` writes a frame (filling in its length and checksum) and shows
  the replies.

No protocol is assumed: iRobot does not publish the one inside the robot.
Ports are opened with termios on Linux and macOS; elsewhere pyserial is
needed (``pip install pyserial``).
"""

import functools
import glob
import json
import operator
import os
import queue
import select
import sys
import threading
import time
import zlib
from bisect import bisect_right
from collections import Counter, defaultdict

BAUDS = (115200, 9600, 19200, 38400, 57600, 230400, 460800, 921600)
DEFAULT_GAP_MS = 20.0    # USB adapters deliver data in bursts up to ~16 ms apart
MARK_WINDOW = 3.0        # seconds after a mark in which new messages count as its effect
MAX_MESSAGE = 4096
MAX_FRAME = 1024
SAMPLE = 400             # frames used for format detection


class SerialError(Exception):
    pass


# -- the port ----------------------------------------------------------------------

class MarkDecoder:
    """Undo termios PARMRK marking, keeping state across reads.

    FF FF is a real FF byte; FF 00 x is a byte that arrived with a framing
    (or parity) error, which is dropped and counted; FF 00 00 is a break.
    """

    def __init__(self):
        self._pending = b""

    def feed(self, raw):
        data = self._pending + raw
        out, errors, i, n = bytearray(), 0, 0, len(data)
        while i < n:
            b = data[i]
            if b != 0xFF:
                out.append(b)
                i += 1
            elif i + 1 >= n:
                break
            elif data[i + 1] == 0xFF:
                out.append(0xFF)
                i += 2
            elif data[i + 1] == 0x00:
                if i + 2 >= n:
                    break
                errors += 1
                i += 3
            else:
                out.append(0xFF)
                i += 1
        self._pending = data[i:]
        return bytes(out), errors


class SerialPort:
    """A raw 8N1 serial port.

    With termios, reads also count framing errors (bytes received at the
    wrong speed), which is how the baud rate is found. With pyserial they
    can't be counted and ``read`` reports None errors.
    """

    def __init__(self, path, baud=115200):
        self.path = path
        self.baud = None
        self._fd = self._ser = None
        self._decoder = MarkDecoder()
        try:
            import termios  # noqa: F401
            posix = True
        except ImportError:
            posix = False
        if posix:
            try:
                self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            except OSError as exc:
                raise SerialError(f"can't open {path}: {exc.strerror}") from exc
        else:
            try:
                import serial
            except ImportError as exc:
                raise SerialError("on this system serial ports need pyserial: pip install pyserial") from exc
            try:
                self._ser = serial.Serial(path, baud, timeout=0.05)
            except (serial.SerialException, ValueError) as exc:
                raise SerialError(f"can't open {path}: {exc}") from exc
        try:
            self.set_baud(baud)
        except SerialError:
            self.close()
            raise

    @property
    def reports_errors(self):
        return self._fd is not None

    def set_baud(self, baud):
        if self._fd is not None:
            import termios
            speed = getattr(termios, f"B{baud}", None)
            if speed is None:
                raise SerialError(f"{baud} baud isn't available on this system")
            try:
                attrs = termios.tcgetattr(self._fd)
                attrs[0] = termios.INPCK | termios.PARMRK   # mark bytes with framing errors
                attrs[1] = 0
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attrs[3] = 0
                attrs[4] = attrs[5] = speed
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(self._fd, termios.TCSANOW, attrs)
                termios.tcflush(self._fd, termios.TCIFLUSH)
            except termios.error as exc:
                raise SerialError(f"{self.path} is not a serial port ({exc})") from exc
            self._decoder = MarkDecoder()
        else:
            try:
                self._ser.baudrate = baud
                self._ser.reset_input_buffer()
            except Exception as exc:  # pyserial raises several types
                raise SerialError(f"{baud} baud: {exc}") from exc
        self.baud = baud

    def read(self, timeout):
        """Wait up to timeout seconds for data. Returns (bytes, framing errors or None)."""
        if self._fd is not None:
            ready, _, _ = select.select([self._fd], [], [], timeout)
            if not ready:
                return b"", 0
            try:
                raw = os.read(self._fd, 4096)
            except BlockingIOError:
                return b"", 0
            except OSError as exc:
                raise SerialError(f"{self.path} stopped working ({exc.strerror}); was it unplugged?") from exc
            if not raw:
                raise SerialError(f"{self.path} was disconnected")
            return self._decoder.feed(raw)
        try:
            self._ser.timeout = timeout
            data = self._ser.read(1)
            if data and self._ser.in_waiting:
                data += self._ser.read(self._ser.in_waiting)
        except Exception as exc:
            raise SerialError(f"{self.path} stopped working ({exc})") from exc
        return data, None

    def write(self, data):
        if self._fd is None:
            self._ser.write(data)
            self._ser.flush()
            return
        import termios
        view = memoryview(data)
        while view:
            select.select([], [self._fd], [], 1.0)
            try:
                n = os.write(self._fd, view)
            except BlockingIOError:
                continue
            view = view[n:]
        try:
            termios.tcdrain(self._fd)
        except termios.error:
            pass

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        if self._ser is not None:
            self._ser.close()
            self._ser = None


def list_ports():
    """Serial devices that look like USB adapters."""
    found = []
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.usbserial*", "/dev/cu.SLAB*",
                    "/dev/cu.wchusbserial*", "/dev/cu.usbmodem*"):
        found += sorted(glob.glob(pattern))
    if not found and os.name == "nt":
        try:
            from serial.tools import list_ports as lp
            found = [p.device for p in lp.comports()]
        except ImportError:
            pass
    return found


# -- finding the baud rate --------------------------------------------------------------

def _structure(data):
    """Compressed size over raw size: lower means more repetitive, i.e. more structured."""
    return len(zlib.compress(bytes(data), 9)) / max(1, len(data))


def pick_baud(samples):
    """Choose from [{"baud", "data", "errors"}] (errors None when unknown).

    A UART read too fast produces framing errors; read too slowly, isolated
    bytes can still arrive without errors (but wrong). So the right speed is
    the fastest one with almost no errors.
    """
    heard = [s for s in samples if len(s["data"]) >= 8]
    if not heard:
        return None
    if all(s["errors"] is None for s in heard):
        # no error counts (pyserial): prefer text, then the most repetitive data
        return min(heard, key=lambda s: (_printable_share(s["data"]) < 0.9, _structure(s["data"])))["baud"]

    def rate(s):
        return (s["errors"] or 0) / (len(s["data"]) + (s["errors"] or 0))
    clean = [s for s in heard if rate(s) <= 0.02]
    if clean:
        return max(clean, key=lambda s: s["baud"])["baud"]
    return min(heard, key=rate)["baud"]


def detect_baud(port, bauds=BAUDS, dwell=1.5, log=print):
    samples = []
    log(f"   {'baud':>7}  {'bytes':>6}  {'errors':>6}")
    for baud in bauds:
        try:
            port.set_baud(baud)
        except SerialError as exc:
            log(f"   {baud:>7}  skipped ({exc})")
            continue
        data, errors = bytearray(), (0 if port.reports_errors else None)
        end = time.time() + dwell
        while True:
            left = end - time.time()
            if left <= 0:
                break
            chunk, e = port.read(min(left, 0.1))
            data += chunk
            if errors is not None:
                errors += e or 0
        samples.append({"baud": baud, "data": bytes(data), "errors": errors})
        log(f"   {baud:>7}  {len(data):>6}  {'?' if errors is None else errors:>6}")
    return pick_baud(samples), samples


# -- recording --------------------------------------------------------------------

class Assembler:
    """Cuts one direction's byte stream into messages at gaps of silence."""

    def __init__(self, gap):
        self.gap = gap
        self.buf = bytearray()
        self.start = self.last = 0.0
        self.errors = 0

    def add(self, t, data, errors=0):
        done = []
        if self.buf and (t - self.last > self.gap or len(self.buf) >= MAX_MESSAGE):
            done.append(self.flush())
        if data:
            if not self.buf:
                self.start = t
            self.buf += data
            self.last = t
        self.errors += errors or 0
        return done

    def poll(self, now):
        if self.buf and now - self.last > self.gap:
            return [self.flush()]
        return []

    def flush(self):
        msg = (self.start, bytes(self.buf), self.errors)
        self.buf = bytearray()
        self.errors = 0
        return msg


def _printable_share(data):
    if not data:
        return 0.0
    return sum(1 for b in data if 32 <= b < 127 or b in (9, 10, 13)) / len(data)


def show(data, limit=40):
    if len(data) > 3 and _printable_share(data) >= 0.9:
        text = data.decode("latin-1").rstrip("\r\n")
        return repr(text if len(text) <= limit * 2 else text[:limit * 2] + "...")
    head = " ".join(f"{b:02X}" for b in data[:limit])
    return head + (f" ... ({len(data)} bytes)" if len(data) > limit else "")


def capture(ports, baud="auto", seconds=120.0, out_path="roomba_serial.jsonl",
            gap_ms=DEFAULT_GAP_MS, live=True, marks_from_stdin=True, log=print):
    """Record the line(s) into out_path, then analyze the recording.

    ports is one device per direction (one or two). Returns the analysis.
    """
    names = "AB"
    opened = []
    try:
        for path in ports:
            opened.append(SerialPort(path, 115200 if baud == "auto" else int(baud)))
        if baud == "auto":
            log(f"1. Finding the baud rate on {ports[0]} (the line must be busy: power the robot "
                "on, or press a button, while this runs)")
            chosen = None
            for i, port in enumerate(opened):
                if i:
                    log(f"   ...and on {ports[i]}")
                chosen, _ = detect_baud(port, log=log)
                if chosen:
                    break
            if not chosen:
                log("   Nothing arrived at any speed. Check that GND is connected, that the adapter's RX "
                    "is on a TX line, and that the robot is on. Some lines only talk when something "
                    "happens: try again while starting a clean. Or give --baud.")
                return None
            log(f"   => {chosen} baud")
            baud = chosen
        baud = int(baud)
        for port in opened:
            port.set_baud(baud)

        log(f"2. Recording {'until Ctrl+C' if not seconds else f'for {int(seconds)} s'} to {os.path.abspath(out_path)}")
        for i, path in enumerate(ports):
            log(f"   line {names[i]} = {path}")
        if marks_from_stdin and sys.stdin and sys.stdin.isatty():
            log("   Type a note and press Enter whenever you do something (e.g. 'clean', 'dock', "
                "'bumped it') so it can be matched to the messages that follow.")
        events = queue.Queue()
        stop = threading.Event()

        def reader(i, port):
            while not stop.is_set():
                try:
                    data, errors = port.read(0.05)
                except SerialError as exc:
                    events.put(("error", time.time(), str(exc), 0))
                    return
                if data or errors:
                    events.put((names[i], time.time(), data, errors))

        def marker():
            n = 0
            while not stop.is_set():
                line = sys.stdin.readline()
                if not line:
                    return
                n += 1
                events.put(("mark", time.time(), line.strip() or f"mark {n}", 0))

        threads = [threading.Thread(target=reader, args=(i, p), daemon=True) for i, p in enumerate(opened)]
        if marks_from_stdin and sys.stdin and sys.stdin.isatty():
            threads.append(threading.Thread(target=marker, daemon=True))
        t0 = time.time()
        gap = gap_ms / 1000.0
        assemblers = {names[i]: Assembler(gap) for i in range(len(opened))}
        count = 0
        with open(out_path, "w", encoding="utf-8") as out:
            out.write(json.dumps({"type": "start", "t": round(t0, 4), "baud": baud, "gap_ms": gap_ms,
                                  "ports": {names[i]: p for i, p in enumerate(ports)}}) + "\n")

            def emit(direction, msg):
                t, data, errors = msg
                rec = {"t": round(t, 4), "dir": direction, "hex": data.hex()}
                if errors:
                    rec["err"] = errors
                out.write(json.dumps(rec) + "\n")
                if live:
                    log(f"{t - t0:9.3f}  {direction}  {show(data)}" + (f"  [{errors} errors]" if errors else ""))

            for th in threads:
                th.start()
            try:
                while not seconds or time.time() - t0 < seconds:
                    try:
                        kind, t, data, errors = events.get(timeout=0.02)
                    except queue.Empty:
                        kind = None
                    if kind == "error":
                        log(f"   stopped: {data}")
                        break
                    if kind == "mark":
                        out.write(json.dumps({"t": round(t, 4), "mark": data}) + "\n")
                        log(f"{t - t0:9.3f}  ----- {data} -----")
                    elif kind:
                        for msg in assemblers[kind].add(t, data, errors):
                            emit(kind, msg)
                            count += 1
                    now = time.time()
                    for direction, asm in assemblers.items():
                        for msg in asm.poll(now):
                            emit(direction, msg)
                            count += 1
                    out.flush()
            except KeyboardInterrupt:
                pass
            stop.set()
            for direction, asm in assemblers.items():
                if asm.buf:
                    emit(direction, asm.flush())
                    count += 1
        log(f"   {count} messages saved")
    finally:
        for port in opened:
            port.close()
    log("3. What the recording shows")
    return analyze(load_capture(out_path), log=log)


def load_capture(path):
    rec = {"ports": {}, "baud": None, "messages": [], "marks": []}
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            item = json.loads(line)
            if item.get("type") == "start":
                rec["ports"] = item.get("ports", {})
                rec["baud"] = item.get("baud")
            elif "mark" in item:
                rec["marks"].append((item["t"], item["mark"]))
            elif "hex" in item:
                rec["messages"].append((item["t"], item["dir"], bytes.fromhex(item["hex"]),
                                        item.get("err", 0)))
    return rec


# -- frame formats ------------------------------------------------------------------

def _crc16(data, poly, init, reflected):
    crc = init
    for b in data:
        if reflected:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ poly if crc & 1 else crc >> 1
        else:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ poly) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


CHECKSUMS = {
    "sum8": lambda b: bytes([sum(b) & 0xFF]),
    "neg8": lambda b: bytes([-sum(b) & 0xFF]),
    "not8": lambda b: bytes([~sum(b) & 0xFF]),
    "xor8": lambda b: bytes([functools.reduce(operator.xor, b, 0)]),
    "sum16le": lambda b: (sum(b) & 0xFFFF).to_bytes(2, "little"),
    "sum16be": lambda b: (sum(b) & 0xFFFF).to_bytes(2, "big"),
    "crc16modbus": lambda b: _crc16(b, 0xA001, 0xFFFF, True).to_bytes(2, "little"),
    "crc16ccitt": lambda b: _crc16(b, 0x1021, 0xFFFF, False).to_bytes(2, "big"),
    "crc16xmodem": lambda b: _crc16(b, 0x1021, 0x0000, False).to_bytes(2, "big"),
}
CHECKSUM_TEXT = {
    "sum8": "8-bit sum", "neg8": "8-bit sum, negated", "not8": "8-bit sum, inverted",
    "xor8": "XOR", "sum16le": "16-bit sum, low byte first", "sum16be": "16-bit sum, high byte first",
    "crc16modbus": "CRC-16/MODBUS", "crc16ccitt": "CRC-16/CCITT-FALSE", "crc16xmodem": "CRC-16/XMODEM",
}
LENGTH_KINDS = {"u8": 1, "u16le": 2, "u16be": 2}


def _read_len(frame, off, kind):
    if kind == "u8":
        return frame[off]
    return int.from_bytes(frame[off:off + 2], "little" if kind == "u16le" else "big")


def _chk_size(fmt):
    return len(CHECKSUMS[fmt["checksum"][0]](b"")) if fmt.get("checksum") else 0


def frame_ok(frame, fmt):
    """Does the frame's checksum match? (True when the format has none.)"""
    if not fmt.get("checksum"):
        return True
    name, start = fmt["checksum"]
    size = len(CHECKSUMS[name](b""))
    end = len(frame) - len(fmt.get("trailer") or b"")
    if end - size < start:
        return False
    return CHECKSUMS[name](frame[start:end - size]) == frame[end - size:end]


def find_length(frames, hlen):
    """Find a field holding the frame length. Returns (offset, kind, delta, share) or None,
    where frame length = field value + delta."""
    if len({len(f) for f in frames}) < 2:
        return None
    found = []
    for off in range(hlen, hlen + 4):
        for kind, size in LENGTH_KINDS.items():
            deltas = Counter(len(f) - _read_len(f, off, kind) for f in frames if len(f) >= off + size)
            if not deltas:
                continue
            delta, hits = deltas.most_common(1)[0]
            fitting = {len(f) for f in frames if len(f) >= off + size and len(f) - _read_len(f, off, kind) == delta}
            if -2 <= delta <= 64 and len(fitting) >= 2:
                found.append((hits / len(frames), off, kind, delta))
    if not found:
        return None
    top = max(f[0] for f in found)
    if top < 0.6:
        return None
    # earliest offset among the best; a 2-byte field before a 1-byte one at the same place
    share, off, kind, delta = min((f for f in found if f[0] >= top - 0.02),
                                  key=lambda f: (f[1], -LENGTH_KINDS[f[2]]))
    return off, kind, delta, share


def find_checksum(frames, trailer=b""):
    """Find the checksum at the end of the frames. Returns (name, start, share) or None."""
    best = None
    for name, fn in CHECKSUMS.items():
        size = len(fn(b""))
        for start in range(5):
            hits = 0
            for f in frames:
                end = len(f) - len(trailer)
                if end - size > start and fn(f[start:end - size]) == f[end - size:end]:
                    hits += 1
            share = hits / max(1, len(frames))
            if hits >= 4 and share >= 0.8 and (best is None or share > best[2] + 1e-9):
                best = (name, start, share)
    return best


def _distinct(frames):
    return list(dict.fromkeys(frames))[:SAMPLE]


def split_at(messages, header):
    """Pieces of each message starting at each occurrence of header."""
    pieces = []
    for m in messages:
        i = m.find(header)
        while i != -1:
            j = m.find(header, i + 1)
            pieces.append(m[i:j if j != -1 else len(m)])
            i = j
    return pieces


def parse_spans(stream, fmt):
    """(start, end) of each frame in a byte stream."""
    h = fmt["header"]
    spans, n = [], len(stream)
    length = fmt.get("length")
    if length:
        off, kind, delta = length
        need = off + LENGTH_KINDS[kind]
        minimum = need + _chk_size(fmt) + len(fmt.get("trailer") or b"")
        i = stream.find(h)
        while i != -1 and i + need <= n:
            total = _read_len(stream, i + off, kind) + delta
            if total < minimum or total > MAX_FRAME:
                i = stream.find(h, i + 1)
                continue
            if i + total > n:
                break
            if not frame_ok(stream[i:i + total], fmt):
                i = stream.find(h, i + 1)
                continue
            spans.append((i, i + total))
            i = stream.find(h, i + total)
        return spans
    starts = []
    i = stream.find(h)
    while i != -1:
        starts.append(i)
        i = stream.find(h, i + 1)
    k = 0
    while k < len(starts):
        end = starts[k + 1] if k + 1 < len(starts) else n
        if fmt.get("checksum") and not frame_ok(stream[starts[k]:end], fmt):
            # the header may also occur inside the data: try taking in the next pieces
            for j in range(k + 2, min(k + 7, len(starts) + 1)):
                cand = starts[j] if j < len(starts) else n
                if frame_ok(stream[starts[k]:cand], fmt):
                    spans.append((starts[k], cand))
                    k = j
                    break
            else:
                k += 1   # not a frame after all
            continue
        spans.append((starts[k], end))
        k += 1
    return spans


def _with_format(messages, header):
    """Try one header: returns (fmt, frames, fit) or None."""
    frames = split_at(messages, header)
    if len(frames) < 4:
        return None
    fmt = {"header": header, "length": None, "checksum": None, "trailer": None}
    stream = b"".join(messages)
    sample = _distinct(frames)   # repeats of one frame would "confirm" any guess
    fit = 0.0
    length = find_length(sample, len(header))
    if length:
        fmt["length"] = length[:3]
        fit = length[3]
        frames = [stream[a:b] for a, b in parse_spans(stream, fmt)]
        sample = _distinct(frames)
    if len(sample) >= 4:
        last = Counter(f[-1] for f in sample).most_common(1)[0]
        trailers = [b"", bytes([last[0]])] if last[1] / len(sample) >= 0.9 else [b""]
        for trailer in trailers:
            chk = find_checksum(sample, trailer)
            if chk:
                fmt["checksum"] = chk[:2]
                fmt["trailer"] = trailer or None
                fit = max(fit, chk[2])
                break
        else:
            if len(trailers) == 2:
                fmt["trailer"] = trailers[1]
    if fmt["checksum"]:
        frames = [stream[a:b] for a, b in parse_spans(stream, fmt)]
    return fmt, frames, fit


def detect_format(messages):
    """Work out the framing of one direction's messages (gap-split byte strings).

    Returns (fmt, frames) or (None, []). fmt has header, length (offset, kind,
    delta), checksum (name, start) and trailer; any of the last three may be None.
    """
    stream = b"".join(messages)
    if len(stream) < 16:
        return None, []
    starts2 = Counter(m[:2] for m in messages if len(m) >= 2)
    starts1 = Counter(m[:1] for m in messages if m)
    sample = stream[:200000]
    bigrams = Counter(sample[i:i + 2] for i in range(len(sample) - 1))
    candidates = {h for h, _ in starts2.most_common(3)} | {h for h, _ in starts1.most_common(2)} | \
                 {h for h, _ in bigrams.most_common(5)}
    best = None
    for header in candidates:
        tried = _with_format(messages, header)
        if not tried:
            continue
        fmt, frames, fit = tried
        if not fit:
            share = sum(1 for m in messages if m.startswith(header)) / len(messages)
            fit = share * 0.5 if share >= 0.5 and len(messages) >= 4 else 0.0
        if not fit:
            continue
        coverage = sum(len(f) for f in frames) / len(stream)
        starting = sum(1 for m in messages if m.startswith(header)) / len(messages)
        score = 0.5 * fit + 0.3 * coverage + 0.2 * starting + 0.01 * len(header)
        if best is None or score > best[0]:
            best = (score, fmt, frames)
    if not best:
        return None, []
    return best[1], best[2]


def format_spec(fmt):
    """The --frame-format text for serial-send, e.g. 'len@4:u16be+7,chk=sum8@0'."""
    parts = []
    if fmt.get("length"):
        off, kind, delta = fmt["length"]
        parts.append(f"len@{off}:{kind}{delta:+d}")
    if fmt.get("checksum"):
        name, start = fmt["checksum"]
        parts.append(f"chk={name}@{start}")
    if fmt.get("trailer"):
        parts.append(f"end={fmt['trailer'].hex().upper()}")
    return ",".join(parts)


def parse_spec(text):
    fmt = {"length": None, "checksum": None, "trailer": None}
    for part in filter(None, (p.strip() for p in (text or "").split(","))):
        try:
            if part.startswith("len@"):
                off, rest = part[4:].split(":")
                kind = next(k for k in sorted(LENGTH_KINDS, key=len, reverse=True) if rest.startswith(k))
                fmt["length"] = (int(off), kind, int(rest[len(kind):] or 0))
            elif part.startswith("chk="):
                name, start = part[4:].split("@")
                if name not in CHECKSUMS:
                    raise ValueError
                fmt["checksum"] = (name, int(start))
            elif part.startswith("end="):
                fmt["trailer"] = bytes.fromhex(part[4:])
            else:
                raise ValueError
        except (ValueError, StopIteration):
            raise ValueError(f"can't read {part!r} in the frame format "
                             "(expected e.g. len@4:u16be+7,chk=sum8@0,end=0D)") from None
    return fmt


def build_frame(body, fmt):
    """body is the frame without its checksum and end byte; the length field (if any)
    is filled in, then the checksum and end byte appended."""
    body = bytearray(body)
    trailer = fmt.get("trailer") or b""
    total = len(body) + _chk_size(fmt) + len(trailer)
    if fmt.get("length"):
        off, kind, delta = fmt["length"]
        size = LENGTH_KINDS[kind]
        if len(body) < off + size:
            raise ValueError(f"the frame is too short to hold its length field (byte {off})")
        value = total - delta
        if value < 0 or value >= 256 ** size:
            raise ValueError("the frame's length doesn't fit its length field")
        body[off:off + size] = value.to_bytes(size, "big" if kind == "u16be" else "little")
    if fmt.get("checksum"):
        name, start = fmt["checksum"]
        body += CHECKSUMS[name](bytes(body[start:]))
    return bytes(body + trailer)


# -- analysis --------------------------------------------------------------------

def _fixed_positions(fmt):
    skip = set(range(len(fmt["header"])))
    if fmt.get("length"):
        off, kind, _ = fmt["length"]
        skip |= set(range(off, off + LENGTH_KINDS[kind]))
    return skip


def type_position(frames, fmt):
    """The first byte after the header that takes a handful of values: the message type."""
    if not frames:
        return None
    tail = _chk_size(fmt) + len(fmt.get("trailer") or b"")
    shortest = min(len(f) for f in frames) - tail
    skip = _fixed_positions(fmt)
    for p in range(len(fmt["header"]), shortest):
        if p in skip:
            continue
        values = Counter(f[p] for f in frames)
        if 2 <= len(values) <= 40:
            return p
        if len(values) > 40:
            return None
    return None


def is_tuya(fmt):
    return (fmt and fmt["header"] == b"\x55\xAA" and fmt.get("length") == (4, "u16be", 7)
            and fmt.get("checksum") == ("sum8", 0))


TUYA_COMMANDS = {0x00: "heartbeat", 0x01: "product info", 0x02: "work mode", 0x03: "Wi-Fi status",
                 0x04: "reset Wi-Fi", 0x06: "command (datapoints)", 0x07: "status report (datapoints)",
                 0x08: "query status", 0x1C: "time"}
TUYA_TYPES = {0: "raw", 1: "bool", 2: "value", 3: "string", 4: "enum", 5: "bitmap"}


def tuya_datapoints(frames_by_dir):
    """{(direction, dp id): {"type", "count", "values"}} from Tuya 0x06/0x07 frames."""
    found = {}
    for direction, frames in frames_by_dir.items():
        for f in frames:
            if len(f) < 7 or f[3] not in (0x06, 0x07):
                continue
            data, i = f[6:-1], 0
            while i + 4 <= len(data):
                dp, kind, size = data[i], data[i + 1], int.from_bytes(data[i + 2:i + 4], "big")
                raw = data[i + 4:i + 4 + size]
                i += 4 + size
                if kind in (1, 2, 4):
                    value = int.from_bytes(raw, "big", signed=(kind == 2))
                elif kind == 3:
                    value = raw.decode("utf-8", "replace")
                else:
                    value = raw.hex()
                entry = found.setdefault((direction, dp), {"type": TUYA_TYPES.get(kind, kind),
                                                           "count": 0, "values": []})
                entry["count"] += 1
                if value not in entry["values"] and len(entry["values"]) < 6:
                    entry["values"].append(value)
    return found


def _at(marks, t):
    """The mark whose window contains time t, or None."""
    for tm, label in marks:
        if 0 <= t - tm <= MARK_WINDOW:
            return tm, label
    return None


def analyze(rec, log=print):
    """Explain a recording (from load_capture). Prints findings and returns them."""
    ports, marks = rec["ports"], sorted(rec["marks"])
    report = {"baud": rec.get("baud"), "lines": {}}
    units = []                # (t, direction, bytes, kind) for matching against marks
    frames_by_dir = {}
    for direction in sorted({m[1] for m in rec["messages"]}):
        msgs = [(t, data) for t, d, data, _ in rec["messages"] if d == direction and data]
        errors = sum(e for _, d, _, e in rec["messages"] if d == direction)
        raw = [data for _, data in msgs]
        total = sum(len(d) for d in raw)
        name = f"line {direction}" + (f" ({ports[direction]})" if direction in ports else "")
        line = {"bytes": total, "messages": len(raw), "errors": errors}
        report["lines"][direction] = line
        log(f"   {name}: {total} bytes in {len(raw)} messages" + (f", {errors} framing errors" if errors else ""))
        if errors > max(3, total // 50):
            log("     Many framing errors: the baud rate is probably wrong, or GND isn't connected.")
        if not raw:
            continue
        duration = max(1e-6, msgs[-1][0] - msgs[0][0])
        if _printable_share(b"".join(raw)) >= 0.9:
            line["text"] = True
            lines = []
            for t, data in msgs:
                for text in data.decode("latin-1").replace("\r", "\n").split("\n"):
                    if text.strip():
                        lines.append((t, text.strip()))
                        data = text.strip().encode("latin-1")
                        units.append((t, direction, data, (direction, data)))
            common = Counter(text for _, text in lines).most_common(8)
            line["common_lines"] = common
            log("     It's text: a log console, or a text protocol (AT-style commands).")
            if any("rst:" in text or "boot:" in text or "ets " in text for _, text in lines):
                log("     It includes ESP32 boot messages: this is the ESP32's own console (its UART0).")
            log("     most frequent lines:")
            for text, n in common:
                log(f"       {n:5}x  {text[:100]}")
            continue
        fmt, frames = detect_format(raw)
        if not fmt:
            log("     No repeating frame structure found. Messages (split at pauses) start with: " +
                ", ".join(f"{h.hex(' ').upper()} ({n})" for h, n in Counter(m[:2] for m in raw).most_common(4)))
            units += [(t, direction, data, (direction, data)) for t, data in msgs]
            continue
        # timestamps for frames: the message each one starts in
        offsets, stream = [], bytearray()
        for t, data in msgs:
            offsets.append(len(stream))
            stream += data
        spans = parse_spans(bytes(stream), fmt)
        timed = [(msgs[bisect_right(offsets, a) - 1][0], bytes(stream[a:b])) for a, b in spans]
        frames = [f for _, f in timed]
        frames_by_dir[direction] = frames
        covered = sum(len(f) for f in frames) / max(1, total)
        spec = format_spec(fmt)
        line.update(format=fmt, spec=spec, frames=len(frames), covered=round(covered, 3))
        log(f"     Frames start with {fmt['header'].hex(' ').upper()}; {len(frames)} frames cover "
            f"{covered:.0%} of the bytes.")
        if fmt["length"]:
            off, kind, delta = fmt["length"]
            size = {"u8": "1 byte", "u16le": "2 bytes, low byte first", "u16be": "2 bytes, high byte first"}[kind]
            log(f"     Length: byte {off} ({size}); frame length = value {delta:+d}.")
        elif len({len(f) for f in frames}) == 1:
            log(f"     Every frame is {len(frames[0])} bytes long.")
        else:
            log("     No length field found (frames are told apart by their start bytes).")
        if fmt["checksum"]:
            cname, start = fmt["checksum"]
            log(f"     Checksum: {CHECKSUM_TEXT[cname]} of bytes {start} onwards, at the end"
                + (f" (before the end byte {fmt['trailer'].hex().upper()})." if fmt["trailer"] else "."))
        else:
            log("     No checksum recognised" + (f"; frames end with {fmt['trailer'].hex().upper()}."
                                                   if fmt["trailer"] else "."))
        if is_tuya(fmt):
            log("     This matches the Tuya MCU serial protocol (55 AA, version, command, length, "
                "data, 8-bit sum).")
        if spec:
            log(f"     For serial-send:  --frame-format {spec}")
        tpos = type_position(frames, fmt)
        line["type_byte"] = tpos
        # a frame is "new" after a note if no frame of its type and length came at other times
        units += [(t, direction, f, (direction, f[tpos] if tpos is not None else None, len(f)))
                  for t, f in timed]
        groups = defaultdict(list)
        for t, f in timed:
            groups[f[tpos] if tpos is not None else len(f)].append((t, f))
        tail = _chk_size(fmt) + len(fmt.get("trailer") or b"")
        skip = _fixed_positions(fmt)
        types = []
        for key, items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            fs = [f for _, f in items]
            shortest = min(len(f) for f in fs)
            usual = Counter(len(f) for f in fs).most_common(1)[0][0]
            same = [f for f in fs if len(f) == usual]
            varying = [p for p in range(usual - tail) if p not in skip and len({f[p] for f in same}) > 1]
            near = sum(1 for t, _ in items if _at(marks, t))
            types.append({"key": key, "count": len(fs), "per_s": len(fs) / duration,
                          "lengths": (shortest, max(len(f) for f in fs)), "varying": varying,
                          "only_near_marks": bool(marks) and near == len(fs), "example": fs[-1]})
        line["types"] = types
        what = f"byte {tpos}" if tpos is not None else "length"
        log(f"     Message types (by {what}):")
        for ty in types[:16]:
            lo, hi = ty["lengths"]
            label = f"{ty['key']:02X}" if tpos is not None else f"{ty['key']}B"
            if is_tuya(fmt) and ty["key"] in TUYA_COMMANDS:
                label += f" {TUYA_COMMANDS[ty['key']]}"
            notes = []
            if ty["varying"]:
                notes.append("changing bytes " + ",".join(map(str, ty["varying"][:10])) +
                             ("..." if len(ty["varying"]) > 10 else ""))
            if ty["only_near_marks"]:
                notes.append("only after your notes")
            log(f"       {label:<28} {ty['count']:5}x  {ty['per_s']:6.2f}/s  "
                f"{lo if lo == hi else f'{lo}-{hi}'} bytes  {'; '.join(notes)}")
            log(f"         e.g. {show(ty['example'], 32)}")
        if len(types) > 16:
            log(f"       ... and {len(types) - 16} more")
        moving = [ty for ty in types if ty["per_s"] >= 0.5 and len(ty["varying"]) >= 2]
        if moving:
            log("     Sent regularly with changing bytes (look here for positions, odometry, "
                "sensors): " + ", ".join(f"{ty['key']:02X}" if tpos is not None else f"{ty['key']}B"
                                         for ty in moving[:6]))

    if any(is_tuya(line.get("format")) for line in report["lines"].values()):
        dps = tuya_datapoints(frames_by_dir)
        report["tuya_datapoints"] = dps
        if dps:
            log("   Tuya datapoints (6 = sent to the robot's controller, 7 = reported by it):")
            for (direction, dp), info in sorted(dps.items(), key=lambda kv: (kv[0][1], kv[0][0])):
                log(f"     line {direction}  dp {dp:3}  {info['type']:<6} {info['count']:5}x  "
                    f"values {info['values']}")

    if marks:
        log(f"   After your notes (first {int(MARK_WINDOW)} s):")
        seen_elsewhere = Counter()
        by_mark = defaultdict(list)
        for t, direction, data, kind in units:
            hit = _at(marks, t)
            if hit:
                by_mark[hit].append((t, direction, data, kind))
            else:
                seen_elsewhere[kind] += 1
        report["marks"] = {}
        for tm, label in marks:
            new, listed = [], set()
            for t, direction, data, kind in by_mark.get((tm, label), []):
                if not seen_elsewhere[kind] and (direction, data) not in listed:
                    listed.add((direction, data))
                    new.append((t - tm, direction, data))
            new.sort(key=lambda item: item[0])
            report["marks"][label] = new
            log(f"     '{label}': " + ("nothing new" if not new else f"{len(new)} messages seen only then"))
            for dt, direction, data in new[:8]:
                log(f"       +{dt:4.1f}s  {direction}  {show(data, 32)}")
    if not marks and report["lines"]:
        log("   Tip: record again and type a note each time you press a button in the app; the "
            "messages that follow are listed per note, ready to replay with serial-send.")
    return report


# -- sending -----------------------------------------------------------------------

def parse_hex(text):
    cleaned = "".join(text).replace(":", "").replace(",", "").replace("0x", "").replace(" ", "")
    try:
        return bytes.fromhex(cleaned)
    except ValueError:
        raise ValueError(f"not hex bytes: {''.join(text)!r}") from None


def send(path, baud, body, frame_format=None, repeat=1, interval=1.0, listen=2.0,
         gap_ms=DEFAULT_GAP_MS, log=print):
    """Send body (completed with length/checksum by frame_format) and print what comes back."""
    frame = build_frame(body, parse_spec(frame_format)) if frame_format else bytes(body)
    port = SerialPort(path, baud)
    replies = []
    try:
        for k in range(max(1, repeat)):
            port.write(frame)
            sent_at = time.time()
            log(f"sent      {show(frame, 64)}")
            asm = Assembler(gap_ms / 1000.0)
            wait = listen if k == repeat - 1 else interval
            end = sent_at + wait
            while time.time() < end:
                data, errors = port.read(0.02)
                now = time.time()
                for msg in asm.add(now, data, errors) + asm.poll(now):
                    replies.append(msg[1])
                    log(f"+{msg[0] - sent_at:6.3f}s  {show(msg[1], 64)}")
            if asm.buf:
                msg = asm.flush()
                replies.append(msg[1])
                log(f"+{msg[0] - sent_at:6.3f}s  {show(msg[1], 64)}")
    finally:
        port.close()
    if not replies:
        log("No reply. (Replies only show if the adapter's RX is on the line coming back.)")
    return frame, replies
