import io
import json
import os
import random
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout

from roomba_mapper import serial_probe as sp
from roomba_mapper.__main__ import build_parser

try:
    import pty
    import termios  # noqa: F401
    HAVE_PTY = True
except ImportError:
    HAVE_PTY = False

TUYA = {"header": b"\x55\xAA", "length": (4, "u16be", 7), "checksum": ("sum8", 0), "trailer": None}


def tuya(cmd, data, ver=0x03):
    return sp.build_frame(bytes([0x55, 0xAA, ver, cmd, 0, 0]) + data, TUYA)


def quiet(fn, *args, **kw):
    out = io.StringIO()
    result = fn(*args, log=lambda *a: print(*a, file=out), **kw)
    return result, out.getvalue()


class ChecksumAndFrameTests(unittest.TestCase):
    def test_crc_check_values(self):
        data = b"123456789"
        self.assertEqual(sp.CHECKSUMS["crc16modbus"](data), (0x4B37).to_bytes(2, "little"))
        self.assertEqual(sp.CHECKSUMS["crc16ccitt"](data), (0x29B1).to_bytes(2, "big"))
        self.assertEqual(sp.CHECKSUMS["crc16xmodem"](data), (0x31C3).to_bytes(2, "big"))

    def test_build_frame_fills_length_and_checksum(self):
        frame = tuya(0x00, b"")
        self.assertEqual(frame, bytes.fromhex("55AA0300000002"))  # Tuya heartbeat
        fmt = sp.parse_spec("len@1:u8+3,chk=xor8@1,end=0D")
        frame = sp.build_frame(b"\xAA\x00\x10\x20\x30", fmt)
        self.assertEqual(frame[1], len(frame) - 3)
        self.assertEqual(frame[-1], 0x0D)
        self.assertTrue(sp.frame_ok(frame, fmt))

    def test_spec_round_trip(self):
        fmt = {"header": b"\xAA", "length": (2, "u16le", -1), "checksum": ("crc16modbus", 1),
               "trailer": b"\x0D"}
        spec = sp.format_spec(fmt)
        self.assertEqual(spec, "len@2:u16le-1,chk=crc16modbus@1,end=0D")
        back = sp.parse_spec(spec)
        for key in ("length", "checksum", "trailer"):
            self.assertEqual(back[key], fmt[key])
        with self.assertRaises(ValueError):
            sp.parse_spec("chk=md5@0")

    def test_parse_hex(self):
        self.assertEqual(sp.parse_hex(["55", "AA", "0x03", "00:01"]), bytes.fromhex("55AA030001"))
        with self.assertRaises(ValueError):
            sp.parse_hex(["5G"])


class LowLevelTests(unittest.TestCase):
    def test_mark_decoder_across_reads(self):
        dec = sp.MarkDecoder()
        # 01, a real FF, a byte with a framing error, 02, a break, 03 - split awkwardly
        stream = b"\x01\xFF\xFF\xFF\x00\x41\x02\xFF\x00\x00\x03"
        out, errors = bytearray(), 0
        for i in range(len(stream)):
            data, e = dec.feed(stream[i:i + 1])
            out += data
            errors += e
        self.assertEqual(bytes(out), b"\x01\xFF\x02\x03")
        self.assertEqual(errors, 2)

    def test_pick_baud_takes_fastest_error_free(self):
        samples = [
            {"baud": 9600, "data": b"\xF0" * 40, "errors": 0},       # too slow: wrong but clean
            {"baud": 115200, "data": bytes(range(40)), "errors": 0},
            {"baud": 230400, "data": bytes(range(60)), "errors": 25},
            {"baud": 460800, "data": b"", "errors": 0},
        ]
        self.assertEqual(sp.pick_baud(samples), 115200)
        self.assertIsNone(sp.pick_baud([{"baud": 9600, "data": b"", "errors": 0}]))
        noisy = [{"baud": 9600, "data": b"x" * 20, "errors": 9}, {"baud": 19200, "data": b"x" * 20, "errors": 3}]
        self.assertEqual(sp.pick_baud(noisy), 19200)

    def test_assembler_splits_at_gaps(self):
        asm = sp.Assembler(0.02)
        self.assertEqual(asm.add(0.000, b"\x01\x02"), [])
        self.assertEqual(asm.add(0.010, b"\x03"), [])
        done = asm.add(0.100, b"\x04")
        self.assertEqual(done, [(0.0, b"\x01\x02\x03", 0)])
        self.assertEqual(asm.poll(0.105), [])
        self.assertEqual(asm.poll(0.200), [(0.100, b"\x04", 0)])


def tuya_recording(seed=1):
    """Two directions of a Tuya-style line, some frames merged together as a USB adapter does."""
    rnd = random.Random(seed)
    messages, marks, t = [], [], 1000.0
    x = y = 0
    for k in range(60):
        t += 0.5
        x += rnd.randint(-30, 30)
        y += rnd.randint(-30, 30)
        status = tuya(0x07, bytes([0x0F, 2, 0, 4]) + x.to_bytes(4, "big", signed=True)
                      + bytes([0x10, 2, 0, 4]) + y.to_bytes(4, "big", signed=True))
        beat = tuya(0x00, b"\x01")
        if k % 3 == 0:
            messages.append((t, "A", status + beat, 0))      # merged in one USB burst
        else:
            messages.append((t, "A", status, 0))
            messages.append((t + 0.01, "A", beat, 0))
        messages.append((t + 0.02, "B", tuya(0x00, b"", ver=0x00), 0))
        if k % 4 == 0:
            messages.append((t + 0.03, "B", tuya(0x08, bytes([k % 7]) * (k % 3 + 1), ver=0x00), 0))
        if k == 30:
            marks.append((t + 0.1, "clean"))
            messages.append((t + 0.3, "B", tuya(0x06, bytes([0x01, 1, 0, 1, 1]), ver=0x00), 0))
            messages.append((t + 0.4, "A", tuya(0x07, bytes([0x01, 1, 0, 1, 1])), 0))
    return {"ports": {"A": "/dev/ttyUSB0", "B": "/dev/ttyUSB1"}, "baud": 115200,
            "messages": messages, "marks": marks}


class AnalysisTests(unittest.TestCase):
    def test_tuya_line_is_decoded(self):
        rec = tuya_recording()
        report, text = quiet(sp.analyze, rec)
        a = report["lines"]["A"]
        self.assertEqual(a["format"]["header"], b"\x55\xAA")
        self.assertEqual(a["format"]["length"], (4, "u16be", 7))
        self.assertEqual(a["format"]["checksum"], ("sum8", 0))
        self.assertEqual(a["spec"], "len@4:u16be+7,chk=sum8@0")
        self.assertEqual(a["frames"], 121)   # merged bursts were split into their frames
        self.assertIn("Tuya MCU", text)
        self.assertEqual(a["type_byte"], 3)
        status = next(ty for ty in a["types"] if ty["key"] == 0x07)
        self.assertGreaterEqual(status["count"], 60)
        self.assertTrue(set(status["varying"]) & set(range(10, 14)))   # x changes
        dps = report["tuya_datapoints"]
        self.assertEqual(dps[("A", 0x0F)]["type"], "value")
        self.assertEqual(dps[("B", 0x01)]["values"], [1])
        # the command sent after the note is listed for it
        new = report["marks"]["clean"]
        self.assertTrue(any(d == "B" and data[3] == 0x06 for _, d, data in new))
        cmd = next(data for _, d, data in new if d == "B")
        # and serial-send can rebuild it from the frame without its checksum
        self.assertEqual(sp.build_frame(cmd[:-1], sp.parse_spec(a["spec"])), cmd)

    def test_other_framing(self):
        fmt = {"header": b"\xA5", "length": (1, "u8", 4), "checksum": ("crc16modbus", 1), "trailer": None}
        rnd = random.Random(3)
        msgs = []
        for k in range(40):
            body = bytes([0xA5, 0, rnd.choice((0x10, 0x11, 0x20))]) + bytes(rnd.randrange(256) for _ in range(rnd.randint(1, 9)))
            msgs.append(sp.build_frame(body, fmt))
        found, frames = sp.detect_format(msgs)
        self.assertEqual(found["header"], b"\xA5")
        self.assertEqual(found["length"], (1, "u8", 4))
        self.assertEqual(found["checksum"], ("crc16modbus", 1))
        self.assertEqual(len(frames), 40)

    def test_trailer_and_no_length(self):
        fmt = {"header": b"\x7E", "length": None, "checksum": ("xor8", 1), "trailer": b"\x0D"}
        msgs = [sp.build_frame(bytes([0x7E, k % 4]) + bytes(range(k % 7 + 2)), fmt) for k in range(30)]
        found, frames = sp.detect_format(msgs)
        self.assertEqual(found["checksum"], ("xor8", 1))
        self.assertEqual(found["trailer"], b"\x0D")
        self.assertEqual(len(frames), 30)

    def test_text_console(self):
        rec = {"ports": {"A": "x"}, "baud": 115200, "marks": [], "messages": [
            (1.0, "A", b"ets Jun  8 2016 00:22:57\r\n\r\nrst:0x1 (POWERON_RESET),boot:0x13\r\n", 0),
            (2.0, "A", b"I (523) wifi: connected\r\n", 0),
            (3.0, "A", b"I (900) mqtt: ping\r\n", 0),
            (4.0, "A", b"I (900) mqtt: ping\r\n", 0)]}
        report, text = quiet(sp.analyze, rec)
        self.assertTrue(report["lines"]["A"]["text"])
        self.assertIn("ESP32 boot messages", text)
        self.assertIn("I (900) mqtt: ping", text)

    def test_esp32_message_log(self):
        log = (b"I (3504) common_APIs.c: CiREAL Request for Request_ID: 0x00\r\n"
               b"I (3504) common_APIs.c: TxMessage : \r\n"
               b"I (3514) common_APIs.c: Message : 0x3fca3708 \r\n"
               b"                            Version : 1:0 \r\n"
               b"                         Request ID : 0x00 \r\n"
               b"                          Object ID : 0x0200 \r\n"
               b"                               Type : 0x01 \r\n"
               b"                              Count : 1 \r\n\r\n"
               b"I (3524) common_APIs.c:   0x0018, Len/Res:2, \r\n[] \r\n"
               b"\x1b[0;32mI (3530) common_APIs.c: RxMessage : \x1b[0m\r\n"
               b"                         Request ID : 0x00 \r\n"
               b"                          Object ID : 0x0200 \r\n"
               b"                               Type : 0x81 \r\n"
               b"I (3534) common_APIs.c:   0x0018, Len/Res:2, \r\n[00 0E ] \r\n"
               b"I (3544) common_APIs.c: SUCCESS!! CiREAL Response for Request_ID: 0x00 received\r\n")
        # cut the log into USB-sized pieces, some in the middle of a line
        msgs = [(1.0 + k / 100, log[i:i + 37], 0) for k, i in enumerate(range(0, len(log), 37))]
        rec = {"ports": {"A": "COM3"}, "baud": 115200, "marks": [(0.99, "check")],
               "messages": [(t, "A", d, e) for t, d, e in msgs]}
        report, text = quiet(sp.analyze, rec)
        found = report["lines"]["A"]["cireal"]
        self.assertEqual([(m["dir"], m["obj"], m["op"]) for m in found],
                         [("to MCU", 0x0200, 0x01), ("from MCU", 0x0200, 0x81)])
        self.assertEqual(found[1]["attrs"], [(0x0018, 2, b"\x00\x0e")])
        self.assertIn("mission: error", text)
        self.assertIn("00 0E (= 14)", text)
        self.assertIn("from MCU get reply object 0x0200: 0x0018 mission: error = 00 0E (= 14)", text)

    def test_random_noise_has_no_format(self):
        rnd = random.Random(5)
        msgs = [bytes(rnd.randrange(256) for _ in range(rnd.randint(3, 30))) for _ in range(30)]
        fmt, _ = sp.detect_format(msgs)
        self.assertIsNone(fmt)

    def test_recording_file_round_trip(self):
        rec = tuya_recording()
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "rec.jsonl")
            with open(path, "w") as fh:
                fh.write(json.dumps({"type": "start", "baud": 115200, "ports": rec["ports"]}) + "\n")
                for t, d, data, e in rec["messages"]:
                    fh.write(json.dumps({"t": t, "dir": d, "hex": data.hex()}) + "\n")
                for t, label in rec["marks"]:
                    fh.write(json.dumps({"t": t, "mark": label}) + "\n")
            back = sp.load_capture(path)
        self.assertEqual(back["messages"][:3], rec["messages"][:3])
        self.assertEqual(back["marks"], rec["marks"])
        self.assertEqual(back["baud"], 115200)


@unittest.skipUnless(HAVE_PTY, "needs a pseudo-terminal")
class PortTests(unittest.TestCase):
    """A pseudo-terminal stands in for the USB-serial adapter."""

    def setUp(self):
        self.master, slave = pty.openpty()
        self.path = os.ttyname(slave)
        self.addCleanup(os.close, slave)
        self.addCleanup(os.close, self.master)

    def test_capture_records_and_decodes(self):
        frames = [tuya(0x07, bytes([0x0F, 2, 0, k % 3 + 1]) + k.to_bytes(k % 3 + 1, "big")) for k in range(12)]

        def robot():
            time.sleep(0.2)
            for f in frames:
                os.write(self.master, f)
                time.sleep(0.05)
        threading.Thread(target=robot, daemon=True).start()
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "cap.jsonl")
            report, text = quiet(sp.capture, [self.path], baud=115200, seconds=1.5, out_path=out,
                                 marks_from_stdin=False)
            rec = sp.load_capture(out)
        got = b"".join(data for _, _, data, _ in rec["messages"])
        self.assertEqual(got, b"".join(frames))
        self.assertEqual(report["lines"]["A"]["spec"], "len@4:u16be+7,chk=sum8@0")

    def test_ctrl_c_ends_cleanly(self):
        import signal
        import subprocess
        import sys
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "cap.jsonl")
            proc = subprocess.Popen([sys.executable, "-m", "roomba_mapper", "serial-probe", "--port", self.path,
                                     "--baud", "115200", "--seconds", "0", "--out", out],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL,
                                    cwd=os.path.join(os.path.dirname(__file__), ".."))
            time.sleep(1.0)
            os.write(self.master, b"hello\r\n")
            time.sleep(0.5)
            proc.send_signal(signal.SIGINT)
            stdout, stderr = proc.communicate(timeout=10)
            self.assertEqual(proc.returncode, 0, stderr.decode())
            self.assertNotIn(b"Traceback", stderr)
            self.assertIn(b"1 messages saved", stdout)
            with open(out) as fh:
                self.assertIn('"type": "stop"', fh.read())

    def test_reading_a_closed_port_is_a_serial_error(self):
        port = sp.SerialPort(self.path, 115200)
        port.close()
        with self.assertRaises(sp.SerialError):
            port.read(0.01)

    def test_real_ff_bytes_survive(self):
        port = sp.SerialPort(self.path, 115200)
        try:
            os.write(self.master, b"\x01\xFF\x02")
            data, errors = b"", 0
            end = time.time() + 2
            while len(data) < 3 and time.time() < end:
                chunk, e = port.read(0.1)
                data += chunk
                errors += e
            self.assertEqual((data, errors), (b"\x01\xFF\x02", 0))
        finally:
            port.close()

    def test_send_fills_checksum_and_shows_reply(self):
        def controller():
            got = b""
            while len(got) < 7:
                got += os.read(self.master, 64)
            os.write(self.master, tuya(0x00, b"\x01"))
        threading.Thread(target=controller, daemon=True).start()
        (frame, replies), _ = quiet(sp.send, self.path, 115200, bytes.fromhex("55AA00000000"),
                                    frame_format="len@4:u16be+7,chk=sum8@0", listen=1.0)
        self.assertEqual(frame, bytes.fromhex("55AA00000000FF"))
        self.assertEqual(replies, [tuya(0x00, b"\x01")])

    def test_not_a_port(self):
        with tempfile.NamedTemporaryFile() as fh:
            with self.assertRaises(sp.SerialError):
                sp.SerialPort(fh.name, 115200)
        with self.assertRaises(sp.SerialError):
            sp.SerialPort("/dev/does-not-exist", 115200)


class CliTests(unittest.TestCase):
    def test_commands_parse(self):
        p = build_parser()
        a = p.parse_args(["serial-probe", "--port", "/dev/ttyUSB0", "--port2", "/dev/ttyUSB1", "--baud", "9600"])
        self.assertEqual((a.cmd, a.port2, a.baud), ("serial-probe", "/dev/ttyUSB1", "9600"))
        a = p.parse_args(["serial-send", "--port", "/dev/ttyUSB0", "--frame-format", "chk=sum8@0", "55", "AA"])
        self.assertEqual(a.frame, ["55", "AA"])

    def test_analyze_saved_file(self):
        from roomba_mapper.__main__ import main
        rec = tuya_recording()
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "rec.jsonl")
            with open(path, "w") as fh:
                for t, d, data, e in rec["messages"]:
                    fh.write(json.dumps({"t": t, "dir": d, "hex": data.hex()}) + "\n")
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(main(["serial-probe", "--analyze", path]), 0)
        self.assertIn("For serial-send:  --frame-format len@4:u16be+7,chk=sum8@0", out.getvalue())


if __name__ == "__main__":
    unittest.main()
