#!/usr/bin/env python3
"""
ArduImageCapture - Camera Stoplight (Python edition)
====================================================
Standalone replacement for the ArduImageCapture Java tool by Indrek Luuk
(https://github.com/indrekluuk/ArduImageCapture), which only worked as a
plug-in for Arduino IDE 1.x. Arduino IDE 2.x has no Java "Tools" plug-in
system, so this runs as its own small desktop program.

Works together with the "stoplight" version of ExampleUart.cpp that comes with
it. The Arduino runs three start-up checks and this program shows the result:

    RED     1  Arduino runs the sketch and talks to the PC (upload, libraries, baud OK)
               checklist: link, baud, sketch version, board, mode, free memory
    YELLOW  2  Camera is powered and answers (3.3V, GND, SIOD, SIOC)
               checklist: SIOD line, SIOC line, camera answers, camera ID, power
    GREEN   3  Camera clock, VSYNC, PCLK and all 8 data wires (D0-D7) work
               checklist: one line per wire

Requirements
    pip install pyserial pillow
    (Linux only, if tkinter is missing:  sudo apt install python3-tk)

Usage
    python arduimagecapture.py                 # open the program
    python arduimagecapture.py --demo          # fake camera, everything works
    python arduimagecapture.py --demo-fault    # fake camera, two bad data wires
    python arduimagecapture.py --demo-nocam    # fake camera that is not powered
    python arduimagecapture.py --demo-i2c      # fake camera, SIOC missing its resistor
    python arduimagecapture.py --demo-vga      # fake 640x480 camera picture (try Picture: Large)
    python arduimagecapture.py --selftest      # test the decoder, no window

Only one program can use a serial port at a time: press Stop here before
uploading from the Arduino IDE, and close the IDE's Serial Monitor.
"""

import json
import math
import os
import queue
import sys
import threading
import time

MAX_W = 640
MAX_H = 480

# ---------------------------------------------------------------------------
# Protocol decoder (no GUI / serial dependencies - can be tested on its own)
# ---------------------------------------------------------------------------

START_COMMAND = 0x00
SUPPORTED_COMMAND_VERSION = 0x10
COMMAND_MASK = 0x0F
COMMAND_NEW_FRAME = 0x01
COMMAND_DEBUG_DATA = 0x03
COMMAND_CHECKPOINT = 0x04      # stoplight checks (stoplight sketch only)

FMT_RGB565_PARITY = 1
FMT_GRAY_PARITY = 2

H_PARITY_CHECK = 0b00100000
H_PARITY_INVERT = 0b00001000
L_PARITY_CHECK = 0b00001000
L_PARITY_INVERT = 0b00100000

R_OK, G_OK, B_OK = 1, 2, 4
ALL_OK = 7

# Checkpoint stages / results (must match the sketch)
STAGE_RED, STAGE_YELLOW, STAGE_GREEN = 1, 2, 3
RESULT_FAIL, RESULT_PASS, RESULT_TESTING = 0, 1, 2

BLANK_W, BLANK_H = 40, 30      # size of the little checkpoint pictures


class Frame:
    """RGB frame buffer. `valid` holds per-pixel flags of which channels
    were received intact (parity check) so damaged ones can be repaired."""

    def __init__(self, w, h, fmt):
        self.w, self.h, self.fmt = w, h, fmt
        self.rgb = bytearray(w * h * 3)
        self.valid = bytearray(w * h)
        self.pos = 0
        self.done = False
        self.bad = []

    def put(self, r, g, b, flags=ALL_OK):
        """Store a pixel. Returns True if this pixel completed a line."""
        if self.done:
            return False
        p = self.pos
        i = p * 3
        self.rgb[i] = r
        self.rgb[i + 1] = g
        self.rgb[i + 2] = b
        self.valid[p] = flags
        if flags != ALL_OK:
            self.bad.append(p)
        self.pos = p + 1
        return self.pos % self.w == 0

    @property
    def lines_done(self):
        return self.pos // self.w

    def fix_pixels(self, indexes):
        """Replace damaged channels with the average of valid neighbours."""
        w, h = self.w, self.h
        for p in indexes:
            x, y = p % w, p // w
            for ch, flag in ((0, R_OK), (1, G_OK), (2, B_OK)):
                if self.valid[p] & flag:
                    continue
                tot = cnt = 0
                for nx, ny in ((x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)):
                    if 0 <= nx < w and 0 <= ny < h:
                        q = ny * w + nx
                        if q < self.pos and self.valid[q] & flag:
                            tot += self.rgb[q * 3 + ch]
                            cnt += 1
                self.rgb[p * 3 + ch] = tot // cnt if cnt else 0


class Decoder:
    """Feed raw serial bytes with feed(). Callbacks:
         on_line(frame)                  a scan line is complete
         on_frame(frame)                 the last line of a frame is complete
         on_debug(text)                  debug text from the Arduino
         on_checkpoint(stage, result, extra_bytes)   stoplight check result"""

    def __init__(self, on_line=None, on_frame=None, on_debug=None, on_checkpoint=None):
        self.on_line = on_line or (lambda f: None)
        self.on_frame = on_frame or (lambda f: None)
        self.on_debug = on_debug or (lambda t: None)
        self.on_checkpoint = on_checkpoint or (lambda s, r, e: None)
        self.frame = Frame(MAX_W, MAX_H, FMT_RGB565_PARITY)
        self.cmd = None
        self.pend = bytearray()

    def reset(self):
        """Forget any half-received data (used when listening starts again)."""
        self.cmd = None
        self.pend = bytearray()
        self.frame = Frame(1, 1, FMT_RGB565_PARITY)

    # -- byte stream --------------------------------------------------------
    def feed(self, data):
        i, n = 0, len(data)
        while i < n:
            if self.cmd is not None:
                self._command_byte(data[i])
                i += 1
                continue
            j = data.find(START_COMMAND, i)
            end = n if j < 0 else j
            if end > i:
                self._pixels(data[i:end])
            if j < 0:
                break
            self.cmd = [None, bytearray()]
            self.pend.clear()
            i = j + 1

    # -- commands -----------------------------------------------------------
    def _command_byte(self, b):
        c = self.cmd
        if c[0] is None:
            c[0] = b
        else:
            c[1].append(b)
        if c[0] is not None and len(c[1]) >= c[0] + 1:   # data + checksum
            self.cmd = None
            self._process_command(c[0], bytes(c[1]))

    def _process_command(self, length, buf):
        chk = 0
        for k in range(length):
            chk ^= buf[k]
        if chk != buf[length]:
            self.on_debug("Command checksum failed!")
            return
        if length == 0:
            return
        code = buf[0] & COMMAND_MASK
        version = buf[0] & ~COMMAND_MASK & 0xFF
        if version != SUPPORTED_COMMAND_VERSION:
            self.on_debug("Received command version 0x%x. Supported 0x%x." %
                          (version, SUPPORTED_COMMAND_VERSION))
        if code == COMMAND_NEW_FRAME:
            self._new_frame(length, buf)
        elif code == COMMAND_DEBUG_DATA:
            if length > 1:
                self.on_debug(buf[1:length].decode("utf-8", "replace"))
        elif code == COMMAND_CHECKPOINT:
            if length >= 3:
                self.on_checkpoint(buf[1], buf[2], bytes(buf[3:length]))
        else:
            self.on_debug("Unknown command code '%d'" % code)

    def _new_frame(self, length, buf):
        if length != 4:
            self.on_debug("Invalid new-frame command length %d, should be 4" % length)
            return
        w = buf[1] | ((buf[3] & 0x03) << 8)
        h = buf[2] | ((buf[3] & 0x0C) << 6)
        fmt = (buf[3] & 0xF0) >> 4
        w = max(1, min(MAX_W, w))
        h = max(1, min(MAX_H, h))
        if fmt not in (FMT_RGB565_PARITY, FMT_GRAY_PARITY):
            self.on_debug("Unknown pixel format code '%d'" % fmt)
            fmt = FMT_RGB565_PARITY
        old = self.frame
        self.frame = Frame(w, h, fmt)
        if old.done and (old.w, old.h) == (w, h):
            self.frame.rgb[:] = old.rgb        # start from the last picture: no black flicker
        self.pend.clear()

    # -- pixels -------------------------------------------------------------
    def _pixels(self, seg):
        f = self.frame
        gray = f.fmt == FMT_GRAY_PARITY
        pend = self.pend
        for byte in seg:
            pend.append(byte)
            if len(pend) < 2:
                continue
            if gray:
                self._gray_pair(f, pend)
            else:
                self._rgb_pair(f, pend)
            if f.done:
                pend.clear()

    def _emit(self, f, r, g, b, flags):
        if f.put(r, g, b, flags):
            if f.bad:
                f.fix_pixels([p for p in f.bad if p // f.w == f.lines_done - 1])
            self.on_line(f)
            if f.lines_done >= f.h:
                f.done = True
                if f.bad:
                    f.fix_pixels(f.bad)
                self.on_frame(f)

    def _rgb_pair(self, f, pend):
        hi, lo = pend[0], pend[1]
        hi_ok = ((hi & H_PARITY_CHECK) != 0) != ((hi & H_PARITY_INVERT) != 0)
        lo_ok = ((lo & L_PARITY_CHECK) != 0) == ((lo & L_PARITY_INVERT) != 0)
        if hi_ok and lo_ok:
            pend.clear()
            raw = (hi << 8) | lo
            self._emit(f, (raw >> 8) & 0xF8, (raw >> 3) & 0xFC, (raw << 3) & 0xF8, ALL_OK)
        elif not hi_ok:
            del pend[0]
            self._emit(f, 0, 0, (hi << 3) & 0xF8, B_OK)
        else:
            del pend[0]
            self._emit(f, hi & 0xF8, 0, 0, R_OK)

    def _gray_pair(self, f, pend):
        b1, b2 = pend[0], pend[1]
        pend.clear()
        first_type = lambda v: (v & 1) == 0
        if first_type(b1):
            self._emit(f, b1, b1, b1, ALL_OK)
        else:
            self._emit(f, 0, 0, 0, 0)
            self._emit(f, b1, b1, b1, ALL_OK)
            pend.append(b2)
            return
        if not first_type(b2):
            self._emit(f, b2, b2, b2, ALL_OK)
        else:
            self._emit(f, 0, 0, 0, 0)
            pend.append(b2)


# ---------------------------------------------------------------------------
# Stoplight analysis: turns the Arduino's raw check results into plain language
# ---------------------------------------------------------------------------

# Where each OV7670 wire goes on an Arduino Uno (from the LiveOV7670 README)
UNO_PINS = {"VSYNC": "2", "PCLK": "12",
            "D0": "A0", "D1": "A1", "D2": "A2", "D3": "A3",
            "D4": "4", "D5": "5", "D6": "6", "D7": "7"}
WIRE_NAMES = ["VSYNC", "PCLK"] + ["D%d" % i for i in range(8)]
SKETCH_VERSION_EXPECTED = 2     # STOPLIGHT_SKETCH_VERSION in ExampleUart.cpp

# The checklist shown for each stage: (row key, label)
ROW_SPEC = {
    STAGE_RED: [("link", "Serial link to PC"), ("baud", "Baud rate"),
                ("sketch", "Stoplight sketch"), ("board", "Board & clock"),
                ("mode", "Selected mode"), ("ram", "Free memory")],
    STAGE_YELLOW: [("siod", "SIOD line (pin A4)"), ("sioc", "SIOC line (pin A5)"),
                   ("answer", "Camera answers"), ("id", "Camera ID"),
                   ("power", "3.3V, GND, RESET, PWDN")],
    STAGE_GREEN: [(n, "%s  (pin %s)" % (n, UNO_PINS[n])) for n in WIRE_NAMES],
}
TAB_TITLES = {STAGE_RED: "1 · Software", STAGE_YELLOW: "2 · Camera", STAGE_GREEN: "3 · Wires"}
FORMAT_NAMES = {1: "colour (RGB565)", 2: "grayscale"}


def analyze_red(extra):
    """Stage 1. extra = 13 bytes from the sketch (see sendStageOneReport):
    version, mode, width(2), height(2), pixel format, baud/100 (2), MHz, board, free RAM (2).
    Returns dict(ok, headline, advice, rows). The stage always passes: it only
    runs when the Arduino got through to the PC. Rows may carry warnings."""
    e = bytes(extra)
    rows = [("link", "ok", "OK - data is arriving from the Arduino"),
            ("baud", "ok", "OK - messages arrive clean")]
    if len(e) >= 13:
        ver, mode = e[0], e[1]
        w, h = e[2] | (e[3] << 8), e[4] | (e[5] << 8)
        fmt = e[6]
        baud = (e[7] | (e[8] << 8)) * 100
        mhz, board = e[9], e[10]
        ram = e[11] | (e[12] << 8)
        if ver == SKETCH_VERSION_EXPECTED:
            rows.append(("sketch", "ok", "OK - stoplight sketch version %d" % ver))
        else:
            rows.append(("sketch", "bad", "sketch is version %d, this program expects %d - "
                         "update both files together" % (ver, SKETCH_VERSION_EXPECTED)))
        if board == 1 and mhz == 16:
            rows.append(("board", "ok", "OK - Arduino Uno/Nano at 16 MHz"))
        elif board == 2:
            rows.append(("board", "warn", "Arduino Mega - this kit is wired for the Uno"))
        elif board == 1:
            rows.append(("board", "warn", "Uno/Nano running at %d MHz (expected 16)" % mhz))
        else:
            rows.append(("board", "warn", "board not recognised - this kit is made for the Uno"))
        rows.append(("mode", "ok", "OK - UART_MODE %d: %dx%d %s, %d baud"
                     % (mode, w, h, FORMAT_NAMES.get(fmt, "format %d" % fmt), baud)))
        if ram >= 300:
            rows.append(("ram", "ok", "OK - %d bytes free" % ram))
        elif ram >= 150:
            rows.append(("ram", "warn", "low - only %d bytes free" % ram))
        else:
            rows.append(("ram", "bad", "very low - only %d bytes free, the sketch may crash" % ram))
    else:
        for key in ("sketch", "board", "mode", "ram"):
            rows.append((key, "unknown", "not reported by this sketch"))
    problems = any(st in ("warn", "bad") for _, st, _ in rows)
    advice = "The sketch and its libraries compiled, uploaded and are running."
    if problems:
        advice += " Check the ⚠ / ✘ items in the Software tab."
    return dict(ok=True, rows=rows, advice=advice,
                headline="The Arduino is running your sketch and talking to the computer.")


def red_problem_rows(kind):
    """Stage 1 rows when the PC itself sees the problem ('silent' or 'garbled')."""
    if kind == "garbled":
        return [("link", "ok", "OK - data is arriving"),
                ("baud", "bad", "garbled - baud rate probably does not match the sketch")]
    return [("link", "bad", "nothing received - check port, cable and upload"),
            ("baud", "unknown", "can't tell until data arrives")]


def analyze_yellow(result, extra):
    """Stage 2. extra = (i2c_error, pid, ver, line_flags).
    line_flags: 0x01 SIOD rises (pull-up present), 0x02 SIOC rises, 0x04 SIOD/SIOC shorted.
    Returns dict(ok, headline, advice, rows)."""
    e = list(extra) + [0, 0, 0, 0]
    err, pid, ver = e[0], e[1], e[2]
    have_lines = len(extra) >= 4
    lines = e[3] if have_lines else 0
    siod_ok, sioc_ok, shorted = bool(lines & 1), bool(lines & 2), bool(lines & 4)
    answered = (err == 0)
    id_ok = answered and pid == 0x76

    def line_row(key, ok):
        if not have_lines:
            return (key, "unknown", "not reported by this sketch")
        if shorted:
            return (key, "bad", "touching the other camera wire (SIOD/SIOC short)")
        if ok:
            return (key, "ok", "OK - line idles high (10K pull-up present)")
        return (key, "bad", "stays LOW - 10K resistor to 3.3V missing, or wire touching GND")

    rows = [line_row("siod", siod_ok), line_row("sioc", sioc_ok)]
    if answered:
        rows.append(("answer", "ok", "OK - camera answered"))
    elif err == 5:
        rows.append(("answer", "bad", "timeout - the two wires are jammed"))
    else:
        rows.append(("answer", "bad", "no answer from the camera"))
    if not answered:
        rows.append(("id", "unknown", "can't read until the camera answers"))
    elif id_ok:
        rows.append(("id", "ok", "OK - ID 0x%02X 0x%02X (OV7670)" % (pid, ver)))
    else:
        rows.append(("id", "bad", "ID 0x%02X - this is not an OV7670" % pid))
    lines_healthy = (not have_lines) or (siod_ok and sioc_ok and not shorted)
    if answered:
        rows.append(("power", "ok", "OK - camera is awake"))
    elif lines_healthy:
        rows.append(("power", "warn", "suspect - check 3.3V, GND, RESET and PWDN"))
    else:
        rows.append(("power", "unknown", "fix the SIOD / SIOC wires first"))

    if result == RESULT_PASS:
        return dict(ok=True, rows=rows, headline="Camera is powered!",
                    advice="The Arduino can talk to the camera, and SIOD and SIOC look good.")
    if shorted:
        return dict(ok=False, rows=rows, headline="SIOD and SIOC are touching each other.",
                    advice="Separate the two wires: Arduino A4 and A5 must not connect.")
    if have_lines and not (siod_ok and sioc_ok):
        names = " and ".join(n for n, ok in (("SIOD (pin A4)", siod_ok),
                                              ("SIOC (pin A5)", sioc_ok)) if not ok)
        return dict(ok=False, rows=rows,
                    headline="Camera wire problem: %s does not idle high." % names,
                    advice=("Each of SIOD and SIOC needs a 10K resistor to 3.3V and must not touch "
                            "GND. Check that wire and its resistor."))
    if err == 5:
        return dict(ok=False, rows=rows, headline="The camera wires are jammed (I2C timeout).",
                    advice=("SIOD (A4) or SIOC (A5) may be swapped or shorted. Check both wires "
                            "and their 10K resistors to 3.3V."))
    if answered:
        return dict(ok=False, rows=rows,
                    headline=("Something answers, but it does not look like an OV7670 "
                              "(ID 0x%02X, expected 0x76)." % pid),
                    advice="Check that this is an OV7670 camera module.")
    return dict(ok=False, rows=rows, headline="The camera is not answering.",
                advice=("SIOD and SIOC look fine, so check the camera's power:\n"
                        "  1. 3.3V and GND wires are plugged in (3.3V, NOT 5V).\n"
                        "  2. RESET pin goes to 3.3V and PWDN pin goes to GND.\n"
                        "  3. SIOD goes to A4 and SIOC goes to A5 (not swapped)."))


def analyze_green(result, extra):
    """Stage 3. extra = (seen_high_mask, seen_low_mask, flags).
    Returns dict(ok, headline, advice, rows=[(wire, state, text)])
    where state is 'ok', 'bad' or 'unknown'."""
    seen_high, seen_low, flags = (list(extra) + [0, 0, 0])[:3]
    vs_high, vs_low = bool(flags & 0x01), bool(flags & 0x02)
    pc_high, pc_low = bool(flags & 0x04), bool(flags & 0x08)
    pc_ok = bool(flags & 0x10)
    cam_dead = bool(flags & 0x20)
    vs_ok = vs_high and vs_low

    rows = []
    if vs_ok:
        rows.append(("VSYNC", "ok", "OK - frame signal is changing"))
    elif vs_high:
        rows.append(("VSYNC", "bad", "stuck HIGH - wire missing or loose?"))
    else:
        rows.append(("VSYNC", "bad", "stuck LOW - wire touching GND?"))
    if pc_ok:
        rows.append(("PCLK", "ok", "OK - pixel clock is running"))
    elif pc_high and not pc_low:
        rows.append(("PCLK", "bad", "stuck HIGH - wire missing or loose?"))
    elif pc_low and not pc_high:
        rows.append(("PCLK", "bad", "stuck LOW - no pixel clock"))
    else:
        rows.append(("PCLK", "bad", "not pulsing"))

    bad_data = []
    for i in range(8):
        name = "D%d" % i
        if not pc_ok:
            rows.append((name, "unknown", "can't test until PCLK works"))
            continue
        hi = bool(seen_high >> i & 1)
        lo = bool(seen_low >> i & 1)
        if hi and lo:
            rows.append((name, "ok", "OK - data is changing"))
        elif hi:
            rows.append((name, "bad", "stuck HIGH - wire missing or loose?"))
            bad_data.append("%s (pin %s) stuck HIGH" % (name, UNO_PINS[name]))
        else:
            rows.append((name, "bad", "stuck LOW - wire touching GND?"))
            bad_data.append("%s (pin %s) stuck LOW" % (name, UNO_PINS[name]))

    if result == RESULT_PASS:
        return dict(ok=True, rows=rows,
                    headline="All data wires are connected correctly!",
                    advice=("Live video starts now. Colours look wrong? Two data wires may be "
                            "swapped (the wire check can't see swaps)."))
    if cam_dead:
        return dict(ok=False, rows=rows,
                    headline="The camera stopped answering during the test.",
                    advice="Re-check the stage 2 wires: power, SIOD (A4) and SIOC (A5).")
    if not vs_ok and not pc_ok:
        return dict(ok=False, rows=rows,
                    headline="No signal at all from the camera.",
                    advice=("The camera clock is probably not running. Check XCLK (pin 3, through the "
                            "5V->3.3V level shifter), RESET to 3.3V and PWDN to GND."))
    if not pc_ok:
        return dict(ok=False, rows=rows,
                    headline="VSYNC works, but there is no pixel clock.",
                    advice="Check the PCLK wire (Arduino pin 12).")
    if not vs_ok:
        return dict(ok=False, rows=rows,
                    headline="The pixel clock works, but VSYNC does not.",
                    advice="Check the VSYNC wire (Arduino pin 2).")
    return dict(ok=False, rows=rows,
                headline="Data wire problem: " + ", ".join(bad_data) + ".",
                advice=("Push each listed wire firmly in at both ends and make sure it goes to the "
                        "right camera pin."))


# ---------------------------------------------------------------------------
# Test / demo helpers: build the same byte stream the Arduino sends
# ---------------------------------------------------------------------------

def encode_command(code, payload=b""):
    data = bytes([SUPPORTED_COMMAND_VERSION | code]) + payload
    chk = 0
    for b in data:
        chk ^= b
    return bytes([START_COMMAND, len(data)]) + data + bytes([chk])


def encode_new_frame(w, h, fmt):
    b3 = ((w >> 8) & 3) | (((h >> 8) & 3) << 2) | (fmt << 4)
    return encode_command(COMMAND_NEW_FRAME, bytes([w & 0xFF, h & 0xFF, b3]))


def encode_checkpoint(stage, result, extra=b""):
    return encode_command(COMMAND_CHECKPOINT, bytes([stage, result]) + bytes(extra))


def encode_rgb_pixel(r, g, b):
    raw = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    hi, lo = raw >> 8, raw & 0xFF
    hi = (hi & ~H_PARITY_INVERT & 0xFF) | (0 if hi & H_PARITY_CHECK else H_PARITY_INVERT)
    lo = (lo & ~L_PARITY_INVERT & 0xFF) | (L_PARITY_INVERT if lo & L_PARITY_CHECK else 0)
    if lo == 0:
        lo = 1          # 0x00 is reserved as the command-start marker
    return bytes([hi, lo])


def solid_frame_bytes(color, w=BLANK_W, h=BLANK_H):
    return encode_new_frame(w, h, FMT_RGB565_PARITY) + encode_rgb_pixel(*color) * (w * h)


def demo_frame_bytes(w=160, h=120, t=0.0):
    """Moving colour-gradient test frame as a full wire-format byte string."""
    out = bytearray(encode_new_frame(w, h, FMT_RGB565_PARITY))
    for y in range(h):
        for x in range(w):
            r = int(255 * x / w)
            g = int(255 * y / h)
            b = int(127 + 127 * math.sin(t + (x + y) / 20.0))
            out += encode_rgb_pixel(r, g, b)
    return bytes(out)


RED_RGB, YELLOW_RGB, GREEN_RGB = (248, 0, 0), (248, 252, 0), (0, 252, 0)


def demo_hello():
    """Stage 1 report of a pretend Uno running UART_MODE 5 (320x240, 500000 baud)."""
    return bytes([2, 5, 320 & 0xFF, 320 >> 8, 240, 0, 1, 5000 & 0xFF, 5000 >> 8, 16, 1,
                  900 & 0xFF, 900 >> 8])


RECHECK = None      # a demo step whose chunk is None waits for the Recheck button


def demo_script(scenario):
    """Yields (bytes, seconds_to_wait) steps imitating the stoplight sketch.
    A step (None, 0) means "wait here until the student presses Recheck".
    scenario: ok | fault (bad data wires) | nocam (no power) | i2c (SIOC has no pull-up)
              | vga (everything works, 640x480 live picture)"""
    yield encode_checkpoint(STAGE_RED, RESULT_PASS, demo_hello()), 0.1
    yield solid_frame_bytes(RED_RGB), 1.6
    if scenario == "nocam":
        while True:                                   # the camera is never fixed
            yield encode_checkpoint(STAGE_YELLOW, RESULT_FAIL, bytes([2, 0xFF, 0xFF, 0x03])), 0.0
            yield RECHECK, 0.0
    if scenario == "i2c":
        yield encode_checkpoint(STAGE_YELLOW, RESULT_FAIL, bytes([2, 0xFF, 0xFF, 0x01])), 0.0
        yield RECHECK, 0.0                            # ...student fixes SIOC, then it passes
    yield encode_checkpoint(STAGE_YELLOW, RESULT_PASS, bytes([0, 0x76, 0x73, 0x03])), 0.1
    yield solid_frame_bytes(YELLOW_RGB), 1.6
    if scenario == "fault":
        yield encode_checkpoint(STAGE_GREEN, RESULT_TESTING), 3.0
        # first check: D2 stuck low and D5 stuck high; VSYNC + PCLK fine
        yield encode_checkpoint(STAGE_GREEN, RESULT_FAIL,
                                bytes([0xFF & ~(1 << 2), 0xFF & ~(1 << 5), 0x1F])), 0.0
        yield RECHECK, 0.0
        yield encode_checkpoint(STAGE_GREEN, RESULT_TESTING), 3.0
        # second check: D2 fixed, D5 still stuck high
        yield encode_checkpoint(STAGE_GREEN, RESULT_FAIL, bytes([0xFF, 0xFF & ~(1 << 5), 0x1F])), 0.0
        yield RECHECK, 0.0
    yield encode_checkpoint(STAGE_GREEN, RESULT_TESTING), 2.0
    yield encode_checkpoint(STAGE_GREEN, RESULT_PASS, bytes([0xFF, 0xFF, 0x1F])), 0.1
    yield solid_frame_bytes(GREEN_RGB), 1.6
    t = 0.0
    vw, vh = (640, 480) if scenario == "vga" else (160, 120)
    while True:
        yield demo_frame_bytes(vw, vh, t), 0.0
        t += 0.5


def selftest():
    errors = []
    lines, frames, dbg, cps = [], [], [], []

    def mk():
        return Decoder(lambda f: lines.append(f.lines_done), lambda f: frames.append(f),
                       dbg.append, lambda s, r, e: cps.append((s, r, bytes(e))))

    # 1. RGB frame + debug text + chunked input
    d = mk()
    w, h = 8, 4
    src = [[(x * 30 & 0xF8, y * 60 & 0xFC, (x + y) * 20 & 0xF8) for x in range(w)] for y in range(h)]
    stream = encode_command(COMMAND_DEBUG_DATA, b"hello camera") + encode_new_frame(w, h, FMT_RGB565_PARITY)
    for row in src:
        for px in row:
            stream += encode_rgb_pixel(*px)
    d.feed(stream[:20]); d.feed(stream[20:])
    if dbg != ["hello camera"]:
        errors.append("debug text: %r" % dbg)
    if lines != [1, 2, 3, 4] or len(frames) != 1:
        errors.append("RGB lines %r frames %d" % (lines, len(frames)))
    else:
        f = frames[-1]
        for y in range(h):
            for x in range(w):
                got = tuple(f.rgb[(y * w + x) * 3:(y * w + x) * 3 + 3])
                if any(abs(a - b) > 8 for a, b in zip(got, src[y][x])):
                    errors.append("RGB pixel %d,%d got %s expected %s" % (x, y, got, src[y][x]))
                    break

    # 2. dropped byte -> still completes
    frames.clear()
    d = mk()
    stream = encode_new_frame(w, h, FMT_RGB565_PARITY)
    for row in src:
        for px in row:
            stream += encode_rgb_pixel(*px)
    d.feed(stream[:30] + stream[31:] + encode_rgb_pixel(0, 0, 0) * 2)
    if not frames:
        errors.append("frame did not complete after dropped byte")

    # 3. grayscale
    frames.clear()
    d = mk()
    stream = encode_new_frame(4, 2, FMT_GRAY_PARITY)
    vals = [10, 100, 200, 30, 40, 50, 60, 250]
    for i, v in enumerate(vals):
        stream += bytes([(v & 0xFE) | (i & 1)])
    d.feed(stream)
    exp = [(v & 0xFE) | (i & 1) for i, v in enumerate(vals)]
    if not frames or [frames[-1].rgb[i * 3] for i in range(8)] != exp:
        errors.append("grayscale decode")

    # 4. bad checksum
    dbg.clear()
    d = mk()
    bad = bytearray(encode_new_frame(4, 4, 1)); bad[-1] ^= 0xFF
    d.feed(bytes(bad))
    if not dbg or "checksum" not in dbg[0]:
        errors.append("checksum failure not reported")

    # 5. checkpoint commands + small checkpoint frame
    cps.clear(); frames.clear()
    d = mk()
    d.feed(encode_checkpoint(STAGE_YELLOW, RESULT_FAIL, bytes([2, 255, 255])) +
           encode_checkpoint(STAGE_GREEN, RESULT_TESTING) + solid_frame_bytes(YELLOW_RGB))
    if cps != [(2, 0, bytes([2, 255, 255])), (3, 2, b"")]:
        errors.append("checkpoint decode %r" % cps)
    if not frames or (frames[-1].w, frames[-1].h) != (BLANK_W, BLANK_H):
        errors.append("blank frame size")
    elif tuple(frames[-1].rgb[0:3])[0] < 200 or tuple(frames[-1].rgb[0:3])[2] > 40:
        errors.append("yellow frame colour %r" % tuple(frames[-1].rgb[0:3]))

    # 6a. stage 1 report
    a = analyze_red(demo_hello())
    st = {k: s_ for k, s_, _ in a["rows"]}
    if not a["ok"] or any(v != "ok" for v in st.values()) or len(st) != 6:
        errors.append("stage 1 rows %r" % st)
    txt = {k: t for k, _, t in a["rows"]}
    if "UART_MODE 5" not in txt["mode"] or "320x240" not in txt["mode"] or "500000" not in txt["mode"]:
        errors.append("stage 1 mode text: " + txt["mode"])
    bad = bytearray(demo_hello()); bad[0] = 9; bad[10] = 2; bad[11] = 20; bad[12] = 0
    st = {k: s_ for k, s_, _ in analyze_red(bytes(bad))["rows"]}
    if st["sketch"] != "bad" or st["board"] != "warn" or st["ram"] != "bad":
        errors.append("stage 1 warnings %r" % st)
    st = {k: s_ for k, s_, _ in analyze_red(b"")["rows"]}
    if st["sketch"] != "unknown" or st["link"] != "ok":
        errors.append("stage 1 without report %r" % st)
    if red_problem_rows("garbled")[1][1] != "bad" or red_problem_rows("silent")[0][1] != "bad":
        errors.append("stage 1 problem rows")

    # 6b. stage 2 line checks
    a = analyze_yellow(RESULT_PASS, bytes([0, 0x76, 0x73, 0x03]))
    if not a["ok"] or "powered" not in a["headline"] or any(s_ != "ok" for _, s_, _ in a["rows"]):
        errors.append("yellow pass %r" % a["rows"])
    a = analyze_yellow(RESULT_FAIL, bytes([2, 255, 255, 0x03]))        # lines fine, camera silent
    st = {k: s_ for k, s_, _ in a["rows"]}
    if a["ok"] or "not answering" not in a["headline"] or "RESET" not in a["advice"] \
            or st["siod"] != "ok" or st["answer"] != "bad" or st["power"] != "warn":
        errors.append("yellow no answer %r / %s" % (st, a["headline"]))
    a = analyze_yellow(RESULT_FAIL, bytes([2, 255, 255, 0x01]))        # SIOC has no pull-up
    st = {k: s_ for k, s_, _ in a["rows"]}
    if st["siod"] != "ok" or st["sioc"] != "bad" or "SIOC (pin A5)" not in a["headline"] \
            or "SIOD" in a["headline"].replace("SIOC", "") or st["power"] != "unknown":
        errors.append("yellow sioc fault %r / %s" % (st, a["headline"]))
    a = analyze_yellow(RESULT_FAIL, bytes([2, 255, 255, 0x07]))        # shorted together
    if "touching each other" not in a["headline"]:
        errors.append("yellow short headline: " + a["headline"])
    a = analyze_yellow(RESULT_FAIL, bytes([2, 255, 255]))              # old sketch: no line info
    if dict((k, s_) for k, s_, _ in a["rows"])["siod"] != "unknown":
        errors.append("yellow old sketch rows")
    a = analyze_yellow(RESULT_FAIL, bytes([0, 0x70, 0x00, 0x03]))      # answers but wrong ID
    if "OV7670" not in a["headline"] or dict((k, s_) for k, s_, _ in a["rows"])["id"] != "bad":
        errors.append("yellow wrong id")
    a = analyze_yellow(RESULT_FAIL, bytes([5, 0, 0, 0x03]))
    if "jammed" not in a["headline"]:
        errors.append("yellow timeout")

    # 7. stage 3 analysis
    a = analyze_green(RESULT_PASS, bytes([0xFF, 0xFF, 0x1F]))
    if not a["ok"] or any(s != "ok" for _, s, _ in a["rows"]):
        errors.append("green pass rows")
    a = analyze_green(RESULT_FAIL, bytes([0xFF & ~(1 << 2), 0xFF & ~(1 << 5), 0x1F]))
    st = {n: s for n, s, _ in a["rows"]}
    if a["ok"] or st["D2"] != "bad" or st["D5"] != "bad" or st["D0"] != "ok":
        errors.append("green fault rows %r" % st)
    if "D2" not in a["headline"] or "stuck HIGH" not in a["headline"] or "A2" not in a["headline"]:
        errors.append("green fault headline: " + a["headline"])
    # seen_high missing => stuck LOW ; seen_low missing => stuck HIGH
    texts = {n: t for n, _, t in a["rows"]}
    if "LOW" not in texts["D2"] or "HIGH" not in texts["D5"]:
        errors.append("stuck high/low wording")
    a = analyze_green(RESULT_FAIL, bytes([0, 0, 0x01]))     # nothing running at all
    if "No signal" not in a["headline"] or "XCLK" not in a["advice"]:
        errors.append("no-signal message")
    a = analyze_green(RESULT_FAIL, bytes([0, 0, 0x03]))     # vsync fine, no pclk
    if "PCLK" not in a["advice"]:
        errors.append("pclk message")
    if any(s != "unknown" for n, s, _ in a["rows"] if n.startswith("D")):
        errors.append("data lines should be 'unknown' without PCLK")

    # 8. the demo script decodes end to end
    cps.clear(); frames.clear()
    d = mk()
    it = demo_script("ok")
    for _ in range(9):
        d.feed(next(it)[0])
    if [c[:2] for c in cps] != [(1, 1), (2, 1), (3, 2), (3, 1)] or len(frames) < 4 \
            or len(cps[0][2]) != 13 or len(cps[1][2]) != 4:
        errors.append("demo script %r frames=%d" % (cps, len(frames)))

    # 8b. failing demo scenarios wait for Recheck; advice no longer promises auto-retests
    import itertools
    steps = [c for c, _ in itertools.islice(demo_script("fault"), 13)]
    if steps.count(None) != 2:
        errors.append("fault demo should wait twice for Recheck: %r" % steps.count(None))
    if sum(1 for c, _ in itertools.islice(demo_script("nocam"), 8) if c is None) < 2:
        errors.append("nocam demo should keep waiting for Recheck")
    texts = []
    for rr, ex in ((RESULT_FAIL, bytes([2, 255, 255, 0x03])), (RESULT_FAIL, bytes([2, 255, 255, 0x01]))):
        texts.append(analyze_yellow(rr, ex)["advice"])
    texts.append(analyze_green(RESULT_FAIL, bytes([0xFB, 0xFF, 0x1F]))["advice"])
    if any("retest" in t_ for t_ in texts):
        errors.append("advice still mentions automatic retests")

    msg = friendly_serial_error("COM3", Exception("PermissionError(13, 'Access is denied.')"))
    if "busy" not in msg:
        errors.append("busy-port message")

    if errors:
        print("SELFTEST FAILED")
        for e in errors:
            print(" -", e)
        return 1
    print("SELFTEST OK: frames, checkpoints, stoplight analysis and demo script all behave.")
    return 0


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

CONFIG_PATH = os.path.join(os.path.expanduser("~"), ".arduimagecapture.json")
BAUD_RATES = [2000000, 1000000, 500000, 250000, 230400, 115200]
IMG_BOX_W, IMG_BOX_H = 480, 360          # "Normal" picture area
IMG_LARGE_W, IMG_LARGE_H = 640, 480        # "Large" picture area (full-size VGA)


def load_config():
    try:
        with open(CONFIG_PATH) as fh:
            return json.load(fh)
    except Exception:
        return {}


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w") as fh:
            json.dump(cfg, fh)
    except Exception:
        pass


def friendly_serial_error(port, e):
    """Turn raw pyserial/OS errors into something a student can act on."""
    raw = str(e)
    m = raw.lower()
    if "permission denied" in m and sys.platform.startswith("linux"):
        return ("Linux is not letting you use %s.\n\nFix (one time): run\n"
                "   sudo usermod -a -G dialout $USER\nthen log out and back in." % port)
    if any(k in m for k in ("access is denied", "permission", "busy", "in use", "errno 13")):
        return ("%s is busy.\n\nSomething else is using the board. Try:\n"
                "  1. Close the Arduino Serial Monitor / Serial Plotter\n"
                "  2. Close any other copy of this program\n"
                "  3. Unplug the USB cable, plug it back in, then press Listen" % port)
    if any(k in m for k in ("no such file", "could not find", "cannot find", "errno 2", "filenotfound")):
        return ("%s was not found.\n\nIs the Arduino plugged in? Plug it in, wait a few seconds, "
                "then choose the port again." % port)
    return "Could not open %s.\n\nDetails: %s" % (port, raw)


LAMPS = [
    (STAGE_RED, "RED", "#d32f2f", "#f7d6d6", "Arduino & software"),
    (STAGE_YELLOW, "YELLOW", "#f2b600", "#fbefc2", "Camera powered"),
    (STAGE_GREEN, "GREEN", "#2e9e44", "#d3eed6", "Data wires"),
]
ROW_STYLE = {          # state -> (icon, colour)
    "idle": ("·", "#9a9a9a"), "busy": ("…", "#1a3d7c"), "ok": ("✔", "#14833b"),
    "bad": ("✘", "#c62828"), "warn": ("⚠", "#b36b00"), "unknown": ("?", "#9a9a9a"),
}


def run_gui(demo=None):
    try:
        import tkinter as tk
        from tkinter import ttk, messagebox
    except ImportError:
        sys.exit("tkinter is missing. Linux: sudo apt install python3-tk")
    try:
        from PIL import Image, ImageTk
    except ImportError:
        sys.exit("Pillow is missing. Run: pip install pillow")
    serial = list_ports = None
    if not demo:
        try:
            import serial
            import serial.tools.list_ports as list_ports
        except ImportError:
            sys.exit("pyserial is missing. Run: pip install pyserial")

    cfg = load_config()
    events = queue.SimpleQueue()          # worker threads -> GUI thread
    S = {"stop": None, "thread": None, "photo": None, "dirty": False, "placeholder": True,
         "listen_since": 0.0, "got_data": False, "silent_warned": False, "frames": 0,
         "checksum_shown": False, "got_checkpoint": False, "legacy_warned": False,
         "port": "", "baud": 0, "lamp": {1: "off", 2: "off", 3: "off"},
         "stage3_ok": False, "reveal": [], "ticks": 0, "last_scan": 0.0, "ports": None,
         "attempts": {1: 1, 2: 1, 3: 1}, "tx": None}

    def on_line(f):
        S["placeholder"] = False
        S["dirty"] = True

    decoder = Decoder(
        on_line=on_line,
        on_frame=lambda f: events.put(("frame", f.w, f.h)),
        on_debug=lambda t: events.put(("debug", t)),
        on_checkpoint=lambda s, r, e: events.put(("checkpoint", s, r, e)),
    )

    def feed(data):
        S["got_data"] = True
        decoder.feed(data)

    root = tk.Tk()
    root.title("ArduImageCapture - Camera Stoplight")

    # ---- toolbar ---------------------------------------------------------
    bar = ttk.Frame(root, padding=4)
    bar.pack(side="top", fill="x")
    port_var = tk.StringVar()
    baud_var = tk.StringVar(value=str(cfg.get("baud", 500000)))
    ttk.Label(bar, text="Port").pack(side="left")
    port_box = ttk.Combobox(bar, textvariable=port_var, width=24, state="readonly")
    port_box.pack(side="left", padx=(2, 8))
    ttk.Label(bar, text="Baud").pack(side="left")
    baud_box = ttk.Combobox(bar, textvariable=baud_var, width=9, state="readonly",
                            values=[str(b) for b in BAUD_RATES])
    baud_box.pack(side="left", padx=(2, 8))
    listen_btn = ttk.Button(bar, text="Listen")
    stop_btn = ttk.Button(bar, text="Stop", state="disabled")
    restart_btn = ttk.Button(bar, text="↻ Restart")
    listen_btn.pack(side="left", padx=2)
    stop_btn.pack(side="left", padx=2)
    restart_btn.pack(side="left", padx=2)
    ttk.Label(bar, text="Picture").pack(side="left", padx=(12, 2))
    size_var = tk.StringVar(value=cfg.get("picture", "Normal"))
    size_box = ttk.Combobox(bar, textvariable=size_var, width=7, state="readonly",
                            values=["Normal", "Large"])
    size_box.pack(side="left")
    pic_box = {"w": IMG_BOX_W, "h": IMG_BOX_H}
    if size_var.get() == "Large":
        pic_box.update(w=IMG_LARGE_W, h=IMG_LARGE_H)

    # ---- stoplight -------------------------------------------------------
    lamps = tk.Canvas(root, width=660, height=108, highlightthickness=0)
    lamps.pack(side="top", pady=(4, 0))

    def draw_lamps():
        lamps.delete("all")
        blink = (S["ticks"] // 12) % 2 == 0          # ~0.5 s
        for idx, (n, name, col, tint, sub) in enumerate(LAMPS):
            cx, cy, r = 110 + idx * 220, 34, 26
            st = S["lamp"][n]
            fill, outline, width, glyph, gcol = "#dcdcdc", "#a8a8a8", 2, "", "#ffffff"
            if st == "ok":
                fill, outline, glyph, gcol = col, col, "✔", "#ffffff"
            elif st == "bad":
                fill, outline, width, glyph, gcol = (tint if blink else "#ffffff"), col, 4, "!", col
            elif st == "busy":
                fill, outline, width, glyph, gcol = tint, col, 3, "…", col
            lamps.create_oval(cx - r, cy - r, cx + r, cy + r, fill=fill, outline=outline, width=width)
            if glyph:
                lamps.create_text(cx, cy - 1, text=glyph, fill=gcol, font=("TkDefaultFont", 20, "bold"))
            lamps.create_text(cx, 74, text="%d · %s" % (n, name), font=("TkDefaultFont", 10, "bold"))
            lamps.create_text(cx, 92, text=sub, font=("TkDefaultFont", 9), fill="#555555")
            if idx < 2:
                lamps.create_text(cx + 110, cy, text="▶", fill="#b0b0b0", font=("TkDefaultFont", 14))

    # ---- status strip: one short line (details live in the tabs below) ------
    status = tk.Label(root, text="", anchor="w", justify="left", padx=8, pady=4, height=2,
                      font=("TkDefaultFont", 10, "bold"), wraplength=1040)
    status.pack(side="top", fill="x", pady=(2, 0))
    COLORS = {"info": ("#e8f0fe", "#1a3d7c"), "ok": ("#d9f2d9", "#14532d"),
              "warn": ("#fff3cd", "#7a5200"), "error": ("#f8d7da", "#7a1c24")}

    def set_status(text, kind="info"):
        bg, fg = COLORS[kind]
        status.config(text=text, bg=bg, fg=fg)

    # ---- middle: picture + wire checklist --------------------------------
    mid = ttk.Frame(root)
    mid.pack(side="top", padx=6, pady=4)
    pic = tk.Canvas(mid, width=pic_box["w"], height=pic_box["h"], bg="black", highlightthickness=0)
    pic.pack(side="left")
    notebook = ttk.Notebook(mid)
    notebook.pack(side="left", padx=(8, 0), fill="both")
    tab_frames, row_widgets, advice_widgets = {}, {}, {}
    for stage in (STAGE_RED, STAGE_YELLOW, STAGE_GREEN):
        fr = ttk.Frame(notebook, padding=8)
        notebook.add(fr, text=TAB_TITLES[stage])
        tab_frames[stage] = fr
        for r_i, (key, label) in enumerate(ROW_SPEC[stage]):
            icon = ttk.Label(fr, text="·", width=2, anchor="center")
            lab = ttk.Label(fr, text=label, width=22, anchor="w")
            det = ttk.Label(fr, text="not tested yet", width=40, anchor="w",
                            wraplength=300, justify="left")
            icon.grid(row=r_i, column=0, pady=2, sticky="n")
            lab.grid(row=r_i, column=1, sticky="nw", pady=2)
            det.grid(row=r_i, column=2, sticky="nw", pady=2)
            row_widgets[(stage, key)] = (icon, lab, det)
        # "What it means / what to do" box at the bottom of the tab
        box = tk.Frame(fr)
        head = tk.Label(box, text="", anchor="w", justify="left", padx=8, pady=4,
                        font=("TkDefaultFont", 10, "bold"), wraplength=440)
        body = tk.Label(box, text="", anchor="w", justify="left", padx=8, pady=2, wraplength=440)
        head.pack(fill="x")
        body.pack(fill="x")
        btn = ttk.Button(box, text="↻ Recheck")          # shown only when there is a problem
        box.grid(row=len(ROW_SPEC[stage]) + 1, column=0, columnspan=3, sticky="ew", pady=(12, 0))
        box.grid_remove()
        advice_widgets[stage] = (box, head, body, btn)

    def set_advice(stage, headline, advice, kind, button=None):
        """button: None, "recheck" (retest this stage) or "restart" (start over)."""
        box, head, body, btn = advice_widgets[stage]
        bg, fg = COLORS[kind]
        if button == "recheck":
            advice += "\n\nWhen you have fixed it, press Recheck."
        box.config(bg=bg)
        head.config(text=headline, bg=bg, fg=fg)
        body.config(text=advice, bg=bg, fg=fg)
        btn.pack_forget()
        if button == "recheck":
            btn.config(text="↻ Recheck", command=lambda: recheck(stage))
            btn.pack(anchor="w", padx=8, pady=(4, 8))
        elif button == "restart":
            btn.config(text="↻ Restart", command=restart)
            btn.pack(anchor="w", padx=8, pady=(4, 8))
        box.grid()

    def clear_advice(*stages):
        for stage in (stages or (STAGE_RED, STAGE_YELLOW, STAGE_GREEN)):
            advice_widgets[stage][0].grid_remove()

    def set_row(stage, key, state, text=None):
        icon, lab, det = row_widgets[(stage, key)]
        sym, col = ROW_STYLE[state]
        default = {"idle": "not tested yet", "busy": "checking…"}.get(state, "")
        icon.config(text=sym, foreground=col)
        det.config(text=text or default, foreground=col)

    def reset_rows(*stages):
        stages = stages or (STAGE_RED, STAGE_YELLOW, STAGE_GREEN)
        S["reveal"] = [r for r in S["reveal"] if r[0] not in stages]
        for stage in stages:
            for key, _ in ROW_SPEC[stage]:
                set_row(stage, key, "idle")

    def show_tab(stage):
        try:
            notebook.select(tab_frames[stage])
        except Exception:
            pass

    def queue_rows(stage, rows):
        """Rows appear one at a time (see tick) so students can follow the checks."""
        S["reveal"] = [r for r in S["reveal"] if r[0] != stage] + \
                      [(stage, k, st, tx) for (k, st, tx) in rows]

    log = tk.Text(root, height=3, width=88)
    log.pack(side="top", fill="x", padx=6, pady=(0, 6))

    def log_line(text):
        log.insert("end", text + "\n")
        if int(log.index("end-1c").split(".")[0]) > 200:
            log.delete("1.0", "50.0")
        log.see("end")

    # ---- picture drawing -------------------------------------------------
    def draw_picture():
        pic.delete("all")
        bw, bh = pic_box["w"], pic_box["h"]
        if S["placeholder"]:
            pic.create_text(bw // 2, bh // 2, fill="#8a8a8a", justify="center",
                            font=("TkDefaultFont", 12),
                            text="Waiting for the Arduino…" if S["thread"] else
                            "Press Listen to start")
            return
        f = decoder.frame
        img = Image.frombytes("RGB", (f.w, f.h), bytes(f.rgb))
        fit = min(bw / f.w, bh / f.h)
        if fit >= 1:
            k = int(fit)
            if k > 1:
                img = img.resize((f.w * k, f.h * k), Image.NEAREST)
        else:
            img = img.resize((max(1, int(f.w * fit)), max(1, int(f.h * fit))), Image.BILINEAR)
            if not S.get("shrink_hint") and size_var.get() == "Normal":
                S["shrink_hint"] = True
                log_line("This picture (%dx%d) is shown reduced to fit. Choose Picture: Large "
                         "to see it at full size." % (f.w, f.h))
        S["photo"] = ImageTk.PhotoImage(img)
        pic.create_image(bw // 2, bh // 2, image=S["photo"])

    def apply_size(event=None):
        if size_var.get() == "Large":
            pic_box.update(w=IMG_LARGE_W, h=IMG_LARGE_H)
        else:
            pic_box.update(w=IMG_BOX_W, h=IMG_BOX_H)
        pic.config(width=pic_box["w"], height=pic_box["h"])
        S["shrink_hint"] = False
        S["dirty"] = True
        cfg["picture"] = size_var.get()
        save_config(cfg)

    size_box.bind("<<ComboboxSelected>>", apply_size)

    # ---- view reset (what makes Restart unambiguous) ---------------------
    def reset_view():
        S.update(lamp={1: "off", 2: "off", 3: "off"}, stage3_ok=False, placeholder=True,
                 got_data=False, silent_warned=False, frames=0, checksum_shown=False,
                 got_checkpoint=False, legacy_warned=False, dirty=True)
        decoder.reset()
        reset_rows()
        clear_advice()
        log.delete("1.0", "end")

    # ---- port handling ---------------------------------------------------
    def scan_ports(force=False):
        if demo:
            ports = ["DEMO"]
        else:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = S["ports"] or []
        if ports != S["ports"] or force:
            S["ports"] = ports
            port_box["values"] = ports
            want = cfg.get("port")
            if port_var.get() not in ports:
                port_var.set(want if want in ports else (ports[0] if ports else ""))
            if not S["thread"]:
                if not ports:
                    set_status("No Arduino found — plug it in with the USB cable "
                               "(this list updates by itself).", "warn")
                else:
                    set_status("Ready — choose the port and the same baud rate as your sketch, "
                               "then press Listen.", "info")

    # ---- capture threads -------------------------------------------------
    def reset_board(ser):
        """Pulse DTR so the Uno restarts and replays its start-up checks."""
        try:
            ser.dtr = False
            time.sleep(0.05)
            ser.dtr = True
            time.sleep(0.1)
            ser.dtr = False
            time.sleep(0.05)
            ser.reset_input_buffer()
        except Exception:
            pass

    def serial_worker(port, baud, stop, tx):
        try:
            ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:
            events.put(("open_failed", friendly_serial_error(port, e)))
            return
        try:
            reset_board(ser)
            while not stop.is_set():
                data = ser.read(max(1, ser.in_waiting))
                if data:
                    feed(data)
                while True:                            # messages for the Arduino (Recheck)
                    try:
                        out = tx.get_nowait()
                    except queue.Empty:
                        break
                    ser.write(out)
                    ser.flush()
        except Exception:
            if not stop.is_set():
                events.put(("lost", "Lost connection to %s.\n\nWas the USB cable unplugged? Plug it "
                            "back in and press Listen." % port))
        finally:
            try:
                ser.close()
            except Exception:
                pass

    def demo_worker(stop, tx):
        time.sleep(0.6)
        for chunk, wait in demo_script(demo):
            if chunk is None:                          # wait for the Recheck button
                while not stop.is_set():
                    try:
                        tx.get(timeout=0.1)
                        break
                    except queue.Empty:
                        pass
                time.sleep(0.4)                        # pretend the Arduino is retesting
                continue
            for k in range(0, len(chunk), 512):          # drip-feed like a UART
                if stop.is_set():
                    return
                feed(chunk[k:k + 512])
                time.sleep(0.004 if len(chunk) > 600 else 0)
            end = time.time() + wait
            while time.time() < end:
                if stop.is_set():
                    return
                time.sleep(0.05)

    def set_idle():
        S["thread"] = None
        listen_btn.config(state="normal")
        stop_btn.config(state="disabled")
        port_box.config(state="readonly")
        baud_box.config(state="readonly")

    def start():
        if S["thread"]:
            return
        port = port_var.get()
        if not demo and not port:
            set_status("No port selected — plug in the Arduino and wait for it to appear "
                       "in the Port list.", "warn")
            messagebox.showwarning("No port selected",
                                   "No serial port is selected.\n\nPlug in the Arduino with the USB "
                                   "cable and choose the port from the list.")
            return
        baud = int(baud_var.get())
        reset_view()                                   # clear old picture, lamps and checks
        stop = threading.Event()
        tx = queue.SimpleQueue()                       # PC -> Arduino messages
        S.update(stop=stop, tx=tx, port=port, baud=baud, listen_since=time.time(),
                 attempts={1: 1, 2: 1, 3: 1})
        target = demo_worker if demo else serial_worker
        args = (stop, tx) if demo else (port, baud, stop, tx)
        th = threading.Thread(target=target, args=args, daemon=True)
        S["thread"] = th
        th.start()
        listen_btn.config(state="disabled")
        stop_btn.config(state="normal")
        port_box.config(state="disabled")
        baud_box.config(state="disabled")
        set_status("Listening on %s at %d baud — the Arduino is restarting, the red light "
                   "should come on in a moment." % (port, baud), "info")
        cfg.update(port=port, baud=baud)
        save_config(cfg)

    def stop(message=True):
        if S["stop"]:
            S["stop"].set()
        th = S["thread"]
        if th:
            th.join(timeout=1)
        set_idle()
        if message:
            set_status("Stopped — the port is free, so you can upload from the Arduino IDE. "
                       "Press Listen when you are done.", "info")

    def restart():
        stop(message=False)
        time.sleep(0.2)
        start()

    def recheck(stage):
        """Tell the Arduino to test this stage again (it waits after a failure)."""
        if not S["thread"] or S["tx"] is None:
            return
        S["attempts"][stage] += 1
        S["tx"].put(b"R")
        S["lamp"][stage] = "busy"
        S["reveal"] = [r for r in S["reveal"] if r[0] != stage]
        for key, _ in ROW_SPEC[stage]:
            set_row(stage, key, "busy")
        clear_advice(stage)
        set_status("Rechecking stage %d (check #%d)…" % (stage, S["attempts"][stage]), "info")

    listen_btn.config(command=start)
    stop_btn.config(command=stop)
    restart_btn.config(command=restart)

    # ---- event handling --------------------------------------------------
    def handle_debug(text):
        if "checksum" in text.lower():
            if not S["checksum_shown"]:
                S["checksum_shown"] = True
                S["lamp"][1] = "bad"
                show_tab(STAGE_RED)
                for k, st, tx in red_problem_rows("garbled"):
                    set_row(STAGE_RED, k, st, tx)
                log_line("Garbled data received.")
                set_advice(STAGE_RED, "The data is garbled, so the baud rate is probably wrong.",
                           "Choose the SAME baud rate as UART_MODE in your sketch "
                           "(modes 1-2 = 115200, 3-6 = 500000, 7-11 = 1000000; modes 12-17 = 2000000 are for "
                           "faster boards such as the Mega, not the Uno), then press Restart.", "error",
                           button="restart")
                set_status("Stage 1 · RED problem — garbled data (wrong baud rate?). "
                           "See the Software tab.", "error")
            return
        log_line(text)

    def check_note(stage):
        n = S["attempts"][stage]
        return " (check #%d)" % n if n > 1 else ""

    def handle_checkpoint(stage, result, extra):
        S["got_checkpoint"] = True
        S["checksum_shown"] = False
        show_tab(stage)
        if stage == STAGE_RED:
            a = analyze_red(extra)
            S["attempts"] = {1: 1, 2: 1, 3: 1}
            S["lamp"] = {1: "ok", 2: "off", 3: "off"}
            S["stage3_ok"] = False
            reset_rows()
            clear_advice()
            queue_rows(STAGE_RED, a["rows"])
            set_advice(STAGE_RED, a["headline"], a["advice"], "ok")
            set_status("✔ Stage 1 · RED passed — Arduino and software are OK. "
                       "Checking the camera…", "ok")
        elif stage == STAGE_YELLOW:
            a = analyze_yellow(result, extra)
            S["lamp"][1] = "ok"
            S["lamp"][2] = "ok" if a["ok"] else "bad"
            S["lamp"][3] = "off"
            S["stage3_ok"] = False
            reset_rows(STAGE_GREEN)
            clear_advice(STAGE_GREEN)
            queue_rows(STAGE_YELLOW, a["rows"])           # not reset: no flicker on retests
            set_advice(STAGE_YELLOW, a["headline"], a["advice"], "ok" if a["ok"] else "error",
                       button=None if a["ok"] else "recheck")
            if a["ok"]:
                set_status("✔ Stage 2 · YELLOW passed%s — %s Checking the data wires…"
                           % (check_note(STAGE_YELLOW), a["headline"]), "ok")
            else:
                set_status("Stage 2 · YELLOW%s — %s Fix it, then press Recheck in the Camera tab."
                           % (check_note(STAGE_YELLOW), a["headline"]), "error")
        elif stage == STAGE_GREEN:
            S["lamp"][1] = S["lamp"][2] = "ok"
            if result == RESULT_TESTING:
                S["lamp"][3] = "busy"
                S["reveal"] = [r for r in S["reveal"] if r[0] != STAGE_GREEN]
                clear_advice(STAGE_GREEN)
                for name in WIRE_NAMES:
                    set_row(STAGE_GREEN, name, "busy")
                set_status("Stage 3 · GREEN%s — checking the camera clock, VSYNC and data wires "
                           "(about 4 seconds)…" % check_note(STAGE_GREEN), "info")
                return
            a = analyze_green(result, extra)
            S["lamp"][3] = "ok" if a["ok"] else "bad"
            S["stage3_ok"] = a["ok"]
            queue_rows(STAGE_GREEN, a["rows"])
            set_advice(STAGE_GREEN, a["headline"], a["advice"], "ok" if a["ok"] else "error",
                       button=None if a["ok"] else "recheck")
            if a["ok"]:
                set_status("✔ Stage 3 · GREEN passed%s — %s" % (check_note(STAGE_GREEN), a["headline"]),
                           "ok")
            else:
                set_status("Stage 3 · GREEN%s — %s Fix it, then press Recheck in the Wires tab."
                           % (check_note(STAGE_GREEN), a["headline"]), "error")

    def handle_frame(w, h):
        S["frames"] += 1
        S["checksum_shown"] = False
        if S["stage3_ok"] and (w, h) != (BLANK_W, BLANK_H):
            set_status("✔ Live video — %s, %dx%d, %d baud — frame %d"
                       % (S["port"], w, h, S["baud"], S["frames"]), "ok")
        elif not S["got_checkpoint"]:
            set_status("Receiving video from %s at %d baud (%dx%d) — frame %d"
                       % (S["port"], S["baud"], w, h, S["frames"]), "ok")

    def tick():
        S["ticks"] += 1
        while True:
            try:
                ev = events.get_nowait()
            except queue.Empty:
                break
            if ev[0] == "debug":
                handle_debug(ev[1])
            elif ev[0] == "checkpoint":
                handle_checkpoint(ev[1], ev[2], ev[3])
            elif ev[0] == "frame":
                handle_frame(ev[1], ev[2])
            elif ev[0] in ("open_failed", "lost"):
                set_idle()
                S["lamp"][1] = "bad"
                head, _, rest = ev[1].partition("\n\n")
                show_tab(STAGE_RED)
                set_row(STAGE_RED, "link", "bad",
                        "can't open the port" if ev[0] == "open_failed" else "connection lost")
                set_row(STAGE_RED, "baud", "unknown", "can't tell until data arrives")
                set_advice(STAGE_RED, head, rest, "error", button="restart")
                set_status("Stage 1 · RED problem — %s See the Software tab." % head, "error")
                if ev[0] == "open_failed":
                    messagebox.showerror("Cannot open port", ev[1])

        # Reveal the check results one line at a time (every ~0.08 s)
        if S["reveal"] and S["ticks"] % 2 == 0:
            stage, key, st, text = S["reveal"].pop(0)
            set_row(stage, key, st, text)

        # Nothing arriving at all?
        if S["thread"] and not S["got_data"] and not S["silent_warned"] \
                and time.time() - S["listen_since"] > 6:
            S["silent_warned"] = True
            S["lamp"][1] = "bad"
            show_tab(STAGE_RED)
            for k, st, tx in red_problem_rows("silent"):
                set_row(STAGE_RED, k, st, tx)
            set_advice(STAGE_RED, "Nothing has arrived for 6 seconds.",
                       "1) Is the stoplight sketch uploaded (EXAMPLE 3 in setup.h)?\n"
                       "2) Is the baud rate the same as UART_MODE?\n"
                       "3) Is this the right port?\n"
                       "4) Try Restart.", "error", button="restart")
            set_status("Stage 1 · RED problem — nothing received yet. See the Software tab.",
                       "error")
        # Old sketch without stoplight checks?
        if S["thread"] and S["frames"] > 0 and not S["got_checkpoint"] and not S["legacy_warned"] \
                and time.time() - S["listen_since"] > 10:
            S["legacy_warned"] = True
            log_line("This sketch has no stoplight checks (old version) - showing video only.")

        if S["dirty"]:
            S["dirty"] = False
            draw_picture()
        draw_lamps()
        if not S["thread"] and not demo and time.time() - S["last_scan"] > 2.0:
            S["last_scan"] = time.time()
            scan_ports()
        root.after(40, tick)

    def on_close():
        try:
            cfg.update(port=port_var.get(), baud=int(baud_var.get() or 500000),
                       picture=size_var.get())
            save_config(cfg)
        finally:
            stop(message=False)
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    reset_rows()
    scan_ports(force=True)
    draw_picture()
    tick()
    root.mainloop()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    mode = None
    for flag, name in (("--demo", "ok"), ("--demo-fault", "fault"), ("--demo-nocam", "nocam"),
                       ("--demo-i2c", "i2c"), ("--demo-vga", "vga")):
        if flag in sys.argv:
            mode = name
    run_gui(demo=mode)
