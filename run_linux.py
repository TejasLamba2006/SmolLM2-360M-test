# server.py  ── Linux / OpenSTLinux version (STM32MP257F-DK)
import math
import os
import socket
import struct
import threading
import time
from collections import deque
from flask import Flask, jsonify, send_from_directory
from pynput import mouse

# ── Constants ─────────────────────────────────────────────────────────────────
COUNTS_PER_CM = 151
UDP_PORT = 2055
EMA_ALPHA = 0.3
SMOOTHING_WIN = 5

# Linux input_event layout (64-bit kernel, ARM64)
# struct input_event { timeval(8+8), type(2), code(2), value(4) } = 24 bytes
INPUT_EVENT_FMT = "llHHi"
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FMT)

# /linux/input-event-codes.h
EV_SYN, EV_REL = 0x00, 0x02
REL_X,  REL_Y = 0x00, 0x01

app = Flask(__name__, static_folder=".")

# ── Shared State ──────────────────────────────────────────────────────────────
lock = threading.Lock()
state = {
    "x":         0.0,
    "y":         0.0,
    "yaw":       0.0,
    "distance":  0.0,
    "recording": False,
    "path":      [[0.0, 0.0]],
}

_raw_dx_buf = deque(maxlen=SMOOTHING_WIN)
_ema_distance = 0.0
_click_count = 0
_last_click = 0.0

# ── Yaw helpers ───────────────────────────────────────────────────────────────


def quaternion_to_yaw(x, y, z):
    w = math.sqrt(max(0.0, 1.0 - x*x - y*y - z*z))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )
    return math.degrees(yaw)


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180

# ── UDP Listener ──────────────────────────────────────────────────────────────


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening on port {UDP_PORT}")

    smoothed_yaw = None
    while True:
        try:
            data, _ = sock.recvfrom(256)
            parts = data.decode().strip().split(",")
            if len(parts) < 3:
                continue
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            raw_yaw = quaternion_to_yaw(x, y, z)

            if smoothed_yaw is None:
                smoothed_yaw = raw_yaw
            else:
                diff = angle_diff(raw_yaw, smoothed_yaw)
                smoothed_yaw = smoothed_yaw + EMA_ALPHA * diff
                smoothed_yaw = (smoothed_yaw + 180) % 360 - 180

            with lock:
                state["yaw"] = round(smoothed_yaw, 2)

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] Error: {e}")

# ── Linux evdev mouse reader ──────────────────────────────────────────────────


def _find_mouse_event_device() -> str:
    """
    Scan /proc/bus/input/devices and return the /dev/input/eventX path
    for the first device that reports EV_REL (relative axes = mouse).
    Falls back to /dev/input/mice if nothing is found.
    """
    try:
        with open("/proc/bus/input/devices") as f:
            content = f.read()

        # Each device block is separated by a blank line
        for block in content.split("\n\n"):
            # Handlers line looks like:  H: Handlers=mouse0 event3 ...
            # Bitmap line looks like:    B: EV=17  (bit 1 = EV_KEY, bit 2 = EV_REL)
            has_rel = False
            event_node = None
            for line in block.splitlines():
                if line.startswith("B: EV="):
                    ev_bits = int(line.split("=")[1], 16)
                    has_rel = bool(ev_bits & (1 << EV_REL))   # bit 2
                if line.startswith("H: Handlers="):
                    for token in line.split():
                        if token.startswith("event"):
                            event_node = f"/dev/input/{token}"
            if has_rel and event_node:
                print(f"[evdev] Auto-detected mouse: {event_node}")
                return event_node
    except Exception as e:
        print(f"[evdev] Device scan failed: {e}")

    print("[evdev] Falling back to /dev/input/mice")
    return "/dev/input/mice"


def _raw_input_delta_callback(raw_dx: int, raw_dy: int):
    """Shared odometry update — identical logic to the Windows version."""
    global _ema_distance

    raw_counts = -raw_dy   # push forward = positive distance
    _raw_dx_buf.append(raw_counts)
    smoothed_counts = sum(_raw_dx_buf) / len(_raw_dx_buf)

    _ema_distance = (1 - EMA_ALPHA) * _ema_distance + \
        EMA_ALPHA * smoothed_counts
    dist_cm = _ema_distance / COUNTS_PER_CM

    with lock:
        if state["recording"]:
            yaw_rad = math.radians(state["yaw"])
            state["x"] += dist_cm * math.cos(yaw_rad)
            state["y"] += dist_cm * math.sin(yaw_rad)
            state["distance"] += dist_cm
            path = state["path"]
            if not path or math.hypot(
                state["x"] - path[-1][0],
                state["y"] - path[-1][1]
            ) > 0.5:
                path.append([round(state["x"], 2), round(state["y"], 2)])


def _evdev_thread(device_path: str):
    """
    Read raw input_event structs from the evdev node.
    Accumulates REL_X / REL_Y within each EV_SYN frame, then fires
    the callback once per sync — exactly like a Raw Input report.
    """
    print(f"[evdev] Opening {device_path}")
    try:
        fd = open(device_path, "rb")
    except PermissionError:
        print(
            f"[evdev] Permission denied on {device_path}.\n"
            f"        Fix with:  sudo usermod -aG input $USER\n"
            f"        or run:    sudo chmod a+r {device_path}"
        )
        return
    except FileNotFoundError:
        print(f"[evdev] Device not found: {device_path}")
        return

    pending_dx = 0
    pending_dy = 0

    while True:
        try:
            raw = fd.read(INPUT_EVENT_SIZE)
            if len(raw) < INPUT_EVENT_SIZE:
                # Device disconnected or short read
                print("[evdev] Short read — device disconnected?")
                time.sleep(1.0)
                continue

            _, _, ev_type, ev_code, ev_value = struct.unpack(
                INPUT_EVENT_FMT, raw)

            if ev_type == EV_REL:
                if ev_code == REL_X:
                    pending_dx += ev_value
                elif ev_code == REL_Y:
                    pending_dy += ev_value

            elif ev_type == EV_SYN:
                # SYN_REPORT (code 0) marks end of one logical event frame
                if (pending_dx != 0 or pending_dy != 0):
                    _raw_input_delta_callback(pending_dx, pending_dy)
                pending_dx = 0
                pending_dy = 0

        except Exception as e:
            print(f"[evdev] Error: {e}")
            time.sleep(0.1)


# ── Mouse click listener (pynput — start / stop / reset) ─────────────────────
def start_mouse_listener():
    device_path = _find_mouse_event_device()

    # evdev thread handles movement deltas
    t = threading.Thread(target=_evdev_thread,
                         args=(device_path,), daemon=True)
    t.start()

    # pynput handles clicks only
    def on_click(x, y, button, pressed):
        global _click_count, _last_click
        if button != mouse.Button.left:
            return

        now = time.time()
        if pressed:
            if now - _last_click < 0.4:
                _click_count += 1
            else:
                _click_count = 1
            _last_click = now

            if _click_count >= 2:
                _click_count = 0
                with lock:
                    state["x"] = 0.0
                    state["y"] = 0.0
                    state["distance"] = 0.0
                    state["recording"] = False
                    state["path"] = [[0.0, 0.0]]
                print("[Mouse] Map reset")
                return

            with lock:
                state["recording"] = True
            print("[Mouse] Recording started")
        else:
            with lock:
                state["recording"] = False
            print("[Mouse] Recording stopped")

    listener = mouse.Listener(on_click=on_click, suppress=False)
    listener.daemon = True
    listener.start()
    print("[Mouse] Click listener started")

# ── Flask Routes ──────────────────────────────────────────────────────────────


@app.route("/")
def index():
    return send_from_directory(".", "index.html")


@app.route("/state")
def get_state():
    with lock:
        return jsonify({
            "x":         round(state["x"], 2),
            "y":         round(state["y"], 2),
            "yaw":       state["yaw"],
            "distance":  round(state["distance"], 2),
            "recording": state["recording"],
            "path":      state["path"][-500:],
        })


# ── Entry Point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    t_udp = threading.Thread(target=udp_listener, daemon=True)
    t_udp.start()

    start_mouse_listener()

    app.run(host="0.0.0.0", port=5000, debug=False,
            use_reloader=False, threaded=True)
