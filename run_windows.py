# server.py
import math
import socket
import threading
import time
from collections import deque
from flask import Flask, jsonify, send_from_directory
from pynput import mouse
import ctypes
import ctypes.wintypes as wt

# ── Constants ────────────────────────────────────────────────────────────────
COUNTS_PER_CM = 151
UDP_PORT = 2055
EMA_ALPHA = 0.3          # Exponential moving average weight (0=smooth, 1=raw)
SMOOTHING_WIN = 5            # Rolling window size for additional smoothing
WM_INPUT = 0x00FF
RID_INPUT = 0x10000003
RIDEV_INPUTSINK = 0x00000100   # receive input even when not foreground
HID_USAGE_PAGE_GENERIC = 0x01
HID_USAGE_GENERIC_MOUSE = 0x02
app = Flask(__name__, static_folder=".")

# ── Shared State (protected by a lock) ───────────────────────────────────────
lock = threading.Lock()
state = {
    "x":         0.0,
    "y":         0.0,
    "yaw":       0.0,
    "distance":  0.0,
    "recording": False,
    "path":      [[0.0, 0.0]],   # list of (x, y) waypoints
}

# Internal accumulators (not exposed directly)
_raw_dx_buf = deque(maxlen=SMOOTHING_WIN)   # rolling window for mouse X
_ema_distance = 0.0                            # EMA-smoothed cumulative distance
_click_count = 0                              # for double-click detection
_last_click = 0.0


# ── ctypes structures ─────────────────────────────────────────────────────────
class RAWINPUTDEVICE(ctypes.Structure):
    _fields_ = [
        ("usUsagePage", wt.USHORT),
        ("usUsage",     wt.USHORT),
        ("dwFlags",     wt.DWORD),
        ("hwndTarget",  wt.HWND),
    ]


class RAWINPUTHEADER(ctypes.Structure):
    _fields_ = [
        ("dwType",  wt.DWORD),
        ("dwSize",  wt.DWORD),
        ("hDevice", wt.HANDLE),
        ("wParam",  wt.WPARAM),
    ]


class RAWMOUSE(ctypes.Structure):
    _fields_ = [
        ("usFlags",             wt.USHORT),
        ("usButtonFlags",       wt.USHORT),
        ("usButtonData",        wt.USHORT),
        ("ulRawButtons",        wt.ULONG),
        ("lLastX",              ctypes.c_long),   # ← raw delta X
        ("lLastY",              ctypes.c_long),   # ← raw delta Y
        ("ulExtraInformation",  wt.ULONG),
    ]


class RAWINPUT(ctypes.Structure):
    class _data(ctypes.Union):
        class _mouse_pad(ctypes.Structure):
            _fields_ = [("_pad", ctypes.c_byte * 8)]  # header alignment
        _fields_ = [("mouse", RAWMOUSE)]
    _fields_ = [
        ("header", RAWINPUTHEADER),
        ("data",   _data),
    ]

# ── Raw Input window + pump ───────────────────────────────────────────────────


def _raw_input_delta_callback(raw_dx: int, raw_dy: int):
    """
    Called from the Raw Input pump with true, unbounded mouse deltas.
    Mirrors the logic that was previously inside pynput's on_move.
    """
    global _ema_distance

    # raw_dy: positive = mouse moved down (away from user on most surfaces)
    # We negate so that pushing forward = positive distance (matches original)
    raw_counts = -raw_dy

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


def _raw_input_thread():
    """
    Creates a hidden message-only window, registers for Raw Input,
    and pumps WM_INPUT messages on its own thread.
    """
    user32 = ctypes.windll.user32
    kernel32 = ctypes.windll.kernel32

    # --- register a minimal window class ---
    WNDPROCTYPE = ctypes.WINFUNCTYPE(
        ctypes.c_long, wt.HWND, wt.UINT, wt.WPARAM, wt.LPARAM
    )

    def wnd_proc(hwnd, msg, wparam, lparam):
        if msg == WM_INPUT:
            # Query size first
            size = wt.UINT(0)
            ctypes.windll.user32.GetRawInputData(
                ctypes.cast(lparam, wt.HANDLE),
                RID_INPUT, None, ctypes.byref(size),
                ctypes.sizeof(RAWINPUTHEADER)
            )
            buf = (ctypes.c_byte * size.value)()
            ctypes.windll.user32.GetRawInputData(
                ctypes.cast(lparam, wt.HANDLE),
                RID_INPUT, buf, ctypes.byref(size),
                ctypes.sizeof(RAWINPUTHEADER)
            )
            raw = ctypes.cast(buf, ctypes.POINTER(RAWINPUT)).contents
            if raw.header.dwType == 0:   # RIM_TYPEMOUSE = 0
                dx = raw.data.mouse.lLastX
                dy = raw.data.mouse.lLastY
                if dx != 0 or dy != 0:
                    _raw_input_delta_callback(dx, dy)
            return 0
        return user32.DefWindowProcW(hwnd, msg, wparam, lparam)

    wnd_proc_c = WNDPROCTYPE(wnd_proc)

    class WNDCLASSEX(ctypes.Structure):
        _fields_ = [
            ("cbSize",        wt.UINT),
            ("style",         wt.UINT),
            ("lpfnWndProc",   WNDPROCTYPE),
            ("cbClsExtra",    ctypes.c_int),
            ("cbWndExtra",    ctypes.c_int),
            ("hInstance",     wt.HINSTANCE),
            ("hIcon",         wt.HANDLE),
            ("hCursor",       wt.HANDLE),
            ("hbrBackground", wt.HANDLE),
            ("lpszMenuName",  wt.LPCWSTR),
            ("lpszClassName", wt.LPCWSTR),
            ("hIconSm",       wt.HANDLE),
        ]

    hinstance = kernel32.GetModuleHandleW(None)
    class_name = "RobotRawInputClass"

    wc = WNDCLASSEX()
    wc.cbSize = ctypes.sizeof(WNDCLASSEX)
    wc.lpfnWndProc = wnd_proc_c
    wc.hInstance = hinstance
    wc.lpszClassName = class_name
    user32.RegisterClassExW(ctypes.byref(wc))

    # HWND_MESSAGE = -3  →  message-only window (invisible, no taskbar entry)
    hwnd = user32.CreateWindowExW(
        0, class_name, "RawInputSink", 0,
        0, 0, 0, 0,
        ctypes.cast(-3, wt.HWND),   # HWND_MESSAGE
        None, hinstance, None
    )

    # --- register mouse for Raw Input ---
    rid = RAWINPUTDEVICE()
    rid.usUsagePage = HID_USAGE_PAGE_GENERIC
    rid.usUsage = HID_USAGE_GENERIC_MOUSE
    rid.dwFlags = RIDEV_INPUTSINK   # receive even when not foreground
    rid.hwndTarget = hwnd
    user32.RegisterRawInputDevices(
        ctypes.byref(rid), 1, ctypes.sizeof(RAWINPUTDEVICE)
    )
    print("[RawInput] Registered, pumping messages…")

    # --- message pump ---
    msg = ctypes.wintypes.MSG()
    while user32.GetMessageW(ctypes.byref(msg), None, 0, 0) != 0:
        user32.TranslateMessage(ctypes.byref(msg))
        user32.DispatchMessageW(ctypes.byref(msg))


def start_mouse_listener():
    """
    Starts:
      1. Raw Input thread  — for unbounded movement deltas (odometry)
      2. pynput listener   — ONLY for click detection (start/stop/reset)
    """
    # Raw Input for movement
    t = threading.Thread(target=_raw_input_thread, daemon=True)
    t.start()

    # pynput only for clicks — on_move is intentionally omitted
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
# ── Yaw helpers ──────────────────────────────────────────────────────────────


def quaternion_to_yaw(x, y, z):
    w = math.sqrt(max(0.0, 1.0 - x*x - y*y - z*z))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )
    return math.degrees(yaw)


def angle_diff(a, b):
    """Shortest signed difference between two angles (degrees)."""
    d = (a - b + 180) % 360 - 180
    return d

# ── UDP Listener Thread ───────────────────────────────────────────────────────


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

            # Smooth yaw with EMA, handling wraparound
            if smoothed_yaw is None:
                smoothed_yaw = raw_yaw
            else:
                diff = angle_diff(raw_yaw, smoothed_yaw)
                smoothed_yaw = smoothed_yaw + EMA_ALPHA * diff
                smoothed_yaw = (smoothed_yaw + 180) % 360 - \
                    180   # keep in [-180,180]

            with lock:
                state["yaw"] = round(smoothed_yaw, 2)

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] Error: {e}")

# ── Mouse Listener ────────────────────────────────────────────────────────────


def on_move(x_abs, y_abs):
    """pynput calls this with ABSOLUTE coords; we need deltas."""
    pass   # handled via on_move_delta below


_last_mouse_pos = [None, None]

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
            "path":      state["path"][-500:],   # last 500 points max
        })


# ── Entry Point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Start background threads BEFORE Flask (use_reloader=False is critical)
    t_udp = threading.Thread(target=udp_listener, daemon=True)
    t_udp.start()

    start_mouse_listener()

    # threaded=True allows concurrent /state polling
    app.run(host="0.0.0.0", port=5000, debug=False,
            use_reloader=False, threaded=True)
