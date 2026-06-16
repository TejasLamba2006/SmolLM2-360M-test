import sys
import math
from datetime import datetime
from pynput import mouse

if len(sys.argv) != 2:
    print("Usage: python calibrate.py <distance_cm>")
    sys.exit(1)

DISTANCE_CM = float(sys.argv[1])

recording = False

total_dx = 0
total_dy = 0

last_x = None
last_y = None


def on_move(x, y):
    global last_x, last_y
    global total_dx, total_dy

    if last_x is None:
        last_x = x
        last_y = y
        return

    dx = x - last_x
    dy = y - last_y

    last_x = x
    last_y = y

    if recording:
        total_dx += dx
        total_dy += dy


def on_click(x, y, button, pressed):
    global recording
    global total_dx
    global total_dy

    if button != mouse.Button.left:
        return

    if pressed:
        total_dx = 0
        total_dy = 0
        recording = True

        print("\nRecording...")
        print(f"Move exactly {DISTANCE_CM} cm")
        print("Release left click when done.\n")

    else:
        recording = False

        counts = math.sqrt(
            total_dx ** 2 +
            total_dy ** 2
        )

        counts_per_cm = counts / DISTANCE_CM

        timestamp = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        result = (
            f"\n[{timestamp}]\n"
            f"Distance(cm): {DISTANCE_CM}\n"
            f"DX: {total_dx}\n"
            f"DY: {total_dy}\n"
            f"Counts: {counts:.2f}\n"
            f"CountsPerCM: {counts_per_cm:.4f}\n"
            f"{'-' * 40}\n"
        )

        print(result)

        with open(
            "calibration_results.txt",
            "a",
            encoding="utf-8"
        ) as f:
            f.write(result)

        print("Saved to calibration_results.txt")


print("===================================")
print(f"Calibration Distance: {DISTANCE_CM} cm")
print("Hold LEFT CLICK to record")
print("Release LEFT CLICK to save")
print("===================================")

with mouse.Listener(
    on_move=on_move,
    on_click=on_click
) as listener:
    listener.join()
