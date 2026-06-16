import socket
import math

HOST = "0.0.0.0"
PORT = 2055

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"Listening on UDP {PORT}")

last_yaw = None

TURN_THRESHOLD = 2.0  # degrees

while True:
    data, addr = sock.recvfrom(4096)

    try:
        text = data.decode(errors="ignore").strip()

        x, y, z = map(float, text.split(","))

        # Reconstruct quaternion W
        w_sq = 1.0 - (x * x + y * y + z * z)
        w = math.sqrt(max(0.0, w_sq))

        # Quaternion -> Yaw
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

        yaw_deg = math.degrees(yaw)

        if last_yaw is not None:

            delta = yaw_deg - last_yaw

            # Handle wraparound
            if delta > 180:
                delta -= 360

            if delta < -180:
                delta += 360

            if delta > TURN_THRESHOLD:
                direction = "TURN LEFT"

            elif delta < -TURN_THRESHOLD:
                direction = "TURN RIGHT"

            else:
                direction = "STRAIGHT"

        else:
            direction = "INIT"

        last_yaw = yaw_deg

        print(
            f"Yaw: {yaw_deg:8.2f}° | "
            f"{direction}"
        )

    except Exception as e:
        print("Parse Error:", e)
        print(data)
