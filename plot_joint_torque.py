import socket
import threading
import ast
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === CONFIG: must match the URScript ===
HOST = "0.0.0.0"   # listen on all interfaces
PORT = 50010       # same as pc_port in URScript

N_DIM = 12         # 6 joint torques + 6 wrist FT
WINDOW_SIZE = 5000  # number of samples shown in the plot

# Labels: first 6 are joint torques, last 6 are wrist wrench
JOINT_LABELS = [f"J{i+1} torque (Nm)" for i in range(6)]
WRIST_LABELS = ["Fx (N)", "Fy (N)", "Fz (N)", "Tx (Nm)", "Ty (Nm)", "Tz (Nm)"]


def tcp_server(data_buffer):
    """TCP server that receives lines from the robot and stores them."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)

        print(f"[INFO] Waiting for UR robot connection on {HOST}:{PORT} ...")
        conn, addr = s.accept()
        print(f"[INFO] Robot connected from {addr}")

        with conn:
            buffer = b""
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    print("[INFO] Connection closed by robot")
                    break

                print(f"[DEBUG] Received raw bytes: {len(chunk)}")
                buffer += chunk

                # We consider each sample ends with ']'
                while b"]" in buffer:
                    # Take everything up to and including the first ']'
                    line, buffer = buffer.split(b"]", 1)
                    line += b"]"  # put the ']' back

                    line_str = line.decode("utf-8", errors="ignore")
                    # Strip whitespace/newlines
                    line_str = line_str.strip()

                    # Remove any junk before the first '[' (e.g. leading '\n')
                    idx = line_str.find("[")
                    if idx == -1:
                        print(f"[WARN] No '[' found in line: {repr(line_str)}")
                        continue
                    line_str = line_str[idx:]

                    if not line_str:
                        continue

                    print(f"[DEBUG] Line from robot (clean): {line_str}")

                    try:
                        values = ast.literal_eval(line_str)
                        if isinstance(values, (list, tuple)) and len(values) == N_DIM:
                            print("[DEBUG] Parsed values:", values)
                            for i in range(N_DIM):
                                data_buffer[i].append(float(values[i]))
                        else:
                            print(f"[WARN] Unexpected data length ({len(values)}): {line_str}")
                    except Exception as e:
                        print(f"[WARN] Failed to parse line '{line_str}': {e}")


def main():
    # One deque per dimension, fixed length for sliding window
    data_buffer = [deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
                   for _ in range(N_DIM)]

    # Start TCP server in background thread
    server_thread = threading.Thread(target=tcp_server, args=(data_buffer,),
                                     daemon=True)
    server_thread.start()

    # --- Matplotlib live plot: 2 subplots (joints on top, wrist on bottom) ---
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    fig.suptitle("UR Joint Torques and Wrist Force/Torque (Realtime)")

    joint_lines = []
    wrist_lines = []

    # Top: joint torques
    for j in range(6):
        (line,) = ax1.plot(range(WINDOW_SIZE),
                           list(data_buffer[j]),
                           label=JOINT_LABELS[j])
        joint_lines.append(line)

    ax1.set_ylabel("Torque (Nm)")
    ax1.legend(loc="upper right")
    ax1.grid(True)

    # Bottom: wrist FT
    for j in range(6):
        idx = 6 + j
        (line,) = ax2.plot(range(WINDOW_SIZE),
                           list(data_buffer[idx]),
                           label=WRIST_LABELS[j])
        wrist_lines.append(line)

    ax2.set_xlabel("Sample")
    ax2.set_ylabel("Force / Torque")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    def update(frame):
        # Update joint torque curves
        for j in range(6):
            joint_lines[j].set_ydata(list(data_buffer[j]))

        # Update wrist FT curves
        for j in range(6):
            idx = 6 + j
            wrist_lines[j].set_ydata(list(data_buffer[idx]))

        # Auto-scale Y for each subplot separately
        joint_vals = [v for dq in data_buffer[:6] for v in dq]
        wrist_vals = [v for dq in data_buffer[6:] for v in dq]

        if joint_vals:
            ymin = min(joint_vals)
            ymax = max(joint_vals)
            margin = max(0.1 * (ymax - ymin), 0.1)
            ax1.set_ylim(ymin - margin, ymax + margin)

        if wrist_vals:
            ymin = min(wrist_vals)
            ymax = max(wrist_vals)
            margin = max(0.1 * (ymax - ymin), 0.1)
            ax2.set_ylim(ymin - margin, ymax + margin)

        return joint_lines + wrist_lines

    # Keep ani in global scope so it's not garbage-collected
    global ani
    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    print("[INFO] Plot window ready. Start the URScript program now.")
    plt.show()  # blocking call


if __name__ == "__main__":
    main()
