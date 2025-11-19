import socket
import threading
import ast
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# === CONFIG: must match the URScript ===
HOST = "0.0.0.0"   # listen on all interfaces
PORT = 50010      # same as pc_port in URScript
N_JOINTS = 6
WINDOW_SIZE = 5000  # number of samples shown in the plot


def tcp_server(data_buffer):
    """TCP server that receives torque lines from the robot and stores them."""
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

                buffer += chunk

                # We consider each torque sample ends with ']'
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
                        continue
                    line_str = line_str[idx:]

                    if not line_str:
                        continue

                    try:
                        torques = ast.literal_eval(line_str)
                        if isinstance(torques, (list, tuple)) and len(torques) == N_JOINTS:
                            print("[DEBUG] Parsed torques:", torques)
                            for j in range(N_JOINTS):
                                data_buffer[j].append(float(torques[j]))
                        else:
                            print(f"[WARN] Unexpected data format: {line_str}")
                    except Exception as e:
                        print(f"[WARN] Failed to parse line '{line_str}': {e}")





def main():
    # One deque per joint, fixed length for sliding window
    data_buffer = [deque([0.0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
                   for _ in range(N_JOINTS)]

    # Start TCP server in background thread
    server_thread = threading.Thread(target=tcp_server, args=(data_buffer,),
                                     daemon=True)
    server_thread.start()

    # --- Matplotlib live plot ---
    fig, ax = plt.subplots()
    lines = []

    for j in range(N_JOINTS):
        (line,) = ax.plot(range(WINDOW_SIZE),
                          list(data_buffer[j]),
                          label=f"Joint {j+1}")
        lines.append(line)

    ax.set_xlabel("Sample")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("UR Robot Joint Torques (Realtime)")
    ax.legend(loc="upper right")
    ax.grid(True)

    def update(frame):
        # Update line data from buffer
        for j in range(N_JOINTS):
            lines[j].set_ydata(list(data_buffer[j]))
        # y-limits auto scale based on current data
        all_vals = [val for dq in data_buffer for val in dq]
        if all_vals:
            ymin = min(all_vals)
            ymax = max(all_vals)
            margin = max(0.1 * (ymax - ymin), 0.1)
            ax.set_ylim(ymin - margin, ymax + margin)
        return lines

    # Keep ani in global scope so it isn't garbage-collected
    global ani
    ani = animation.FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    print("[INFO] Plot window ready. Start the URScript program now.")
    plt.show()   # blocking call


if __name__ == "__main__":
    main()
