import socket
import time
import cv2
import threading
import csv

# Adapter IPs
LOCAL_IP_1 = "192.168.10.3"  # Leader
LOCAL_IP_2 = "192.168.10.2"  # Follower 1
LOCAL_IP_3 = "192.168.10.5"  # Follower 2
LOCAL_IP_4 = "192.168.10.7"  # Follower 3

# Drone IP & Ports
TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889
TELLO_VIDEO_PORT = 11111

# Create sockets
sock_leader = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_leader.bind((LOCAL_IP_1, TELLO_PORT))

sock_follower_1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_1.bind((LOCAL_IP_2, TELLO_PORT))

sock_follower_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_2.bind((LOCAL_IP_3, TELLO_PORT))

sock_follower_3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_3.bind((LOCAL_IP_4, TELLO_PORT))

# Enable logging
log_data = []

def log_event(drone_label, adapter_ip, command, send_time, response_time=None):
    log_data.append({
        "drone": drone_label,
        "adapter": adapter_ip,
        "command": command,
        "send_time": send_time,
        "response_time": response_time,
        "RTT": response_time - send_time if response_time else None
    })

# Send command and log timing
def send_command(sock, command, drone_label="Unknown", adapter_ip="Unknown"):
    send_time = time.time()
    try:
        sock.sendto(command.encode(), (TELLO_IP, TELLO_PORT))
        print(f"[{drone_label}] Sent command: {command}")

        time.sleep(0.1)  # Simulated response wait
        response_time = time.time()
        log_event(drone_label, adapter_ip, command, send_time, response_time)

    except Exception as e:
        print(f"Error: {e}")
        log_event(drone_label, adapter_ip, command, send_time, None)

# Followers mimic leader's commands
def mimic_leader(command, delay=0.5):
    time.sleep(delay)
    send_command(sock_follower_1, command, "Follower 1", LOCAL_IP_2)
    send_command(sock_follower_2, command, "Follower 2", LOCAL_IP_3)
    send_command(sock_follower_3, command, "Follower 3", LOCAL_IP_4)

# Video stream (leader only)
def stream_video():
    cap = cv2.VideoCapture(f"udp://@0.0.0.0:{TELLO_VIDEO_PORT}")
    if not cap.isOpened():
        print("Video stream failed")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No video frame")
            break
        cv2.imshow("Leader Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# === Start Flight ===

send_command(sock_leader, "command", "Leader", LOCAL_IP_1)
send_command(sock_follower_1, "command", "Follower 1", LOCAL_IP_2)
send_command(sock_follower_2, "command", "Follower 2", LOCAL_IP_3)
send_command(sock_follower_3, "command", "Follower 3", LOCAL_IP_4)
time.sleep(2)

send_command(sock_leader, "streamon", "Leader", LOCAL_IP_1)
time.sleep(2)

video_thread = threading.Thread(target=stream_video)
video_thread.start()

send_command(sock_leader, "takeoff", "Leader", LOCAL_IP_1)
mimic_leader("takeoff")
time.sleep(8)

send_command(sock_leader, "up 100", "Leader", LOCAL_IP_1)
mimic_leader("up 100")
time.sleep(5)

send_command(sock_leader, "forward 100", "Leader", LOCAL_IP_1)
mimic_leader("forward 100")
time.sleep(5)

send_command(sock_leader, "cw 180", "Leader", LOCAL_IP_1)
mimic_leader("cw 180")
time.sleep(5)

send_command(sock_leader, "forward 100", "Leader", LOCAL_IP_1)
mimic_leader("forward 100")
time.sleep(5)

send_command(sock_leader, "cw 180", "Leader", LOCAL_IP_1)
mimic_leader("cw 180")
time.sleep(5)

send_command(sock_leader, "land", "Leader", LOCAL_IP_1)
mimic_leader("land")
time.sleep(5)

send_command(sock_leader, "streamoff", "Leader", LOCAL_IP_1)

# Close sockets
sock_leader.close()
sock_follower_1.close()
sock_follower_2.close()
sock_follower_3.close()

# Save CSV
with open("swarm_command_log.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["drone", "adapter", "command", "send_time", "response_time", "RTT"])
    writer.writeheader()
    for entry in log_data:
        writer.writerow(entry)

print("✅ Log saved to 'swarm_command_log.csv'")

video_thread.join()
