import socket
import time
import cv2
import threading

# IP addresses of network adapters
LOCAL_IP_1 = "192.168.10.3"  # Leader
LOCAL_IP_2 = "192.168.10.2"  # Follower 1
LOCAL_IP_3 = "192.168.10.5"  # Follower 2
LOCAL_IP_4 = "192.168.10.7"  # Follower 3

TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889
TELLO_VIDEO_PORT = 11111  # Port for video stream

# Create UDP sockets
sock_leader = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_leader.bind((LOCAL_IP_1, TELLO_PORT))

sock_follower_1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_1.bind((LOCAL_IP_2, TELLO_PORT))

sock_follower_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_2.bind((LOCAL_IP_3, TELLO_PORT))

sock_follower_3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_3.bind((LOCAL_IP_4, TELLO_PORT))


# Function to send commands to a drone
def send_command(sock, command):
    sock.sendto(command.encode(), (TELLO_IP, TELLO_PORT))
    print(f"Sent command: {command}")


# Function to mimic leader's actions
def mimic_leader(command, delay=0.5):
    time.sleep(delay)  # Small delay to mimic leader’s action
    send_command(sock_follower_1, command)
    send_command(sock_follower_2, command)
    send_command(sock_follower_3, command)


# Function to display camera feed from the leader drone
def stream_video():
    cap = cv2.VideoCapture(f"udp://@0.0.0.0:{TELLO_VIDEO_PORT}")
    if not cap.isOpened():
        print("Failed to open video stream")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to receive video frame")
            break

        # Display the video feed
        cv2.imshow("Tello Leader Camera", frame)

        # Exit the window with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# Start command mode for all drones
send_command(sock_leader, "command")
send_command(sock_follower_1, "command")
send_command(sock_follower_2, "command")
send_command(sock_follower_3, "command")
time.sleep(2)

# Turn on camera on the leader drone only
send_command(sock_leader, "streamon")
time.sleep(2)

# Start video stream in a separate thread (to avoid blocking flight commands)
video_thread = threading.Thread(target=stream_video)
video_thread.start()

# Take off leader first, then followers mimic it
send_command(sock_leader, "takeoff")
mimic_leader("takeoff", delay=0.5)
time.sleep(8)

# Leader drone flies up, followers mimic
send_command(sock_leader, "up 100")
mimic_leader("up 100", delay=0.5)
time.sleep(5)

# Leader moves forward, followers mimic
send_command(sock_leader, "forward 100")
mimic_leader("forward 100", delay=0.5)
time.sleep(5)

# Leader rotates, followers mimic
send_command(sock_leader, "cw 180")
mimic_leader("cw 180", delay=0.5)
time.sleep(5)

# Leader moves forward again, followers mimic
send_command(sock_leader, "forward 100")
mimic_leader("forward 100", delay=0.5)
time.sleep(5)

# Leader rotates back, followers mimic
send_command(sock_leader, "cw 180")
mimic_leader("cw 180", delay=0.5)
time.sleep(5)

# Leader lands, followers mimic
send_command(sock_leader, "land")
mimic_leader("land", delay=0.5)
time.sleep(5)

# Turn off camera stream on leader after landing
send_command(sock_leader, "streamoff")

# Close sockets
sock_leader.close()
sock_follower_1.close()
sock_follower_2.close()
sock_follower_3.close()

# Stop video stream
video_thread.join()
