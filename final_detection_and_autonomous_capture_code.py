from djitellopy import Tello
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import time
import socket
from collections import deque
from queue import Queue, Empty
import statistics
import math

# === CONFIGURATION ===
KNOWN_WIDTH = 0.12 # Width of target object in metres
FOCAL_LENGTH = 640
REAL_TOLERANCE = 0.3
PERSISTENCE_FRAMES = 10

# === FOLLOWER SETUP ===
TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889
LOCAL_IP_1 = "192.168.10.2"
LOCAL_IP_2 = "192.168.10.3"
LOCAL_IP_3 = "192.168.10.4"
LOCAL_IP_4 = "192.168.10.5"

sock_follower_1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_1.bind((LOCAL_IP_2, TELLO_PORT))
sock_follower_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_2.bind((LOCAL_IP_3, TELLO_PORT))
sock_follower_3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_follower_3.bind((LOCAL_IP_4, TELLO_PORT))

follower_socks = [sock_follower_1, sock_follower_2, sock_follower_3]

def initialise_followers():
    print("Initialising followers...")
    for _ in range(1):
        for sock in follower_socks:
            sock.sendto(b"command", (TELLO_IP, TELLO_PORT))
        time.sleep(0.3)

CRITICAL_COMMANDS = ["command", "takeoff", "land", "cw", "ccw","forward", "up"]

def mimic_leader(command, delay=0.5):
    retries = 3 if any(c in command for c in CRITICAL_COMMANDS) else 1
    time.sleep(delay)
    for _ in range(retries):
        for sock in follower_socks:
            sock.sendto(command.encode(), (TELLO_IP, TELLO_PORT))
            print(f"Follower sent: {command}")
        time.sleep(0.2)

# === LEADER SETUP ===
tello = Tello()
frame_queue = Queue(maxsize=5)
last_seen_boxes = deque(maxlen=PERSISTENCE_FRAMES)
distance_history = deque(maxlen=5)
object_detected = False
feed_ready = False
lost_frames = 0
scan_ready = False

model = YOLO(r"...")  # Change path to saved .pt file

# === FUNCTIONS ===
def calculate_distance(known_width, focal_length, object_width_in_pixels):
    if object_width_in_pixels == 0:
        return None
    return (known_width * focal_length) / object_width_in_pixels

def send_movement(command):
    tello.send_command_without_return(command)
    mimic_leader(command)

def capture_frames():
    global feed_ready
    while True:
        frame = tello.get_frame_read().frame
        if frame is None:
            continue
        frame = cv2.resize(frame, (640, 480))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if not frame_queue.full():
            frame_queue.put(frame)
        feed_ready = True
        time.sleep(0.05)

def process_video():
    global last_seen_boxes, lost_frames, object_detected
    while True:
        try:
            frame = frame_queue.get(timeout=1)
        except Empty:
            continue

        if frame is None or frame.size == 0:
            print("[WARNING] Skipped empty frame.")
            continue

        results = model(frame, conf=0.6)
        best_box = None
        best_confidence = 0.0
        for result in results:
            for box in result.boxes:
                confidence = box.conf[0].item()
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_box = box.xyxy[0].tolist()

        if best_box:
            x1, y1, x2, y2 = map(int, best_box)
            width_in_pixels = x2 - x1
            distance = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, width_in_pixels)
            distance_text = f"Distance: {distance:.2f} m" if distance else "Distance: N/A"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, distance_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            last_seen_boxes.append((x1, y1, x2, y2))
            object_detected = True
            lost_frames = 0
        else:
            object_detected = False
            lost_frames += 1

        cv2.imshow("Leader Drone Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def control_movement():
    global lost_frames, object_detected, feed_ready
    scanning = False
    move_up_sent = move_forward_sent = landed = False
    capture_mode = False
    stable_frames = 0
    movement_active = False
    x_tolerance_locked = False
    frames_since_last_rotation = 0
    centre_x = 0 # Position of centre of target in live feed
    dx = 0.75  # Centre of net at takeoff is 75cm to right of leader, change if net altered
    dy = -0.75  # Centre of net at takeoff is 75cm behind the leader, change at net altered

    while not feed_ready:
        time.sleep(0.1)

    while True:
        # === If object is lost (but not in capture) ===
        if not object_detected and not capture_mode:
            stable_frames = 0
            lost_frames += 1
            frames_since_last_rotation += 1
            print("Lost frames:", lost_frames)

            if lost_frames >= 30:
                if not scanning:
                    print("Lost target. Starting scan.")

                    # Decide scan direction based on where it was last seen
                    if 'last_seen_centre_x' in locals():
                        if last_seen_centre_x < 320:
                            scan_direction = "ccw"
                        else:
                            scan_direction = "cw"
                        print(f"Scanning direction set to: {scan_direction}")
                    else:
                        scan_direction = "cw"  # Change depending on desired first rotation direction

                    scanning = True
                    frames_since_last_rotation = 0

                # Rotate every 30 lost frames
                if frames_since_last_rotation >= 30:
                    print("Scanning... rotating 60 degrees")
                    if scan_direction == "cw":
                        tello.rotate_clockwise(60)
                        mimic_leader("cw 60")
                    else:
                        tello.rotate_counter_clockwise(60)
                        mimic_leader("ccw 60")

                    frames_since_last_rotation = 0 # Reset to allow for sufficient time at new yaw

            time.sleep(0.1)
            continue

        # === If object is detected ===
        if object_detected and not capture_mode:
            if scanning:
                print("Target reacquired. Stopping scan.")
                scanning = False
                movement_active = False
                stable_frames = 0
                frames_since_last_rotation = 0
                lost_frames = 0

            if not last_seen_boxes:
                time.sleep(0.05)
                continue

            # Use latest bounding box
            x1, y1, x2, y2 = last_seen_boxes[-1]
            centre_x = (x1 + x2) / 2
            frame_centre_x = FOCAL_LENGTH / 2

            box_width = x2 - x1
            if box_width > 0:
                dist = calculate_distance(KNOWN_WIDTH, FOCAL_LENGTH, box_width)
                if dist and dist < 5:  # Ensures value is realistic
                    distance_history.append(dist)

            x_error = centre_x - frame_centre_x

            if not x_tolerance_locked and len(distance_history) >= 3: # Waits for distance_history to have 3 frames
                avg_dist = statistics.median(distance_history)
                x_tolerance = int((REAL_TOLERANCE * FOCAL_LENGTH) / avg_dist)
                x_tolerance_locked = True # Prevents constant recalculation of x_tolerance

            if not x_tolerance_locked and len(distance_history) < 3:
                x_tolerance = 50

            # Proportional rotation
            if abs(x_error) > x_tolerance:
                rotate_speed = int(np.clip(x_error * 0.08, -20, 20))
                send_movement(f"rc 0 0 0 {rotate_speed}")
                movement_active = True
                stable_frames = 0
            else:
                send_movement("rc 0 0 0 0")
                if not movement_active:
                    stable_frames += 1
                    print(f"[INFO] Stable Frames: {stable_frames}/1")
                    if stable_frames >= 1:
                        print("Target centred for 3 frames. Initiating capture mode.")
                        time.sleep(1)
                        capture_mode = True
                        time.sleep(1)
                else:
                    movement_active = False


# === CAPTURE MODE ===
        if capture_mode and not landed:
            print("[CAPTURE MODE] Running simplified logic with updated movement")

            # === Move forward using last known distance ===
            if not move_forward_sent and len(last_seen_boxes) > 0:
                if box_width > 0:
                        # Compute yaw
                    yaw_deg = tello.get_yaw()
                    yaw_rad = math.radians(yaw_deg)

                    net_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad) # Calculate relative x and y net disp.
                    net_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)
                    print(f"Yaw is {yaw_deg}, Net x-coordinate: {net_x}, Net y-coordinate: {net_y}")

                    forward_offset = max(int((avg_dist - net_y) * 100), 30) # Calculate the necessary forward movement
                    forward_offset = int(np.clip(forward_offset, 20, 500))

                    yaw_adjustment_rad = math.atan2(-net_x, forward_offset)
                    yaw_adjustment_deg = int(math.degrees(yaw_adjustment_rad))
                    yaw_adjustment_deg = int(np.clip(yaw_adjustment_deg, -30, 30))

                    if abs(yaw_adjustment_deg) > 4: #Skip yaw adjustment if value is negligible
                        if yaw_adjustment_deg > 0:
                            tello.rotate_clockwise(yaw_adjustment_deg)
                            mimic_leader(f"cw {yaw_adjustment_deg}")
                            print(f"Rotating clockwise: {yaw_adjustment_deg} degrees")
                        else:
                            yaw_adjustment_deg = 0 - yaw_adjustment_deg
                            tello.rotate_counter_clockwise(yaw_adjustment_deg)
                            mimic_leader(f"ccw {yaw_adjustment_deg}")
                            print(f"Rotating anti-clockwise: {yaw_adjustment_deg} degrees")
                        time.sleep(4)
                        # Move along hypotenuse
                        move_dist = math.sqrt(net_x ** 2 + forward_offset ** 2)
                        move_dist = int(np.clip(move_dist * 100, 20, 500))
                    else:
                        move_dist = forward_offset

                    print(f"Moving forward {move_dist} cm")
                    tello.move_forward(move_dist)
                    mimic_leader(f"forward {move_dist}")
                    time.sleep(5)
                    move_forward_sent = True
                else:
                    print("[SKIP] Invalid bounding box width")

            # === Move back up ===
            if move_forward_sent and not move_up_sent:
                move_up_height = 200
                print(f"Moving up {move_up_height} cm")
                tello.move_up(move_up_height)
                mimic_leader(f"up {move_up_height}")
                time.sleep(5)
                move_up_sent = True

            # === Land ===
            if move_up_sent and not landed:
                print("Landing now...")
                tello.land()
                mimic_leader("land")
                landed = True
                time.sleep(5)
                tello.end()

def keep_alive():
    while True:
        try:
            tello.send_command_without_return("command")
        except Exception as e:
            print("Keep-alive failed:", e)
        time.sleep(3)

# === MAIN RUN ===
if __name__ == "__main__":
    try:
        tello.connect()
        print(f"Battery: {tello.get_battery()}%")
        tello.streamon()
        time.sleep(2)
        initialise_followers()
        time.sleep(2)
        tello.takeoff()
        mimic_leader("takeoff")
        time.sleep(5)

        threading.Thread(target=capture_frames, daemon=True).start()
        threading.Thread(target=keep_alive, daemon=True).start()
        threading.Thread(target=control_movement, daemon=True).start()

        process_video()

    except KeyboardInterrupt:
        print("\n[EMERGENCY] KeyboardInterrupt detected! Landing NOW...")
        try:
            tello.land()
            mimic_leader("land")
        except Exception as e:
            print(f"[ERROR] Failed to land: {e}")

    finally:
        try:
            print("[INFO] Cleaning up drone resources...")
            tello.streamoff()
            tello.end()
        except Exception as e:
            print(f"[WARN] Cleanup failed: {e}")
