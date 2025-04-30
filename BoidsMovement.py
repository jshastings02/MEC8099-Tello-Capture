##Successful Trail of Boids Movement
# Note: this is a simulated movement not based on object detection and awareness of eachother
#author: Iline Shaju
import socket
import time
import threading
import numpy as np
import tkinter as tk

# Drone configuration (IP addresses and ports)
DRONES = [
    {"local_ip": "192.168.10.2", "tello_ip": "192.168.10.1", "port": 8889},
    {"local_ip": "192.168.10.3", "tello_ip": "192.168.10.1", "port": 8889},
    {"local_ip": "192.168.10.4", "tello_ip": "192.168.10.1", "port": 8889},
    {"local_ip": "192.168.10.5", "tello_ip": "192.168.10.1", "port": 8889},
]

# Define initial positions in a square formation
positions = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])
velocities = np.zeros_like(positions)

# Function to send a command via UDP
def send_command(sock, tello_ip, port, command):
    try:
        print(f"Sending command: {command} to {tello_ip}:{port}")
        sock.sendto(command.encode(), (tello_ip, port))
        response, _ = sock.recvfrom(1024)  # Wait for response
        print(f"Response from {tello_ip}: {response.decode().strip()}")
    except Exception as e:
        print(f"Error sending command to {tello_ip}: {e}")

# Function to initialize a drone
def initialize_drone(drone):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((drone["local_ip"], drone["port"]))

        send_command(sock, drone["tello_ip"], drone["port"], "command")
        time.sleep(1)

        return sock
    except Exception as e:
        print(f"Error initializing drone {drone['tello_ip']}: {e}")
        return None

# Define separation, alignment, and cohesion forces
def separation(drone_idx, min_distance=0.3):
    force = np.array([0.0, 0.0])
    for i, pos in enumerate(positions):
        if i != drone_idx:
            diff = positions[drone_idx] - pos
            distance = np.linalg.norm(diff)
            if distance < min_distance:
                force += diff / (distance + 1e-6)  # Avoid division by zero
    return force * 0.5  # Reduce aggressive movement

def alignment(drone_idx):
    avg_velocity = np.mean(velocities, axis=0)
    return (avg_velocity - velocities[drone_idx]) * 0.1  # Smooth matching

def cohesion(drone_idx):
    center = np.mean(positions, axis=0)
    return (center - positions[drone_idx]) * 0.05  # Move slightly to center

# Function to send movement commands based on flocking behavior
def apply_behavior(behavior):
    global positions, velocities
    for _ in range(10):  # Run for a few iterations
        for i, sock in enumerate(sockets):
            if behavior == "separation":
                force = separation(i)
            elif behavior == "alignment":
                force = alignment(i)
            elif behavior == "cohesion":
                force = cohesion(i)
            else:
                force = np.array([0.0, 0.0])  # Default no movement

            velocities[i] += force
            positions[i] += velocities[i] * 0.1  # Apply small movement

            x_speed = int(velocities[i][0] * 100)
            y_speed = int(velocities[i][1] * 100)

            x_speed = max(min(x_speed, 50), -50)
            y_speed = max(min(y_speed, 50), -50)

            command = f"rc {x_speed} {y_speed} 0 0"
            send_command(sock, DRONES[i]["tello_ip"], DRONES[i]["port"], command)

        time.sleep(0.5)  # Allow drones to execute

# Function for takeoff
def takeoff():
    for i, sock in enumerate(sockets):
        send_command(sock, DRONES[i]["tello_ip"], DRONES[i]["port"], "takeoff")
    time.sleep(3)

# Function for landing
def land():
    for i, sock in enumerate(sockets):
        send_command(sock, DRONES[i]["tello_ip"], DRONES[i]["port"], "land")
    time.sleep(3)

# Function to run behavior in a separate thread
def run_behavior(behavior):
    threading.Thread(target=apply_behavior, args=(behavior,)).start()

# Initialize drones
sockets = []
for drone in DRONES:
    sock = initialize_drone(drone)
    if sock:
        sockets.append(sock)

# Tkinter GUI
root = tk.Tk()
root.title("Tello Drone Flocking Control")

tk.Button(root, text="Takeoff", command=takeoff, width=15, height=2, bg="green").pack(pady=5)
tk.Button(root, text="Separation", command=lambda: run_behavior("separation"), width=15, height=2).pack(pady=5)
tk.Button(root, text="Alignment", command=lambda: run_behavior("alignment"), width=15, height=2).pack(pady=5)
tk.Button(root, text="Cohesion", command=lambda: run_behavior("cohesion"), width=15, height=2).pack(pady=5)
tk.Button(root, text="Land", command=land, width=15, height=2, bg="red").pack(pady=5)

root.mainloop()