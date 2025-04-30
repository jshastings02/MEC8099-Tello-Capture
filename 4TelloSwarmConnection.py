#Author: Iline Shaju

import socket
import time

# Local IPs for each network adapter
LOCAL_IPS = ["192.168.10.2", "192.168.10.3", "192.168.10.4", "192.168.10.5"]
TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889

# Create sockets for each drone and bind to corresponding adapter
sockets = []
for local_ip in LOCAL_IPS:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, TELLO_PORT))
    sockets.append(sock)

# Function to send a command to each drone through its corresponding socket
def send_to_all(command):
    for sock in sockets:
        sock.sendto(command.encode(), (TELLO_IP, TELLO_PORT))
    print(f"Sent: '{command}' to all drones.")

# Basic takeoff
send_to_all("command")
time.sleep(2)

send_to_all("takeoff")
time.sleep(5)

# Then land all drones
send_to_all("land")
