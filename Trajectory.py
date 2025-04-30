from djitellopy import Tello
import time

# Connect to the Tello drone
drone = Tello()
drone.connect()

# Take off and stabilise
drone.takeoff()
time.sleep(2)

try:
    drone.move_forward(250)
    time.sleep(2)

    drone.rotate_clockwise(180)
    time.sleep(2)

    drone.move_forward(250)
    time.sleep(2)

    drone.rotate_clockwise(180)
    time.sleep(2)

finally:
    drone.land()
