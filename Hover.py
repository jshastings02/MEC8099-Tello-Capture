from djitellopy import Tello
import time

# Connect to Tello
tello = Tello()
tello.connect()

# Print battery level to verify connection
print("Battery:", tello.get_battery(), "%")

# Take off
tello.takeoff()

# Hover for 5 seconds
time.sleep(5)

# Land
tello.land()
