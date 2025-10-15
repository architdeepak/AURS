from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Connect to the vehicle
vehicle = connect('127.0.0.1:14550', wait_ready=True)


# Arm the drone and set mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")
vehicle.arm()

# Wait until the vehicle is armed
while not vehicle.armed:
    print("Waiting for vehicle to arm...")
    time.sleep(1)

# Set the target altitude to 10 meters and take off
vehicle.simple_takeoff(10)  # Target altitude in meters

# Wait until the drone reaches the target altitude
while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {current_altitude}")
    if current_altitude >= 10 * 0.95:  # Reached 95% of the target altitude
        print("Target altitude reached")
        break
    time.sleep(1)

# Define the target location 30 meters forward
current_location = vehicle.location.global_relative_frame
target_location = LocationGlobalRelative(
    current_location.lat + (30 / 111320),  # Convert 30 meters to degrees latitude
    current_location.lon,
    current_location.alt
)

# Move the drone forward by 30 meters (in the current direction)
vehicle.simple_goto(target_location)

# Wait until the drone reaches the target location
while True:
    current_distance = vehicle.location.global_relative_frame.distance_to(target_location)
    if current_distance < 1:  # Target reached
        print("Reached the target location.")
        break
    time.sleep(1)

# Land the drone
vehicle.mode = VehicleMode("LAND")

# Close the vehicle connection
vehicle.close()
