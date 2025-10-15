from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)           #make sure ts match in sitl outputtttttttt 
print("Connected to vehicle.")
def arm_and_takeoff(target_altitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:  
            print("Reached target altitude")
            break
        time.sleep(1)

def fly_in_circle(radius, altitude, center_lat, center_lon, num_points=36):
    print("Flying in a circle...")
    for i in range(num_points):
        angle = (2 * math.pi / num_points) * i
        target_lat = center_lat + (radius * math.cos(angle) / 111111) 
        target_lon = center_lon + (radius * math.sin(angle) / (111111 * math.cos(math.radians(center_lat))))
        target_location = LocationGlobalRelative(target_lat, target_lon, altitude)
        vehicle.simple_goto(target_location)
        time.sleep(2)  

try:
    target_altitude = 10  
    circle_radius = 10  
    arm_and_takeoff(target_altitude)
    current_location = vehicle.location.global_frame
    center_lat = current_location.lat
    center_lon = current_location.lon
    fly_in_circle(circle_radius, target_altitude, center_lat, center_lon)

    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0.1:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        time.sleep(1)
    print("Landed!")

finally:
    print("Closing vehicle connection...")
    vehicle.close()
