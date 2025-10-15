from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def connect_drone():
    print("Connecting to vehicle...")
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    print("Connected to vehicle")
    return vehicle

def takeoff(vehicle, target_altitude):
    print("Taking off...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:  
            print("Reached target altitude")
            break
        time.sleep(1)

def move_to(vehicle, lat, lon, alt):
    print(f"Moving to {lat}, {lon}, {alt}")
    point = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(point)
    time.sleep(5)  

def lawnmower_scan(vehicle, area_length=30, area_width=30, altitude=20, step_size=5):
    print(f"Scanning {area_length}m x {area_width}m area at {altitude}m altitude")
    vehicle.airspeed = 5  
    
    start_lat = vehicle.location.global_relative_frame.lat
    start_lon = vehicle.location.global_relative_frame.lon
    
    direction = 1  
    for i in range(0, area_width, step_size):
        lat = start_lat + (i * 0.00001)
        lon = start_lon + (direction * area_length * 0.00001)
        move_to(vehicle, lat, lon, altitude)
        direction *= -1  
    
    print("Area scan complete.")

def main():
    vehicle = connect_drone()
    try:
        takeoff(vehicle, 20)
        lawnmower_scan(vehicle)
    finally:
        print("Closing vehicle connection...")
        vehicle.close()

if __name__ == "__main__":
    main()

