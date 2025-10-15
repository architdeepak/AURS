from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

def connect_drone():
    vehicle = connect("127.0.0.1:14550", wait_ready=True)  # Change if using a real drone
    return vehicle

def arm_and_takeoff(vehicle, altitude):
    print("Arming motors...")
    while not vehicle.is_armable:
        time.sleep(1)
    
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    
    print("Taking off...")
    vehicle.simple_takeoff(altitude)
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            break
        time.sleep(1)

def move_to_location(vehicle, lat_offset, lon_offset):
    current_location = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(current_location.lat + lat_offset,
                                             current_location.lon + lon_offset,
                                             current_location.alt)
    
    print(f"Moving to {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)
    
    time.sleep(10)  # Wait for the drone to reach the location
    print("Reached target location.")

def main():
    vehicle = connect_drone()
    arm_and_takeoff(vehicle, 10)  # Takeoff to 10 meters
    
    lat_offset = float(input("Enter latitude offset (+/-): "))
    lon_offset = float(input("Enter longitude offset (+/-): "))
    move_to_location(vehicle, lat_offset, lon_offset)
    
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()

if __name__ == "__main__":
    main()
