from pymavlink import mavutil
import time
import math

master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')


print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received!")

def send_command(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,  
        param1, param2, param3, param4, param5, param6, param7
    )


def arm_drone():
    print("Arming the drone...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Drone armed!")


def takeoff(altitude):
    print(f"Taking off to {altitude} meters...")
    send_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
    time.sleep(10)  

def fly_in_circle(radius, altitude, duration=60):
    print(f"Flying in a circle with radius {radius} meters...")
    start_time = time.time()

    while time.time() - start_time < duration:
        elapsed = time.time() - start_time
        angle = 2 * math.pi * elapsed / duration
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)


        send_command(
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0,
            x, y, altitude
        )
        time.sleep(1)


def land():
    print("Landing...")
    send_command(mavutil.mavlink.MAV_CMD_NAV_LAND)
    time.sleep(10)  


def main():
    arm_drone()
    takeoff(altitude=100)
    fly_in_circle(radius=50, altitude=100, duration=120)
    land()
    print("Mission complete!")

if __name__ == "__main__":
    main()
