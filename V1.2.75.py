from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import cv2
import requests
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pymavlink import mavutil
import threading
import random

print("connecting to vehicle")
vehicle = connect('127.0.0.1:14550', wait_ready=True)
print("connected to vehicle")

ML_API_URL = "https://predict.ultralytics.com"
HEADERS = {"x-api-key": "929ca9573e76459c642e4b08f65cbb1bbaf8a1f6bf"}
MODEL_DATA = {
    "model": "https://hub.ultralytics.com/models/RJT1svwxBfBKFCUmFzUt",
    "imgsz": 640,
    "conf": 0.25,
    "iou": 0.45
}

class drone_init:
    def __init__(self, width, length, height):
        self.width = width  
        self.length = length  
        self.height = height
        self.bridge = CvBridge()
        self.image = None
        self.stop_circling = False

        rospy.init_node('image_listener', anonymous=True)
        rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def takeoff(self):
        print("taking off...")
        while not vehicle.is_armable:
            print(" waiting for vehicle to become armable...")
            time.sleep(1)
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print("waiting for arming...")
            time.sleep(1)
        vehicle.simple_takeoff(self.height)
        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
            if vehicle.location.global_relative_frame.alt >= self.height * 0.95:
                print("reached target altitude")
                break
            time.sleep(0.5)  
        
    
    def move_to(self, lat, lon, alt):
        print(f"Moving to {lat}, {lon}, {alt}")
        point = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point, airspeed=2.0)
        time.sleep(0.2)

    def move_to_fast(self, lat, lon, alt):
        point = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point, airspeed=5.0)
        time.sleep(0.2)

    def scan_area(self):
        self.start_lat = vehicle.location.global_relative_frame.lat
        self.start_lon = vehicle.location.global_relative_frame.lon
        self.start_alt = vehicle.location.global_relative_frame.alt
        print(f"Home location set: {self.start_lat}, {self.start_lon}, {self.start_alt}")
        print(f"Scanning area {self.width}m x {self.length}m at {self.height}m altitude.")
        vehicle.airspeed = 2.0
        self.zigzagsearch()

    def zigzagsearch(self):   #CAMERA CAN SEE 10 m
        lat, lon = self.start_lat, self.start_lon
        lat_step = (self.length / 111320)
        lon_step = (self.width / (111320 * math.cos(math.radians(lat))))
        avg_size = (self.length + self.width) / 2
        stops_per_pass=3
        num_passes = int((avg_size / 10) * 2)
        print(f"Lon step: {lon_step}")
        print(f"Number of passes adjusted to: {num_passes}")
        print(f"Stops per pass: {stops_per_pass}")

        for i in range(num_passes):
            start_lon = lon
            end_lon = lon + lon_step if i % 2 == 0 else self.start_lon
            stop_lon_step = (lon_step) / stops_per_pass
            for j in range(1, stops_per_pass):
                print("Sleeping due to inference")
                time.sleep(0.5)
                if self.run_inference():
                    targetlat = lat
                    targetlon = vehicle.location.global_frame.lon + (stop_lon_step/2)
                    print("flying to the target")
                    self.fly_to_target(targetlat, targetlon)
                    return
                time.sleep(0.5)
                if((start_lon + (j * stop_lon_step)) == ((lon + lon_step) - (j * stop_lon_step))):
                    print("TRUE")
                current_lon = start_lon + (j * stop_lon_step) if i % 2 == 0 else ((lon + lon_step) - (j * stop_lon_step))
                print(f"curent_lon{current_lon}")
                self.move_to(lat, current_lon, self.height)
                time.sleep(0.5)
                print(f"Stop {j+1}/{stops_per_pass} at lat: {lat}, lon: {current_lon}")
                time.sleep(0.5)
            
            print("moving to end point")
            self.move_to(lat, end_lon, self.height)
            print("Sleeping due to inference")
            time.sleep(0.5)
            if self.run_inference():
                targetlat = lat + ((lat_step/num_passes)/2)
                targetlon = vehicle.location.global_frame.lon + (stop_lon_step/2)
                print("flying to the target")
                self.fly_to_target(targetlat, targetlon)
                #self.fly_to_target(lat if i % 2 == 0 else lat + lat_step / num_passes, end_lon)
                return
            lat += lat_step / num_passes
        print("Zigzag scan complete. No target detected. Returning home and landing.")
        self.return_to_home()


    def run_inference(self):
        print("Running inference")
        if self.image is None:
            print("No image received yet.")
            return False
        
        cv2.imwrite("image.png", self.image)
        with open("image.png", "rb") as f:
            response = requests.post(ML_API_URL, headers=HEADERS, data=MODEL_DATA, files={"file": f})
        
        response.raise_for_status()
        results = response.json()['images'][0]['results']
        
        print("Sleeping due to inference")
        time.sleep(2)
        
        if results and results[0]['name'] == "person":
            print("Person detected!")
            return True
        print("No person detected )")
        return False

    def fly_to_target(self, lat, lon):
        print(f"Flying to target at {lat}, {lon}")
        self.move_to(lat, lon, self.height)
        print("About to start circling")
        time.sleep(5)
        self.circle_target(lat, lon)

    def circle_target(self, lat, lon):
        radius_meters = 4
        num_points = 6

        radius_lat = radius_meters / 111320  
        radius_lon = radius_meters / (111320 * math.cos(math.radians(lat)))  

        print(f"Circling above target with a {radius_meters}m radius. Type 's' to return to base.")

        def user_input_listener():
            while True:
                user_input = input("Type 's' to return to base: ")
                if user_input.lower() == 's':
                    self.stop_circling = True
                    break

        threading.Thread(target=user_input_listener, daemon=True).start()

        while not self.stop_circling:
            for i in range(num_points):
                if self.stop_circling:
                    break
                angle = i * (360 / num_points)
                offset_lat = lat + (radius_lat * math.cos(math.radians(angle)))
                offset_lon = lon + (radius_lon * math.sin(math.radians(angle)))
                print(f"(Moving to [{offset_lat}], [{offset_lon}] for the circle)")
                self.move_to(offset_lat, offset_lon, self.height)
                time.sleep(2)


        print("Finished circling.")
        self.return_to_home()
        #self.return_to_start()
    def return_to_home(self):
        print(f"Returning to home at {self.start_lat}, {self.start_lon}, {self.start_alt}")
        state = True
        while state:
            self.move_to(self.start_lat, self.start_lon, self.start_alt)
            print(f"current lat: {vehicle.location.global_relative_frame.lat}")
            if abs(self.start_lat - vehicle.location.global_relative_frame.lat) <= 0.00003 and abs(self.start_lon - vehicle.location.global_relative_frame.lon) <= 0.00003:
                print (self.start_lat - vehicle.location.global_relative_frame.lat)
                print(self.start_lon - vehicle.location.global_relative_frame.lon)
                print("reached home")
                state = False
        print("landing")
        self.land()

    def land(self):
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            print(" Waiting for landing...")
            time.sleep(1)
        print("Landed successfully.")
    def movetostart(self):
        print("Moving to the site")
        state = True
        lat_pos = -35.363262
        lon_pos = 149.1652374
        while state:
            self.move_to(lat_pos,lon_pos,self.height)
            print(f"current lat: {vehicle.location.global_relative_frame.lat}")
            if abs(lat_pos - vehicle.location.global_relative_frame.lat) <= 0.00001 and abs(lon_pos - vehicle.location.global_relative_frame.lon) <= 0.00001:
                print (lat_pos - vehicle.location.global_relative_frame.lat)
                print(lon_pos - vehicle.location.global_relative_frame.lon)
                print("reached home")
                state = False


try:
    width = float(input("Enter search area width (m): "))
    length = float(input("Enter search area length (m): "))
    height = float(input("Enter search altitude (m): "))

    drone = drone_init(width, length, height)
    drone.takeoff()
    drone.movetostart()
    time.sleep(5)
    print("Reached the start location: Scanning")
    drone.scan_area()

finally:
    drone.return_to_home()
    print("Closing vehicle connection...")
    vehicle.close()