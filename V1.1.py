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

print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)
print("Connected to vehicle")

ML_API_URL = "https://predict.ultralytics.com"
HEADERS = {"x-api-key": "929ca9573e76459c642e4b08f65cbb1bbaf8a1f6bf"}
MODEL_DATA = {
    "model": "https://hub.ultralytics.com/models/RJT1svwxBfBKFCUmFzUt",
    "imgsz": 640,
    "conf": 0.25,
    "iou": 0.45
}

class SearchDrone:
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
        print("Taking off...")
        while not vehicle.is_armable:
            print(" Waiting for vehicle to become armable...")
            time.sleep(1)
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        vehicle.simple_takeoff(self.height)
        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}")
            if vehicle.location.global_relative_frame.alt >= self.height * 0.95:
                print("Reached target altitude")
                break
            time.sleep(0.5)  
        self.start_lat = vehicle.location.global_relative_frame.lat
        self.start_lon = vehicle.location.global_relative_frame.lon
        self.start_alt = vehicle.location.global_relative_frame.alt
        print(f"Home location set: {self.start_lat}, {self.start_lon}, {self.start_alt}")

    def move_to(self, lat, lon, alt):
        print(f"Moving to {lat}, {lon}, {alt}")
        point = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point, airspeed=3.0)
        time.sleep(5)

    def move_to_fast(self, lat, lon, alt):
        point = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(point, airspeed=4.0)
        time.sleep(0.5)

    def scan_area(self):
        print(f"Scanning area {self.width}m x {self.length}m at {self.height}m altitude.")
        vehicle.airspeed = 2.0
        print("b")
        lat, lon = self.start_lat, self.start_lon  
        lat_step = (self.length / 111320) 
        lon_step = (self.width / (111320 * math.cos(math.radians(lat))))  
        avg_size = (self.length + self.width) / 2
    
        num_passes = (avg_size / 10) * 2
        num_passes = int(num_passes) 
    
        print(f"Number of passes adjusted to: {num_passes}")  
        for i in range(num_passes):
            lon_target = lon + lon_step if i % 2 == 0 else lon - lon_step
            lat += lat_step / num_passes  
            
            self.move_to(lat, lon_target, self.height)
            
            print("Sleeping due to inference")
            time.sleep(2)
            
            if self.run_inference():
                self.fly_to_target(lat, lon_target)
                return  
        
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
        self.move_to_fast(lat, lon, self.height / 2)
        self.circle_target(lat, lon)
        

    def circle_target(self, lat, lon):
        radius_meters = 20
        num_points = 15 

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
                self.move_to_fast(offset_lat, offset_lon, self.height / 2)

        print("Finished circling.")
        self.return_to_home()

    def return_to_home(self):
        print(f"Returning to home at {self.start_lat}, {self.start_lon}, {self.start_alt}")
        state = True
        while state:
            self.move_to(self.start_lat, self.start_lon, self.start_alt)
            print(f"current lat: {vehicle.location.global_relative_frame.lat}")
            if abs(self.start_lat - vehicle.location.global_relative_frame.lat) <= 0.00005 and abs(self.start_lon - vehicle.location.global_relative_frame.lon) <= 0.00005:
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

try:
    width = float(input("Enter search area width (m): "))
    length = float(input("Enter search area length (m): "))
    height = float(input("Enter search altitude (m): "))

    drone = SearchDrone(width, length, height)
    drone.takeoff()
    drone.scan_area()
finally:
    drone.return_to_home()
    print("Closing vehicle connection...")
    vehicle.close()
