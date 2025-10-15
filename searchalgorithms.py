def zigzag_search(self):
    lat, lon = self.start_lat, self.start_lon
    lat_step = self.length / 111320
    lon_step = self.width / (111320 * math.cos(math.radians(lat)))
    
    num_passes = 6
    for i in range(num_passes):
        lon_target = lon + lon_step if i % 2 == 0 else lon - lon_step
        lat += lat_step / num_passes
        
        self.move_to(lat, lon_target, self.height)
        time.sleep(2)  # Simulating scan delay
        
        if self.run_inference():
            self.fly_to_target(lat, lon_target)
            return
    
    print("Zigzag scan complete. No target detected. Returning home.")
    self.return_to_home()

def snake_search(self):
    lat, lon = self.start_lat, self.start_lon
    lat_step = self.length / 111320
    lon_step = self.width / (111320 * math.cos(math.radians(lat)))
    
    for i in range(int(self.length / 10)):
        for j in range(int(self.width / 10)):
            self.move_to(lat, lon, self.height)
            time.sleep(2)
            
            if self.run_inference():
                self.fly_to_target(lat, lon)
                return
            
            lon += lon_step if i % 2 == 0 else -lon_step
        lat += lat_step
    
    print("Snake search complete. No target detected. Returning home.")
    self.return_to_home()

def spiral_search(self):
    lat, lon = self.start_lat, self.start_lon
    step = self.length / 111320
    radius = step
    angle_step = 20
    
    for angle in range(0, 720, angle_step):
        new_lat = lat + (radius * math.cos(math.radians(angle)))
        new_lon = lon + (radius * math.sin(math.radians(angle)))
        self.move_to(new_lat, new_lon, self.height)
        time.sleep(2)
        
        if self.run_inference():
            self.fly_to_target(new_lat, new_lon)
            return
        
        radius += step / 5
    
    print("Spiral search complete. No target detected. Returning home.")
    self.return_to_home()

def grid_search(self):
    lat, lon = self.start_lat, self.start_lon
    lat_step = self.length / 111320
    lon_step = self.width / (111320 * math.cos(math.radians(lat)))
    
    rows, cols = int(self.length / 10), int(self.width / 10)
    for i in range(rows):
        for j in range(cols):
            self.move_to(lat, lon, self.height)
            time.sleep(2)
            
            if self.run_inference():
                self.fly_to_target(lat, lon)
                return
            
            lon += lon_step
        lat += lat_step
        lon = self.start_lon
    
    print("Grid search complete. No target detected. Returning home.")
    self.return_to_home()

def random_walk_search(self):
    lat, lon = self.start_lat, self.start_lon
    step = self.length / 111320
    
    for _ in range(20):
        new_lat = lat + random.choice([-step, step])
        new_lon = lon + random.choice([-step, step])
        self.move_to(new_lat, new_lon, self.height)
        time.sleep(2)
        
        if self.run_inference():
            self.fly_to_target(new_lat, new_lon)
            return
        
        lat, lon = new_lat, new_lon
    
    print("Random walk search complete. No target detected. Returning home.")
    self.return_to_home()

def concentric_circles_search(self):
    lat, lon = self.start_lat, self.start_lon
    step = self.length / 111320
    radius = step
    
    for _ in range(5):
        for angle in range(0, 360, 45):
            new_lat = lat + (radius * math.cos(math.radians(angle)))
            new_lon = lon + (radius * math.sin(math.radians(angle)))
            self.move_to(new_lat, new_lon, self.height)
            time.sleep(2)
            
            if self.run_inference():
                self.fly_to_target(new_lat, new_lon)
                return
        radius += step
    
    print("Concentric circles search complete. No target detected. Returning home.")
    self.return_to_home()

def star_search(self):
    lat, lon = self.start_lat, self.start_lon
    step = self.length / 111320
    angles = [0, 72, 144, 216, 288]
    
    for angle in angles:
        new_lat = lat + (step * math.cos(math.radians(angle)))
        new_lon = lon + (step * math.sin(math.radians(angle)))
        self.move_to(new_lat, new_lon, self.height)
        time.sleep(2)
        
        if self.run_inference():
            self.fly_to_target(new_lat, new_lon)
            return
    
    print("Star search complete. No target detected. Returning home.")
    self.return_to_home()

def circle_search(self):
    lat, lon = self.start_lat, self.start_lon
    radius = min(self.length, self.width) / (2 * 111320)
    num_points = max(4, int((2 * math.pi * radius) / (self.length / 10)))
    
    for i in range(num_points):
        angle = i * (360 / num_points)
        target_lat = lat + (radius * math.cos(math.radians(angle)))
        target_lon = lon + (radius * math.sin(math.radians(angle)))
        
        self.move_to(target_lat, target_lon, self.height)
        time.sleep(2)
        
        if self.run_inference():
            self.fly_to_target(target_lat, target_lon)
            return
        
        for j in range(12):
            circ_angle = j * (360 / 12)
            circ_lat = target_lat + (radius / 10 * math.cos(math.radians(circ_angle)))
            circ_lon = target_lon + (radius / 10 * math.sin(math.radians(circ_angle)))
            self.move_to(circ_lat, circ_lon, self.height)
            time.sleep(2)
            
            if self.run_inference():
                self.fly_to_target(circ_lat, circ_lon)
                return
    
    print("Circle search complete. No target detected. Returning home.")
    self.return_to_home()