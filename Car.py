from Sensor import PID, Camera
import Esp32
import numpy as np
import cv2 as cv
import Arena
import time as time


WIDTH = 80 #mm
LENGTH = 179 #mm
CAR_SIZE = (LENGTH, WIDTH)
WHEEL_SIZE = (23, 15)
TURNING_RADIUS = 245 # mm
REAL_TURNING_RADIUS = TURNING_RADIUS * 1.1
COLOUR = (80,80,80)
WHEEL_COLOUR = (0,0,0)

#distances are from back right corner. 0, 0 (car coordinates) is at the back right corner, x extends positive to the front of the car, y extends positive to the right of the car.
REAR_AXLE_POS = 20 #mm
WHEEL_INSET = 20 #mm
STEERING_AXLE_OFFSET = 8 #mm
STEERING_SPEED = 2 #rad/sec
MAX_SPEED = 60.0 #mm/sec
ACCELERATION_LIMIT = 600 #mm/sec^2
DRAG = 0.96
BACK_SENSOR_OFFSET = 20 #mm, distance from centerline of car to back sensors
SIDE_SENSOR_OFFSET = 50 #mm, distance from centerline of car to side sensors

DIRECTIONS = {
"FRONT": 0,
"LEFT": np.pi/2,
"BACK": np.pi,
"RIGHT": 3 * np.pi/2
}

def Dot(x1, y1, x2, y2):
    return x1 * x2 + y1 * y2 

def Cross(x1, y1, x2, y2):
    return x1 * y2 - x2 * y1

class Car:
     
    def __init__(self, centerX, centerY, direction):
        self.x = centerX
        self.y = centerY
        self.dir = direction
        self.speed = 0
        self.wheel_dir = 0
        self.wheelbase = 120 
        self.state = "UNINITIALISED"
        self.current_time = time.perf_counter() 
        self.last_time = self.current_time - 0.001
        self.distance_data = {}
        self.colour_data = {}
        self.hit_data = {}
        self.visited_colours = []

        self.sensors = { 
            "LEFT": PID(LENGTH/2 - SIDE_SENSOR_OFFSET, 0, DIRECTIONS["LEFT"], self, Arena.START_OFFSET_Y),
            "RIGHT": PID(LENGTH/2 - SIDE_SENSOR_OFFSET, WIDTH, DIRECTIONS["RIGHT"], self, Arena.TRACK_WIDTH - WIDTH - Arena.START_OFFSET_Y),
            "BACK1": PID(0, WIDTH/2 + BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self, Arena.START_OFFSET_X),
            "BACK2": PID(0, WIDTH/2 - BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self, Arena.START_OFFSET_X),
        }

        self.esp = Esp32.Esp32("connectioninfo", self, self.sensors)

        self.camera = Camera(LENGTH - 10, 10, DIRECTIONS["RIGHT"], self)

        self.camera_rays = (
            np.pi/5,
            np.pi/6,
            np.pi/8,
            0,
            -np.pi/8
            -np.pi/6,
            -np.pi/5
        )

        self.target_x = centerX + 100
        self.target_y = centerY

        

    @property
    def x_offset(self): #offset from center X to back left corner
        return self.x + (-LENGTH * np.cos(self.dir) + WIDTH *np.sin(self.dir)) / 2

    @property
    def y_offset(self): #offset from center Y to back left corner
        return self.y + (LENGTH * np.sin(self.dir) + WIDTH *np.cos(self.dir)) / 2

    def relToAbs(self, x, y):
        absX = self.x_offset + x * np.cos(self.dir) - y * np.sin(self.dir)
        absY = self.y_offset - x * np.sin(self.dir) - y * np.cos(self.dir)
        return (absX, absY)

    @property
    def wheel_rects(self):
        return [
            (self.relToAbs(REAR_AXLE_POS, WHEEL_INSET), WHEEL_SIZE, -180/np.pi * (self.dir - self.wheel_dir)),
            (self.relToAbs(REAR_AXLE_POS, WIDTH-WHEEL_INSET), WHEEL_SIZE, -180/np.pi * (self.dir - self.wheel_dir)),
            (self.relToAbs(REAR_AXLE_POS + self.wheelbase, WHEEL_INSET), WHEEL_SIZE, -180/np.pi * self.dir),
            (self.relToAbs(REAR_AXLE_POS + self.wheelbase, WIDTH-WHEEL_INSET), WHEEL_SIZE, -180/np.pi * self.dir),
        ]

    @property
    def max_wheel_dir(self):
        return np.atan(self.wheelbase / TURNING_RADIUS)

    @property
    def dt(self):
        return (self.current_time - self.last_time) 

    def UpdateClockDiff(self):
        self.last_time = self.current_time
        self.current_time = time.perf_counter()
        #print(f"dt: {self.dt}"")

    def PerSecondToPerCycle(self, num_per_sec):
        return num_per_sec * self.dt

    def PerCycleToPerSecond(self, num_per_cycle):
        return num_per_cycle / self.dt

    def Draw(self, img):
        car_rect = Arena.RotatedRect((self.x, self.y), (LENGTH, WIDTH), -180/np.pi * self.dir, COLOUR)
        car_rect.Draw(img)
        for wheel_rect_data in self.wheel_rects:
            wheel_rect = Arena.RotatedRect(wheel_rect_data[0], wheel_rect_data[1], wheel_rect_data[2], WHEEL_COLOUR)
            wheel_rect.Draw(img)

    def Drive(self, target_speed):
        inst_accel_limit = self.PerSecondToPerCycle(self.PerSecondToPerCycle(ACCELERATION_LIMIT))
        inst_speed = self.PerSecondToPerCycle(self.speed)
        inst_target = self.PerSecondToPerCycle(target_speed)
        #print(f"inst_speed: {inst_speed}, inst_accel_limit: {inst_accel_limit}, target: {inst_target}")

        if inst_target > inst_speed:
            inst_speed = inst_speed + inst_accel_limit
            if inst_target < inst_speed:
                inst_speed = inst_target
        elif inst_target < inst_speed:
            inst_speed = inst_speed - inst_accel_limit
            if inst_target > inst_speed:
                inst_speed = inst_target

        self.speed = self.PerCycleToPerSecond(inst_speed)

        if self.speed > MAX_SPEED:
            self.speed = MAX_SPEED
        elif self.speed < -MAX_SPEED:
            self.speed = -MAX_SPEED

    def Turn(self, angle):
        old_angle = self.wheel_dir
        inst_steering_speed = self.PerSecondToPerCycle(STEERING_SPEED)
        dtheta = angle - self.wheel_dir
        if abs(dtheta) > inst_steering_speed:
            if dtheta > 0:
                dtheta = inst_steering_speed
            else:
                dtheta = -inst_steering_speed

        if self.wheel_dir + dtheta > self.max_wheel_dir:
            self.wheel_dir = self.max_wheel_dir
        elif self.wheel_dir + dtheta < -self.max_wheel_dir:
            self.wheel_dir = -self.max_wheel_dir
        else:
            self.wheel_dir += dtheta

        if old_angle != self.wheel_dir:
            delta = self.wheel_dir - old_angle
            self.wheelbase = 120 + STEERING_AXLE_OFFSET * np.cos(self.wheel_dir) - STEERING_AXLE_OFFSET
            rel_x_wheel_movement = STEERING_AXLE_OFFSET * np.cos(delta) - STEERING_AXLE_OFFSET
            rel_y_wheel_movement = STEERING_AXLE_OFFSET * np.sin(delta)

            interpolation_constant = ((REAR_AXLE_POS + self.wheelbase - LENGTH/2)/self.wheelbase)
            rel_x_center_movement = rel_x_wheel_movement * interpolation_constant
            rel_y_center_movement = rel_y_wheel_movement * interpolation_constant
            abs_x_center_movement = -(rel_x_center_movement * np.cos(self.dir) - rel_y_center_movement * np.sin(self.dir))
            abs_y_center_movement = -(rel_x_center_movement * np.sin(self.dir) + rel_y_center_movement * np.cos(self.dir))

            self.x = self.x - abs_x_center_movement
            self.y = self.y - abs_y_center_movement
            self.dir = self.dir + np.arctan2(rel_y_wheel_movement, self.wheelbase)

    def Move(self, real):
        inst_speed = self.PerSecondToPerCycle(self.speed)
        self.x = self.x + inst_speed * np.cos(self.dir - self.wheel_dir/2)
        self.y = self.y - inst_speed * np.sin(self.dir - self.wheel_dir/2)
        self.dir = self.dir + inst_speed/self.wheelbase * np.tan(self.wheel_dir)

        if real:
            self.esp.SetTurningAngle(self.wheel_dir)
            self.esp.SetMotorSpeed(self.speed)
            self.esp.SendRequest()

    def DistTo(self, x, y):
        dx = x - self.x
        dy = y - self.y 
        return np.sqrt(dx**2 + dy**2)



    def MoveTo(self, target_x, target_y, target_dir): 
        AGGRESSIVENESS = 1
        CLOSE_DIST =  100#mm
        TARGET_LINE_WIDTH = 10
        BACKUP_DIST = LENGTH #mm

        dx = (target_x - self.x)
        dy = (target_y - self.y)
        
        dist_to_target = self.DistTo(target_x, target_y)
        dist_to_target_line = Cross(dx, dy, np.cos(target_dir), -np.sin(target_dir))
        out_of_lineness = abs(dist_to_target_line)/dist_to_target # 1 if the path we need to take to get to the target is along the line, 0 if we are far away or we are parallel with the end point
        angle_alignment = -Dot(dx /  dist_to_target, dy / dist_to_target, np.cos(target_dir), -np.sin(target_dir)) # 1 if the path to the target and the target direction align, 0 if we are approaching from the side. -1 if we are facing the opposite way.

        path_dir = np.atan2(-dy, dx) 
        self_dir = self.dir
        if self.speed < 0:
            self_dir = self.dir + np.pi

        dtheta_path = (((path_dir - self_dir + np.pi) % (2 * np.pi)) - np.pi) 
        print(f"dtheta_path: {dtheta_path}\npath_dir: {path_dir}\nself_dir:{self_dir}")
        dtheta_wheels_target = ((target_dir - self.wheel_dir + self.dir + np.pi) % (2 * np.pi)) - np.pi       
        dtheta_target = ((target_dir - self.dir + np.pi) % (2 * np.pi)) - np.pi    

        closeness = (min(1, max(0.01, (CLOSE_DIST - dist_to_target) / CLOSE_DIST))**5)
        print(f"closeness: {closeness}")
        side = np.sign(dist_to_target_line)
        if side == 0: side = 1
        facing = np.sign(dtheta_wheels_target) * side
        if facing == 0: facing = 1
        forwards = np.sign(self.speed)
        if forwards == 0: forwards = 1
        overshot = np.sign(angle_alignment) > 0

        leaving_target_line = abs(dist_to_target_line) * facing * forwards > TARGET_LINE_WIDTH / 2

        print(f"leaving: {leaving_target_line}\nside: {side}\nfacing: {facing}\nforwards: {forwards}\novershot: {overshot}")

        target_speed = min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist_to_target)) * forwards
        if abs(dtheta_target) > 0.05 and dist_to_target_line > TARGET_LINE_WIDTH:
            target_wheel_dir = dtheta_wheels_target * 1.6 * forwards
        else:
            target_wheel_dir = dtheta_path * forwards#+ dtheta_target * (1-(closeness)) * (forwards + 1) /2
        if dist_to_target_line * side < TARGET_LINE_WIDTH / 3:
            print(f"dtheta_target = {dtheta_target}\n angle alignment: {angle_alignment}")
            if forwards == -1:
                target_wheel_dir = np.sign(target_wheel_dir) * self.max_wheel_dir
            if forwards == 1 and abs(dtheta_target) < 0.05:
                target_wheel_dir =  dtheta_target * 0.7 + target_wheel_dir * -0.3

        #target_wheel_dir = target_wheel_dir * abs(forwards + 0.8) / 1.8 * AGGRESSIVENESS
        
        # have we overshot?
        if leaving_target_line:
                target_speed = min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist_to_target)) * forwards
        elif forwards < 0 and (out_of_lineness < 0.2 and dist_to_target > BACKUP_DIST):
            target_speed = min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist_to_target))
        if overshot:
            target_speed = -min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist_to_target))
        
            

        #print(f"dist to target: {dist}")
        print(f"target speed: {target_speed} target turning angle: {target_wheel_dir}")
        self.Turn(target_wheel_dir)
        self.Drive(target_speed)



    def DoAutomation(self, generated_arena, code_arena, arena, ignored_colours, real):

        for sensor in self.esp.sensors.values():
            if real:
                self.distance_data[sensor] = sensor.SenseRealDist()
            else:
                self.distance_data[sensor] = sensor.SenseSimDist(generated_arena)
            self.hit_data[sensor] = sensor.FindHitData(code_arena)

        #self.LocateOnTrack(code_arena)

        for angle in self.camera_rays:
            if real:
                self.colour_data[angle] = self.camera.SenseRealColour()
            else:
                self.colour_data[angle] = self.camera.SenseSimColour(generated_arena, angle)
            if self.colour_data[angle] is not None:
                self.InterpretCameraData(arena, angle, self.colour_data[angle], ignored_colours)
            
        if self.state == "FINDING_START_COLOURS":
            self.MoveTo(self.target_x, self.target_y, 0)
               
        elif self.state == "SEARCHING_FOR_TARGET":
            print(self.DistTo(self.target_x,self.target_y))
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < 3 and self.state == "SEARCHING_FOR_TARGET":
                self.state = "DONE"
                
        elif self.state == "PREPARING_TO_PARK":
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < 5:
                self.state = "PARKING"
                self.speed = 1

        elif self.state == "PARKING":
            self.FindTarget(arena)
            self.MoveTo(self.target_x, self.target_y, np.pi* 3 / 2)
            if self.DistTo(self.target_x, self.target_y) < 3 and self.state == "PARKING": 
                self.state = "PARKED"

        elif self.state == "PARKED":
            cv.waitKey(0)
            self.UpdateClockDiff()
            self.FindTarget(arena)
            if self.state != "REVERSING_OUT":
                self.state = "DONE"
            else:
                self.speed = -1

        elif self.state == "REVERSING_OUT":
            self.MoveTo(self.target_x, self.target_y, 0)
            if self.DistTo(self.target_x, self.target_y) < 5:
                self.state = "SEARCHING_FOR_TARGET"
                self.speed = 1



        print(self.state)
        #print(f"Target: {self.target_x}, {self.target_y}")
        #print(f"Dist to: {self.DistTo(self.target_x, self.target_y)}")
        #print(f"location: {self.x}, {self.y}")

    def LocateOnTrack(self, code_arena):
        x_guess = self.x
        y_guess = self.y
        dir_guess = self.dir

        x_guesses = {}
        y_guesses = {}

        hit_1d = {}

        for sensor in self.esp.sensors.values():
            hit = self.hit_data[sensor]
            dist = self.distance_data[sensor]

            if hit[0] is not None:
                x_guesses[sensor] = hit[0] + (self.x - sensor.x) - dist * np.cos(sensor.dir)
                hit_1d[sensor] = hit[0]

            if hit[1] is not None:
                y_guesses[sensor] = hit[1] + (self.y - sensor.y) + dist * np.sin(sensor.dir)
                hit_1d[sensor] = hit[1]

        

        if len(x_guesses) > 0:
            x_guess = np.median(list(x_guesses.values()))

        if len(y_guesses) > 0:
            y_guess = np.median(list(y_guesses.values()))

        
        DistBetweenSensors = lambda sensor1, sensor2: np.sqrt((sensor1.x - sensor2.x)**2 + (sensor1.y - sensor2.y)**2)
        DistBetweenGuesses = lambda sensor1, sensor2, guessesDict: guessesDict[sensor2] - guessesDict[sensor1]
        PrettyClose = lambda num1, num2: abs(num1 - num2) < 5

        dir_guesses = []

        for guess_dict in (x_guesses, y_guesses):
            if len(guess_dict) >= 2:
                sensors = list(guess_dict.keys())
                for sensor1 in sensors:
                    for sensor2 in sensors[sensors.index(sensor1):]:
                        if PrettyClose(guess_dict[sensor1], guess_dict[sensor2]):
                            if sensor1.dir_offset == sensor2.dir_offset:
                                dir_guesses.append(
                                    np.atan2(
                                        DistBetweenGuesses(sensor1, sensor2, self.distance_data),
                                        DistBetweenSensors(sensor1, sensor2)
                                    )
                                )
                            else:
                                    dir_guesses.append(
                                        np.atan2(
                                            self.distance_data[sensor1] 
                                            + sensor1.x_offset * np.cos(sensor1.dir_offset) 
                                            + sensor1.y_offset * np.sin(sensor1.dir_offset)
                                            ,
                                            self.distance_data[sensor2] 
                                            + sensor2.x_offset * np.sin(sensor2.dir_offset)
                                            + sensor2.y_offset * np.cos(sensor2.dir_offset)
                                    ))
                        else:
                            dir_guesses.append(
                                np.arccos(
                                    DistBetweenGuesses(sensor1, sensor2, hit_1d) /
                                    (self.distance_data[sensor1] + self.distance_data[sensor2] + DistBetweenSensors(sensor1, sensor2))
                                )
                            )

        if len(dir_guesses) > 0:
            print(dir_guesses)
            dir_guess = np.median(dir_guesses)
        
            
        print(f"Updating to x:{self.x}, y:{self.y}, dir:{dir_guess}")
        (self.x, self.y, self.dir) = (x_guess, y_guess, dir_guess)

    def PointInRect(self, x, y, rect):
        if rect.pt1[0] <= x and rect.pt2[0] >= x and rect.pt1[1] <= y and rect.pt2[1] >= y:
            return True
        else:
            return False

    def InterpretCameraData(self, arena, angle, colour, ignored_colours):
        if colour != Arena.PARKED_CAR_COLOUR:
            while np.array_equal(self.camera.SenseSimColour(arena, angle), Arena.PARKED_CAR_COLOUR): 
                (x, y) = self.camera.FindRayIntercept(arena.img, angle)
                full_car_rects =  [rect for rect in arena.car_rects if rect.colour is not None]
                for car_rect in full_car_rects:
                    if self.PointInRect(x ,y, car_rect):
                        car_rect.colour = None  
                        car_rect.Draw(arena.img)

        (x, y) = self.camera.FindRayIntercept(arena.img, angle)
        if not any(np.array_equal(colour, bad_colour) for bad_colour in ignored_colours):
            for rect in (arena.start_rects):
                if self.PointInRect(x, y, rect):
                    rect.colour = colour
                    rect.Draw(arena.img)
                    if self.state == "FINDING_START_COLOURS":
                        self.state = "SEARCHING_FOR_TARGET"
                        self.target_x = Arena.TOTAL_WIDTH - arena.start_pos[0]
                        self.target_y = arena.start_pos[1]

        valid_park_colours = []
        for start_rect in arena.start_rects:
            if start_rect.filled:
                valid_park_colours.append(start_rect.colour)

        for valid_colour in valid_park_colours:
            if np.array_equal(colour, valid_colour):
                for rect in arena.colour_rects:
                    if not rect.filled and self.PointInRect(x, y, rect):
                        rect.colour = colour
                        rect.Draw(arena.img)

    def FindTarget(self, arena):
        if self.state == "DONE" or self.state == "REVERSING OUT":
            pass

        empty_parks = [park for park in arena.car_rects if not park.filled]
        target_rects = [rect for rect in arena.colour_rects if rect.filled and not any(np.array_equal(colour, rect.colour) for colour in self.visited_colours)] # very long line # very useful comment
        target_parks = []
        target_park_rects = []
        for park in empty_parks:
            for rect in target_rects:
                if park.center[0] == rect.center[0]:
                    target_parks.append(park)
                    target_park_rects.append(rect)


        if len(target_parks) > 0:
            if self.state == "SEARCHING_FOR_TARGET":
                self.state = "PREPARING_TO_PARK"
            target_park = sorted(target_parks, key = lambda rect: rect.pt1[0])[0]
            target_park_rect = sorted(target_park_rects, key = lambda rect: rect.pt1[0])[0]
            if self.state == "PARKED":
                print("here")
                self.visited_colours.append(target_park_rect.colour)
                self.state = "REVERSING_OUT"
                self.target_x = target_park.center[0] - REAL_TURNING_RADIUS
                self.target_y = arena.start_pos[1]
            if self.state == "PARKING":
                (self.target_x, self.target_y) = target_park.center
                return
            if self.state == "PREPARING_TO_PARK":
                self.target_x = target_park.center[0] - REAL_TURNING_RADIUS
                self.target_y = arena.start_pos[1]
                return
        else:
            
            self.state = "SEARCHING_FOR_TARGET"
            self.target_x = min(Arena.TOTAL_WIDTH - arena.start_pos[0], self.x + LENGTH * 1.5)
            self.target_y = arena.start_pos[1]
        