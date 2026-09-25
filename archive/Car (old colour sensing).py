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

#distances are from back right corner. 0, 0 (car coordinates) is at the back right corner, x extends positive to the front of the car, y extends positive to the right of the car.
REAR_AXLE_POS = 20 #mm
WHEEL_INSET = 20 #mm
STEERING_AXLE_OFFSET = 8 #mm
STEERING_SPEED = 1 #rad/sec
MAX_SPEED = 800.0 #mm/sec
ACCELERATION_LIMIT = 250 #mm/sec^2
DRAG = 0.96
BACK_SENSOR_OFFSET = 20 #mm, distance from centerline of car to back sensors
SIDE_SENSOR_OFFSET = 50 #mm, distance from centerline of car to side sensors

DIRECTIONS = {
"FRONT": 0,
"LEFT": np.pi/2,
"BACK": np.pi,
"RIGHT": 3 * np.pi/2
}

class Car_Old:
     
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

        self.sensors = ( 
            PID(LENGTH/2 - SIDE_SENSOR_OFFSET, -WIDTH//2, DIRECTIONS["LEFT"], self),
            PID(LENGTH/2 - SIDE_SENSOR_OFFSET, WIDTH//2, DIRECTIONS["RIGHT"], self),
            PID(0, WIDTH/2 + BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self),
            PID(0, WIDTH/2 - BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self),
        )

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
        print(self.dt)

    def PerSecondToPerCycle(self, num_per_sec):
        return num_per_sec * self.dt

    def PerCycleToPerSecond(self, num_per_cycle):
        return num_per_cycle / self.dt

    def Draw(self, img):
        car_rect = ((self.x, self.y), CAR_SIZE, (-180/np.pi * self.dir))
        Arena.RotatedRect(img, car_rect, (80,80,80))
        for wheel_rect in self.wheel_rects:
            Arena.RotatedRect(img, wheel_rect, (0,0,0))

    def DriveDigital(self, direction):
        self.speed = self.speed * DRAG
        if direction == "FORWARDS":
            self.DriveAnalog(MAX_SPEED)
        elif direction == "BACKWARDS":
            self.DriveAnalog(-MAX_SPEED)

    def DriveAnalog(self, target_speed):
        inst_accel_limit = self.PerSecondToPerCycle(self.PerSecondToPerCycle(ACCELERATION_LIMIT))
        inst_speed = self.PerSecondToPerCycle(self.speed)
        inst_target = self.PerSecondToPerCycle(target_speed)
        print(f"inst_speed: {inst_speed}, inst_accel_limit: {inst_accel_limit}, target: {inst_target}")

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

    def TurnDigital(self, direction):
        if direction == "LEFT":
            self.TurnAnalog(self.max_wheel_dir)
        elif direction == "RIGHT":
            self.TurnAnalog(-self.max_wheel_dir)

    def TurnAnalog(self, angle):
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

    def DistTo(self, x, y):
        dx = x - self.x
        dy = y - self.y 
        return np.sqrt(dx**2 + dy**2)

    def MoveTo(self, targetX, targetY, targetDirection): 
        AGGRESSIVENESS = 3
        SLOW_DIST = TURNING_RADIUS #mm
        BACKUP_DIST = 20
        dx = targetX - self.x
        dy = targetY - self.y
        dtheta = ((targetDirection - self.dir + np.pi) % (2 * np.pi)) - np.pi
        dist = self.DistTo(targetX, targetY)
        closeness = max(0.01, min(1, (SLOW_DIST / dist)))
        
        
        target_wheel_dir = closeness * dtheta * AGGRESSIVENESS
        target_wheel_dir = (target_wheel_dir + np.pi) % (2 * np.pi) - np.pi #in range -pi to pi

        delta_cross = (dx / dist) * np.sin(self.dir) - (dy / dist) * np.cos(self.dir) # 0 if in line, 1 if out of line
        dir_cross = np.cos(targetDirection) * np.sin(self.dir) - np.sin(targetDirection) * np.cos(self.dir)
        parallelness = abs(delta_cross) * (1 - abs(dir_cross))
        side = np.sign(delta_cross)
        dot = np.cos(targetDirection) * dx - np.sin(targetDirection) * dy

        if self.speed < 0:
            target_wheel_dir = (parallelness**2) * side * closeness * AGGRESSIVENESS + (1 - parallelness**2) * target_wheel_dir
        
        # have we overshot?
        
        if dot >= 0:
            target_speed = min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist * 0.85)) 
        else:
            target_speed = -min(MAX_SPEED, np.sqrt(2*ACCELERATION_LIMIT * dist * 0.85)) 

        

        if self.speed < 0:
            target_wheel_dir = target_wheel_dir * -1


        if dist < 50 and abs(delta_cross) > 0.1 and abs(dir_cross) < 0.05 and self.speed < 0: #if we're reversing because we missed the target
            self.MoveTo(targetX - BACKUP_DIST * np.cos(targetDirection), targetY + BACKUP_DIST * np.sin(targetDirection), targetDirection)

        print(f"dist to target: {dist}")
        self.TurnAnalog(target_wheel_dir)
        self.DriveAnalog(target_speed)



    def DoAutomation(self, generated_arena, arena, colouredStartRects, verifiedColourZones, parkedCarRects, target_colours, real):
        if self.state == "UNINITIALISED":
            self.LocateOnTrack(generated_arena)
            self.state = "FINDING_START_COLOURS"
            
        elif self.state == "FINDING_START_COLOURS":
            self.MoveTo(arena.start_rects[0][0][0], arena.start_pos[1], 0)
            self.FindTargetColourRects(generated_arena, arena, colouredStartRects, target_colours, real)
            self.SenseCars(generated_arena, arena, parkedCarRects, real)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(colouredStartRects) > 0:
               self.state = "SEARCHING_FOR_TARGET"
               
        elif self.state == "SEARCHING_FOR_TARGET":
            target_x = Arena.TOTAL_WIDTH - generated_arena.start_pos[0]
            target_y = generated_arena.start_pos[1]
            self.MoveTo(target_x, target_y, 0)
            self.FindTargetColourRects(generated_arena, arena, colouredStartRects, target_colours, real)
            self.FindMatchingParks(generated_arena, arena, verifiedColourZones, colouredStartRects, real)
            self.SenseCars(generated_arena, arena, parkedCarRects, real)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(verifiedColourZones) > 0:
                self.state = "PREPARING_TO_PARK"
            elif self.DistTo(target_x, target_y) < 10:
                self.state = "DONE"
                
        elif self.state == "PREPARING_TO_PARK":
            self.FindMatchingParks(generated_arena, arena, verifiedColourZones, colouredStartRects, real)
            self.SenseCars(generated_arena, arena, parkedCarRects, real)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(verifiedColourZones) == 0:
                self.state = "SEARCHING_FOR_TARGET"
                return
            rects = sorted(
                list(verifiedColourZones.keys()), 
                key=lambda rect: rect[0][0]
            )
            target_x = int((rects[0][0][0] + rects[0][1][0]) // 2) - TURNING_RADIUS * 1.23
            target_y = generated_arena.start_pos[1]
            self.MoveTo(target_x, target_y, 0)
            print(self.DistTo(target_x, target_y))
            if self.DistTo(target_x, target_y) < 15 and abs(self.speed) < 5 and self.speed > 0:
                self.state = "PARKING"

        elif self.state == "PARKING":
            rects = sorted(
                list(verifiedColourZones.keys()), 
                key=lambda rect: rect[0][0]
            )
            target_x = int((rects[0][0][0] + rects[0][1][0]) // 2)
            target_y = Arena.TOTAL_HEIGHT - Arena.PARK_STANDOFF - LENGTH // 2
            self.MoveTo(target_x, target_y, np.pi* 3 / 2)
            if self.DistTo(target_x, target_y) < 5: 
                self.state = "DONE"

        print(self.state)

    def LocateOnTrack(self, arena):
        left_dist = self.sensors[0].SenseSimDist(arena)
        right_dist = self.sensors[1].SenseSimDist(arena)
        back_L_dist = self.sensors[2].SenseSimDist(arena)
        back_r_dist = self.sensors[3].SenseSimDist(arena)

        dir = np.arcsin((back_L_dist - back_r_dist) / (2 * BACK_SENSOR_OFFSET))
        center_x = (back_L_dist + back_r_dist) / 2 + LENGTH/2 * np.cos(dir)

        left_y_dist = left_dist * np.cos(dir)
        right_y_dist = right_dist * np.cos(dir)

        center_L_Y = left_y_dist - SIDE_SENSOR_OFFSET * np.sin(dir)
        center_R_Y = Arena.TRACK_WIDTH - right_y_dist + SIDE_SENSOR_OFFSET * np.sin(dir)
        center_y = (center_L_Y + center_R_Y) / 2
        (self.x, self.y, self.dir) = (center_x, center_y, dir)

    def SenseColourZones(self, generated_arena, arena, colour_zones, verified_colour_zones, target_colours, real):
        for angle in self.camera_rays:
            colour = (-1,-1,-1) # unused colour
            if real:
                colour = self.camera.SenseRealColour(angle)
            else:
                colour = self.camera.SenseSimColour(generated_arena, angle)
            for target_colour in target_colours:
                print(f"{colour}, {target_colour}, {np.array_equal(colour, target_colour)}")
                if np.array_equal(colour, target_colour):
                    (x, y) = self.camera.FindRayIntercept(generated_arena.img, angle)
                    for (pt1, pt2) in colour_zones:
                        if (pt1[0] <= x <= pt2[0]) and (pt1[1] <= y <= pt2[1]):
                            Arena.Rect(arena.img, pt1, pt2, colour)
                            verified_colour_zones[(pt1, pt2)] = colour
        return verified_colour_zones

    def FindMatchingParks(self, generated_arena, arena, valid_parks, coloured_start_rects, real):
        print(list(coloured_start_rects.values()))
        self.SenseColourZones(generated_arena, arena, arena.colour_rects, valid_parks, list(coloured_start_rects.values()), real)

    def FindTargetColourRects(self, generated_arena, arena, coloured_start_rects, targetColours, real):
        self.SenseColourZones(generated_arena, arena, arena.start_rects, coloured_start_rects, list(targetColours.values()), real)

    def SenseCars(self, generated_arena, arena, full_car_rects, real):
        self.SenseColourZones(generated_arena, arena, arena.car_rects, full_car_rects, [(0,0,0)], real)

    def CheckParksAreEmpty(self, verified_colour_zones, full_car_rects):
        zonesToInvalidate = []
        for zone in verified_colour_zones.keys():
            for car in full_car_rects.keys():
                if car[0][0] < zone[0][0] and car[1][0] > zone[1][0]:
                    zonesToInvalidate.append(zone)
        for zone in zonesToInvalidate:
            if zone in verified_colour_zones:
                del verified_colour_zones[zone]