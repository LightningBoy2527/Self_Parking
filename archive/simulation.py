import cv2 as cv
import numpy as np
import random as rng

DIRECTIONS = {
    "FRONT": 0,
    "LEFT": np.pi/2,
    "BACK": np.pi,
    "RIGHT": 3 * np.pi/2
} 

COLOURS = { #BGR
    "RED": [0, 0, 255],
    "GREEN": [0, 255, 0],
    "BLUE": [255, 0, 0],
    "YELLOW": [0, 255, 255]
}

START_LENGTH = 345 # mm
TRACK_WIDTH = 268 # mm, excluding parks
PARK_BUFFER = 20 
PARK_STANDOFF = 14 #mm between bumper and wall
PARK_WIDTH = 120 # mm
PARK_DEPTH = 227 # mm
PARK_COUNT = 8
OCCUPIED_PARKS = 4
BORDER_WIDTH = 20 #px
TOTAL_WIDTH = START_LENGTH * 2 + PARK_BUFFER * 2 + PARK_COUNT * PARK_WIDTH + BORDER_WIDTH * 2
TOTAL_HEIGHT = TRACK_WIDTH + PARK_DEPTH + BORDER_WIDTH * 2

def PopulateArena(emptyArena, occupiedParkCount, colourRects, carRects):
    arena = emptyArena
    colourslist = list(COLOURS.values())

    for (pt1, pt2) in colourRects:
        cv.rectangle(arena, pt1, pt2, rng.choice(colourslist), cv.FILLED)
    
    occupiedCarRects = rng.sample(carRects, k=occupiedParkCount)

    for (pt1, pt2) in occupiedCarRects:
        cv.rectangle(arena, pt1, pt2, (0,0,0), cv.FILLED)


def ConstructEmptyArena(populated = True):
    
    CAR_WIDTH = 80 #mm, parked cars
    HALF_CAR_W = int(CAR_WIDTH/2)
    CAR_HEIGHT = 180 #mm, parked cars
    HALF_CAR_H = int(CAR_HEIGHT/2)
    RECT_WIDTH = 64 #mm
    HALF_RECT_WIDTH = int(RECT_WIDTH/2)
    RECT_DEPTH = 20 #px

    #use 1px = 1mm
    
    #center of car starting position
    startPos = (BORDER_WIDTH + 23 + CAR_HEIGHT//2, BORDER_WIDTH + TRACK_WIDTH//2)

    carRects = []
    colouredRects = []
    startRects = [
        (
            (START_LENGTH - BORDER_WIDTH - RECT_WIDTH, TRACK_WIDTH + BORDER_WIDTH),
            (START_LENGTH - BORDER_WIDTH - RECT_WIDTH // 2 - 1, TRACK_WIDTH + BORDER_WIDTH + RECT_DEPTH),
        ),
        (
            (START_LENGTH - BORDER_WIDTH - RECT_WIDTH // 2, TRACK_WIDTH + BORDER_WIDTH),
            (START_LENGTH - BORDER_WIDTH, TRACK_WIDTH + BORDER_WIDTH + RECT_DEPTH),
        ),
        # (
        #     (TOTAL_WIDTH - (START_LENGTH - BORDER_WIDTH), TRACK_WIDTH + BORDER_WIDTH),
        #     (TOTAL_WIDTH - (START_LENGTH - BORDER_WIDTH - RECT_WIDTH // 2 - 1), TRACK_WIDTH + BORDER_WIDTH + RECT_DEPTH),
        # ),
        # (
        #     (TOTAL_WIDTH - (START_LENGTH - BORDER_WIDTH - RECT_WIDTH // 2), TRACK_WIDTH + BORDER_WIDTH),
        #     (TOTAL_WIDTH - (START_LENGTH - BORDER_WIDTH - RECT_WIDTH), TRACK_WIDTH + BORDER_WIDTH + RECT_DEPTH),
        # ),
    ]

    for i in range(PARK_COUNT):
        parkCenterX = int(START_LENGTH + BORDER_WIDTH + PARK_BUFFER + PARK_WIDTH * (i+0.5))
        parkCenterY = int(TRACK_WIDTH + BORDER_WIDTH + PARK_DEPTH - PARK_STANDOFF - HALF_CAR_H)
        parkBottom = TOTAL_HEIGHT - BORDER_WIDTH

        carRects.append(((parkCenterX - HALF_CAR_W, parkCenterY - HALF_CAR_H), (parkCenterX + HALF_CAR_W, parkCenterY + HALF_CAR_H)))
        colouredRects.append(((parkCenterX - HALF_RECT_WIDTH, parkBottom),(parkCenterX + HALF_RECT_WIDTH, parkBottom + RECT_DEPTH)))


    emptyArena = np.zeros((TOTAL_HEIGHT, TOTAL_WIDTH, 3), dtype= np.uint8)
    cv.rectangle(emptyArena, (0, 0), (TOTAL_WIDTH, TOTAL_HEIGHT), (181, 228, 255), cv.FILLED)

    cv.rectangle(emptyArena, (BORDER_WIDTH, BORDER_WIDTH), (TOTAL_WIDTH - BORDER_WIDTH, TRACK_WIDTH + BORDER_WIDTH), (255,255,255), cv.FILLED)
    cv.rectangle(emptyArena, (BORDER_WIDTH + START_LENGTH, BORDER_WIDTH + TRACK_WIDTH), (TOTAL_WIDTH - (BORDER_WIDTH + START_LENGTH), TOTAL_HEIGHT - BORDER_WIDTH), (255,255,255), cv.FILLED)

    if populated:
        keyCode = ord('r')
        while keyCode == ord('r'):
            arena = emptyArena.copy()
            PopulateArena(arena, OCCUPIED_PARKS, colouredRects + startRects, carRects)
            cv.imshow("preview", arena)
            keyCode = cv.waitKey(0)
    else:
            arena = emptyArena.copy()

    return (arena, startPos, startRects, colouredRects, carRects)


class Sensor:
    def __init__(self, x, y, direction, parent):
        self.relX = x
        self.relY = y
        self.relDirection = direction
        self.parent = parent

    @property
    def x(self):
        return self.parent.centerX + self.relX * np.cos(self.parent.direction) + self.relY * np.sin(self.parent.direction)

    @property
    def y(self):
        return self.parent.centerY + self.relX * np.sin(self.parent.direction) - self.relY * np.cos(self.parent.direction)

    @property
    def direction(self):
        return (self.relDirection + self.parent.direction) % (2 * np.pi)

    
    def FindRayIntercept(self, arena):
        startX = self.x
        startY = self.y
        COARSE_DIST = 15 #px
        FINE_DIST = 1 #px

        COARSE_X = COARSE_DIST * np.cos(self.direction)
        COARSE_Y = -COARSE_DIST * np.sin(self.direction)

        FINE_X = FINE_DIST * np.cos(self.direction)
        FINE_Y = -FINE_DIST * np.sin(self.direction)

        scanX = startX + COARSE_X
        scanY = startY + COARSE_Y

        while (0 <= int(scanY) < arena.shape[0] and 0 <= int(scanX) < arena.shape[1] 
               and np.all(arena[int(scanY), int(scanX)] == (255, 255, 255))):
            scanX = scanX + COARSE_X
            scanY = scanY + COARSE_Y

        scanX = scanX - COARSE_X
        scanY = scanY - COARSE_Y

        while (0 <= int(scanY) < arena.shape[0] and 0 <= int(scanX) < arena.shape[1] 
               and np.all(arena[int(scanY), int(scanX)] == (255, 255, 255))):
            scanX = scanX + FINE_X
            scanY = scanY + FINE_Y
        
        finalX = max(0, min(int(scanX), arena.shape[1] - 1))
        finalY = max(0, min(int(scanY), arena.shape[0] - 1))
        
        return (finalX, finalY)

    def SenseDist(self, arena):
        (x, y) = self.FindRayIntercept(arena)
        dx = x - self.x
        dy = y - self.y
        dist = np.sqrt(dx**2 + dy**2)
        return dist

    def SenseColour(self, arena):
        (x, y) = self.FindRayIntercept(arena)
        colour = arena[y, x]
        return colour

class Car:
    CAR_WIDTH = 80 #mm
    CAR_LENGTH = 179 #mm
    CAR_SIZE = (CAR_LENGTH, CAR_WIDTH)
    WHEEL_SIZE = (23, 15)
    TURNING_RADIUS = 245 # mm

    #distances are from back right corner. 0, 0 (car coordinates) is at the back right corner, x extends positive to the front of the car, y extends positive to the right of the car.
    REAR_AXLE_POS = 20 #mm
    WHEEL_INSET = 20 #mm
    STEERING_AXLE_OFFSET = 8 #mm
    STEERING_SPEED = 0.1 #rad/call
    MAX_SPEED = 10.0 #mm/call
    ACCELERATION_LIMIT = 1.0 #mm/call^2
    DRAG = 0.96
    BACK_SENSOR_OFFSET = 20 #mm, distance from centerline of car to back sensors
    SIDE_SENSOR_OFFSET = 50 #mm, distance from centerline of car to side sensors

    def __init__(self, centerX, centerY, direction):
        self.centerX = centerX
        self.centerY = centerY
        self.direction = direction
        self.speed = 0
        self.steeringAngle = 0
        self.wheelbase = 120 
        self.state = "UNINITIALISED"

        self.sensors = ( 
            Sensor(self.CAR_LENGTH/2 - self.SIDE_SENSOR_OFFSET, -self.CAR_WIDTH//2, DIRECTIONS["LEFT"], self),
            Sensor(self.CAR_LENGTH/2 - self.SIDE_SENSOR_OFFSET, self.CAR_WIDTH//2, DIRECTIONS["RIGHT"], self),
            Sensor(0, self.CAR_WIDTH/2 + self.BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self),
            Sensor(0, self.CAR_WIDTH/2 - self.BACK_SENSOR_OFFSET, DIRECTIONS["BACK"], self),
        )

        self.cameraRays = (
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] + np.pi/5, self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] + np.pi/6, self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] + np.pi/8, self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"], self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] - np.pi/8, self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] - np.pi/6, self),
            Sensor(self.CAR_LENGTH - 10, 10, DIRECTIONS["RIGHT"] - np.pi/5, self),
        )

    @property
    def xOffset(self): 
        return self.centerX + (-self.CAR_LENGTH * np.cos(self.direction) + self.CAR_WIDTH *np.sin(self.direction)) / 2

    @property
    def yOffset(self): 
        return self.centerY + (self.CAR_LENGTH * np.sin(self.direction) + self.CAR_WIDTH *np.cos(self.direction)) / 2

    def relToAbs(self, x, y):
        absX = self.xOffset + x * np.cos(self.direction) - y * np.sin(self.direction)
        absY = self.yOffset - x * np.sin(self.direction) - y * np.cos(self.direction)
        return (absX, absY)

    @property
    def wheelRects(self):
        return [
            (self.relToAbs(self.REAR_AXLE_POS, self.WHEEL_INSET), self.WHEEL_SIZE, -180/np.pi * (self.direction - self.steeringAngle)),
            (self.relToAbs(self.REAR_AXLE_POS, self.CAR_WIDTH-self.WHEEL_INSET), self.WHEEL_SIZE, -180/np.pi * (self.direction - self.steeringAngle)),
            (self.relToAbs(self.REAR_AXLE_POS + self.wheelbase, self.WHEEL_INSET), self.WHEEL_SIZE, -180/np.pi * self.direction),
            (self.relToAbs(self.REAR_AXLE_POS + self.wheelbase, self.CAR_WIDTH-self.WHEEL_INSET), self.WHEEL_SIZE, -180/np.pi * self.direction),
        ]

    @property
    def center (self):
        return (self.centerX, self.centerY)

    @property
    def maxTurningAngle(self):
        return np.atan(self.wheelbase / self.TURNING_RADIUS)

    def Draw(self, arena):
        carRect = (self.center, self.CAR_SIZE, (-180/np.pi * self.direction))
        carPoints = np.intp(cv.boxPoints(carRect))
        cv.fillConvexPoly(arena, carPoints, (40,40,40))
        for wheelRect in self.wheelRects:
            wheelPoints = np.intp(cv.boxPoints(wheelRect))
            cv.fillConvexPoly(arena, wheelPoints, (0,0,0))

    def DriveDigital(self, direction):
        self.speed = self.speed * self.DRAG
        if direction == "FORWARDS":
            self.DriveAnalog(self.MAX_SPEED)
        elif direction == "BACKWARDS":
            self.DriveAnalog(-self.MAX_SPEED)

    def DriveAnalog(self, targetSpeed):
        if targetSpeed > self.speed:
            self.speed = self.speed + self.ACCELERATION_LIMIT
            if targetSpeed < self.speed:
                self.speed = targetSpeed
        elif targetSpeed < self.speed:
            self.speed = self.speed - self.ACCELERATION_LIMIT
            if targetSpeed > self.speed:
                self.speed = targetSpeed

        if self.speed > self.MAX_SPEED:
            self.speed = self.MAX_SPEED
        elif self.speed < -self.MAX_SPEED:
            self.speed = -self.MAX_SPEED

    def TurnDigital(self, direction):
        if direction == "LEFT":
            self.TurnAnalog(self.maxTurningAngle)
        elif direction == "RIGHT":
            self.TurnAnalog(-self.maxTurningAngle)

    def TurnAnalog(self, angle):
        oldAngle = self.steeringAngle
        if abs(angle) > self.STEERING_SPEED:
            if angle > 0:
                angle = self.STEERING_SPEED
            else:
                angle = -self.STEERING_SPEED

        if self.steeringAngle + angle > self.maxTurningAngle:
            self.steeringAngle = self.maxTurningAngle
        elif self.steeringAngle + angle < -self.maxTurningAngle:
            self.steeringAngle = -self.maxTurningAngle
        else:
            self.steeringAngle += angle

        if oldAngle != self.steeringAngle:
            delta = self.steeringAngle - oldAngle
            self.wheelbase = 120 + self.STEERING_AXLE_OFFSET * np.cos(self.steeringAngle) - self.STEERING_AXLE_OFFSET
            relXWheelMovement = self.STEERING_AXLE_OFFSET * np.cos(delta) - self.STEERING_AXLE_OFFSET
            relYWheelMovement = self.STEERING_AXLE_OFFSET * np.sin(delta)

            interpolationConstant = ((self.REAR_AXLE_POS + self.wheelbase - self.CAR_LENGTH/2)/self.wheelbase)
            relXCenterMovement = relXWheelMovement * interpolationConstant
            relYCenterMovement = relYWheelMovement * interpolationConstant
            absXCenterMovement = -(relXCenterMovement * np.cos(self.direction) - relYCenterMovement * np.sin(self.direction))
            absYCentreMovement = -(relXCenterMovement * np.sin(self.direction) + relYCenterMovement * np.cos(self.direction))

            self.centerX = self.centerX - absXCenterMovement
            self.centerY = self.centerY - absYCentreMovement
            self.direction = self.direction + np.arctan2(relYWheelMovement, self.wheelbase)

    def Move(self):
        self.centerX = self.centerX + self.speed * np.cos(self.direction - self.steeringAngle/2)
        self.centerY = self.centerY - self.speed * np.sin(self.direction - self.steeringAngle/2)
        self.direction = self.direction + self.speed/self.wheelbase * np.tan(self.steeringAngle)

    def DistTo(self, x, y):
        dx = x - self.centerX
        dy = y - self.centerY 
        return np.sqrt(dx**2 + dy**2)

    def MoveTo(self, targetX, targetY, targetDirection): 
        SLOW_DIST = 450 #mm
        dx = targetX - self.centerX
        dy = targetY - self.centerY
        dtheta = ((targetDirection - self.direction + np.pi) % (2 * np.pi)) - np.pi
        dist = self.DistTo(targetX, targetY)
        closeness = max(0.0, min(1.0, 1.0 - (dist / SLOW_DIST) * 0.2))
        

        targetSteeringAngle = closeness * dtheta
        targetSteeringAngle = (targetSteeringAngle + np.pi) % (2 * np.pi) - np.pi #in range -pi to pi
        
        
        # have we overshot?
        dot = np.cos(targetDirection) * dx - np.sin(targetDirection) * dy
        if dot >= 0:
            targetSpeed = 5/(closeness + 0.1)
        else:
            targetSpeed = -5/(closeness + 0.1) #-dist**1.001/10
            targetSteeringAngle = targetSteeringAngle * -1

        self.TurnAnalog(targetSteeringAngle)
        self.DriveAnalog(targetSpeed)



    def DoAutomation(self, arena, activeArena, startRects, colouredStartRects, colourZones, verifiedColourZones, carRects, parkedCarRects):
        if self.state == "UNINITIALISED":
            self.LocateOnTrack(arena)
            self.state = "FINDING_START_COLOURS"
            
        elif self.state == "FINDING_START_COLOURS":
            self.MoveTo(startRects[0][0][0], BORDER_WIDTH + TRACK_WIDTH//2, 0)
            self.FindTargetColours(arena, activeArena, startRects, colouredStartRects)
            self.SenseCars(arena, activeArena, carRects, parkedCarRects)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(colouredStartRects) > 0:
               self.state = "SEARCHING_FOR_TARGET"
               
        elif self.state == "SEARCHING_FOR_TARGET":
            targetX = TOTAL_WIDTH - startPos[0]
            targetY = BORDER_WIDTH + TRACK_WIDTH//2
            self.MoveTo(targetX, targetY, 0)
            self.FindTargetColours(arena, activeArena, startRects, colouredStartRects)
            self.SenseColourZones(arena, activeArena, colourZones, verifiedColourZones, list(colouredStartRects.values()))
            self.SenseCars(arena, activeArena, carRects, parkedCarRects)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(verifiedColourZones) > 0:
                self.state = "PREPARING_TO_PARK"
            elif self.DistTo(targetX, targetY) < 20:
                self.state = "DONE"
                
        elif self.state == "PREPARING_TO_PARK":
            self.SenseColourZones(arena, activeArena, colourZones, verifiedColourZones, list(colouredStartRects.values()))
            self.SenseCars(arena, activeArena, carRects, parkedCarRects)
            self.CheckParksAreEmpty(verifiedColourZones, parkedCarRects)
            if len(verifiedColourZones) == 0:
                self.state = "SEARCHING_FOR_TARGET"
                return
            rects = sorted(
                list(verifiedColourZones.keys()), 
                key=lambda rect: rect[0][0]
            )
            targetX = int((rects[0][0][0] + rects[0][1][0]) // 2) - self.TURNING_RADIUS * 1.15
            targetY = BORDER_WIDTH + TRACK_WIDTH // 2
            self.MoveTo(targetX, targetY, 0)
            print(self.DistTo(targetX, targetY))
            if self.DistTo(targetX, targetY) < 3 and abs(self.speed < 2) :
                self.state = "PARKING"

        elif self.state == "PARKING":
            rects = sorted(
                list(verifiedColourZones.keys()), 
                key=lambda rect: rect[0][0]
            )
            targetX = int((rects[0][0][0] + rects[0][1][0]) // 2)
            targetY = TOTAL_HEIGHT - BORDER_WIDTH - PARK_STANDOFF - self.CAR_LENGTH // 2
            self.MoveTo(targetX, targetY, np.pi* 3 / 2)
            if self.DistTo(targetX, targetY) < 10: 
                self.state = "DONE"

        print(self.state)

    def LocateOnTrack(self, arena):
        leftDist = self.sensors[0].SenseDist(arena)
        rightDist = self.sensors[1].SenseDist(arena)
        backLDist = self.sensors[2].SenseDist(arena)
        backRDist = self.sensors[3].SenseDist(arena)

        direction = np.arcsin((backLDist - backRDist) / (2 * self.BACK_SENSOR_OFFSET))
        centerX = BORDER_WIDTH + (backLDist + backRDist) / 2 + self.CAR_LENGTH/2 * np.cos(direction)

        leftYDist = leftDist * np.cos(direction)
        rightYDist = rightDist * np.cos(direction)

        centerLeftY = BORDER_WIDTH + leftYDist - self.SIDE_SENSOR_OFFSET * np.sin(direction)
        centerRightY = BORDER_WIDTH + TRACK_WIDTH - rightYDist + self.SIDE_SENSOR_OFFSET * np.sin(direction)
        centerY = (centerLeftY + centerRightY) / 2
        (self.centerX, self.centerY, self.direction) = (centerX, centerY, direction)

    def FindTargetColours(self, arena, activeArena, startRects, colouredStartRects):

        self.SenseColourZones(arena, activeArena, startRects, colouredStartRects, list(COLOURS.values()))
            
        for startRect in colouredStartRects.keys():
            col_tuple = tuple(int(c) for c in colouredStartRects[startRect])
            cv.rectangle(activeArena, startRect[0], startRect[1], col_tuple, cv.FILLED)

        return colouredStartRects

    def SenseColourZones(self, arena, activeArena, colourZones, verifiedColourZones, targetColours):
        for ray in self.cameraRays:
            colour = ray.SenseColour(arena)
            for targetColour in targetColours:
                if np.array_equal(colour, targetColour):
                    (x, y) = ray.FindRayIntercept(arena)
                    for (pt1, pt2) in colourZones:
                        if (pt1[0] <= x <= pt2[0]) and (pt1[1] <= y <= pt2[1]):
                            col_tuple = tuple(int(c) for c in colour)
                            cv.rectangle(activeArena, pt1, pt2, col_tuple, cv.FILLED)
                            verifiedColourZones[(pt1, pt2)] = col_tuple
        return verifiedColourZones

    def SenseCars(self, arena, activeArena, carRects, fullCarRects):
        self.SenseColourZones(arena, activeArena, carRects, fullCarRects, [(0,0,0)])
        for rect in fullCarRects.keys():
            cv.rectangle(activeArena, rect[0], rect[1], (0,0,0), cv.FILLED)

    def CheckParksAreEmpty(self, verifiedColourZones, fullCarRects):
        zonesToInvalidate = []
        for zone in verifiedColourZones.keys():
            for car in fullCarRects.keys():
                if car[0][0] < zone[0][0] and car[1][0] > zone[1][0]:
                    zonesToInvalidate.append(zone)
        for zone in zonesToInvalidate:
            if zone in verifiedColourZones:
                del verifiedColourZones[zone]

def RunManually(arena, car):
    while (True):
        displayArena = arena.copy()

        car.Move()
        car.Draw(displayArena)
        cv.imshow("preview", displayArena)
        keyCode = cv.waitKey(30)
        
        if keyCode == ord('8'): 
            car.DriveDigital("FORWARDS")
            car.TurnDigital(None)
        elif keyCode == ord('2'): 
            car.DriveDigital("BACKWARDS")
            car.TurnDigital(None)
        elif keyCode == ord('4'):
            car.DriveDigital(None)
            car.TurnDigital("LEFT")
        elif keyCode == ord('6'):
            car.DriveDigital(None)
            car.TurnDigital("RIGHT")
        elif keyCode == ord('7'): 
            car.DriveDigital("FORWARDS")
            car.TurnDigital("LEFT")
        elif keyCode == ord('9'): 
            car.DriveDigital("FORWARDS")
            car.TurnDigital("RIGHT")
        elif keyCode == ord('1'): 
            car.DriveDigital("BACKWARDS")
            car.TurnDigital("LEFT")
        elif keyCode == ord('3'): 
            car.DriveDigital("BACKWARDS")
            car.TurnDigital("RIGHT")
        elif keyCode == ord('q'):
            break
        else:
            car.DriveDigital(None)
            car.TurnDigital(None)
    return

def RunAutonomously(arena, car):
    (activeArena, _, startRects, colourZones, carRects) = ConstructEmptyArena(populated = False)
    verifiedColourZones = {}
    colouredStartRects = {}
    parkedCars = {}
    while (car.state != "DONE"):
        displayArena = arena.copy()
        
        car.DoAutomation(arena, activeArena, startRects, colouredStartRects, colourZones, verifiedColourZones, carRects, parkedCars)
        
        sensedArena = activeArena.copy()
        car.Move()
        car.Draw(displayArena)
        car.Draw(sensedArena)
        cv.imshow("simulated", displayArena)
        cv.imshow("sensed", sensedArena)
        keyCode = cv.waitKey(1)

        if keyCode == ord('q'):
            break
    return


seed = 100
while True:
    rng.seed(seed)
    (arena, startPos, _, _, _) = ConstructEmptyArena()
    (x, y) = startPos
    car = Car(x, y, -np.pi/50)
    #RunManually(arena, car)
    RunAutonomously(arena, car)
    if cv.waitKey(0) != ord('r'):
        break
    else:
        seed = seed + 1

