from pydualsense import pydualsense
import Car
import cv2 as cv
import numpy as np


def FindControllerXY(ds):
    x = ds.state.RX
    y = ds.state.LY

    if abs(x) < 10:
        x = -0.5
    if abs(y) < 10:
        y = -0.5


    pct_x = (x + 0.5) / 127.5
    pct_y = -(y + 0.5) / 127.5

    car_lr = np.pi/7 * pct_x
    car_fb = Car.MAX_SPEED * pct_y
    print(f"left/right: {car_lr}, foward/back: {car_fb}")
    return (car_lr, car_fb)


def RunManually(arena, generated_arena, car, real):
    ds = pydualsense() # open controller
    ds.init() # initialize controller
    ds.light.setColorI(80,0,255) # set touchpad color to purple

    while (not ds.state.cross):
        car.UpdateClockDiff()
        displayArena = generated_arena.img.copy()
        sensedArena = arena.img.copy()
        (steer, throttle) = FindControllerXY(ds)
        # if real:
        #     RealCar.Drive(throttle)
        #     RealCar.Turn(steer)
        car.Turn(steer)
        car.Drive(throttle)
        car.Move(real)
        car.Draw(displayArena)
        car.Draw(sensedArena)
        if ds.state.circle:
            cv.imshow("preview", displayArena)
            cv.imshow("sensed", sensedArena)

def MoveManually(car):
    ds = pydualsense() # open controller
    ds.init() # initialize controller
    ds.light.setColorI(80,0,255) # set touchpad color to purple

    (steer, throttle) = FindControllerXY(ds)
    car.Turn(steer)
    car.Drive(throttle)
