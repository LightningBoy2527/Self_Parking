import requests
import cv2 as cv
import numpy as np
import time
import multiprocessing.shared_memory as shared
#import asyncio

cameraURL = "http://tacacam.local"

targetFPS = 10

frameInt = 1.0/targetFPS #1 second (to translate fps)

def setCameraSettings(variable, value):
    try:
        #get sents a tcp request to the specified url. it returns a response object
        response = requests.get( f"{cameraURL}/control", params = {"var": variable, "val": value}, timeout = 2)
        #response is a http request (containing a byte stream of content)
        response.raise_for_status() #this checks if the http response is valid
        print("set", variable, "to", value)
        return True
    except requests.RequestException as e:
        print(f"error setting {e}")
        return False
    
def ConfigureCamera():
    #set the camera up or return error
    #only sets frame size atm
    print("\nconfiguring ESP-32 CAM...")
    try:
        setCameraSettings("framesize", 3) #176x144
        setCameraSettings("brightness", -2)
        setCameraSettings("contrast", 0)
        setCameraSettings("saturation", 3)
        setCameraSettings("ae_level", -1) #assuming this stands for auto-exposure level
    except:
        print("camera failed to set up")
    
def CaptureFrame():
    try:
        #get http request from server with image as bytes
        response = requests.get(f"{cameraURL}/capture", timeout = 2) #triggers a capture from the web
        response.raise_for_status() #check for returned http request format error
        #from the buffer of returned http request, get the bytes as uint8 array
        imageArray = np.frombuffer(response.content, dtype = np.uint8)
        #decode the uint8 array into an image
        frame = cv.imdecode(imageArray, cv.IMREAD_COLOR)
        return frame
    except:
        print("error in CaptureFrame")
        
        
def main():
    #startup procedure
    ConfigureCamera()
    time.sleep(0.5)
    frameCount = 0
    fpsStartTime = time.perf_counter()
    actualFps = 0.0
    nextCaptureTime = time.perf_counter()
    #set up out interprocess global data
    #shared_space = shared.SharedMemory(name='camera_img', create=True, size=1000) #we create the shared memeory here
    
    #continuously run to capture frames
    while True:
        #starts counting (outside of program)
        now = time.perf_counter()
        if  now < nextCaptureTime:
            time.sleep(nextCaptureTime-now)
        frame = CaptureFrame()
        captureEnd = time.perf_counter()
        if frame  is None:
            print("failed to capture frame")
            nextCaptureTime = time.perf_counter() + frameInt
            continue
        frameCount += 1
        elapsed = captureEnd - fpsStartTime
        if elapsed >= 1.0:
            frameCount = 0
            fpsStartTime = captureEnd
        cv.imwrite("ESPCamera.bmp", frame)
        cv.waitKey(1)
        nextCaptureTime += frameInt
        if nextCaptureTime < time.perf_counter():
            nextCaptureTime = time.perf_counter()
            
main()        