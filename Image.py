import cv2 as cv
import numpy as np
import sys
import ColorCheck as Colours

#constants from Colours
SCAN_THICKNESS = Colours.SCAN_THICKNESS
VIEW_ANGLE_D = Colours.CAMERA_VIEW_ANGLE
SCAN_HEIGHT = Colours.SCAN_HEIGHT
SCAN_WIDTH = Colours.SCAN_WIDTH
SCAN_OFFSET = Colours.SCAN_OFFSET
prev_img = np.array([]) #the previous image incase of read error

def ReadImage():
    global prev_img
    try:
        #if wanting hsv instead of bgr, change cv.IMREAD
        image = cv.imread("ESPCamera.bmp", cv.IMREAD_COLOR_BGR)
        image = cv.rotate(image, cv.ROTATE_90_COUNTERCLOCKWISE)
        if (image is None):
            print("detected None image")
            return prev_img
        prev_img = image #for saving errors
        DrawRectangles(image.copy())
        return image
    except Exception as e:
        exception_type, exception_object, traceback = sys.exc_info()
        line = traceback.tb_lineno
        print("Error in ReadImage", e, "at ", line)
        return prev_img
    
#Method: slice an input image into the thin stip
#divide the input pixels into 120 degrees by using .shape
#get the desired pixels
#output idealized color depending on avg pixel
def SenseRealColour(angle_r, image):
    #translate from +/- pi to angle_d
    #if angle_r is less than zero, do 90 - angle_r
    # if angle_r < 0:
    #     angle_d = VIEW_ANGLE_D/2 + angle_r*180/np.pi
    # else:
        
    angle_d = VIEW_ANGLE_D/2 - angle_r*180/np.pi
    
    try:
        #get image size
        height, width, channels = image.shape
        
        #more positive means lower down
        lower_height = int(height/2 - SCAN_THICKNESS/2 + SCAN_HEIGHT)
        upper_height = int(height/2 + SCAN_THICKNESS/2 + SCAN_HEIGHT)
        
        angle_d_step = width/VIEW_ANGLE_D
        #when writing this I assumed positive angle_d is cw
        #cv does row column, so positive means more right
        right_edge = int(angle_d_step*angle_d + SCAN_WIDTH - SCAN_OFFSET)
        left_edge = int(angle_d_step*angle_d - SCAN_WIDTH - SCAN_OFFSET)
        
        #take a slice of input image that is in the area ww want
        image_slice = image[lower_height:upper_height, left_edge:right_edge]
        
        cv.imshow("my tiny slice", image_slice)
        
        
        #using CarLogicCamera
        #this function returns true if the input image has contour of said size and is within range
        result = Colours.CheckColours(image_slice)
        #print(result)
        return result
        
    except Exception as e:
        #track the place an error occured
        exception_type, exception_object, traceback = sys.exc_info()
        line = traceback.tb_lineno
        print("Error in SenseRealColour: ",  e, "at ", line)
        
def DrawRectangles(image):
    START_SCAN_ANGLE = Colours.START_SCAN_ANGLE
    CAMERA_VIEW_ANGLE = Colours.CAMERA_VIEW_ANGLE
    
    BLOCK_SEPARATION = Colours.BLOCK_SEPARATION
    BLOCK_WIDTH = Colours.BLOCK_WIDTH
    SCAN_THICKNESS = Colours.SCAN_THICKNESS
    
    height, width, channels = image.shape
    scan_angle = int(START_SCAN_ANGLE*width/CAMERA_VIEW_ANGLE)
    #start rectangles
    cv.rectangle(image, [int(scan_angle-BLOCK_WIDTH-BLOCK_SEPARATION), int(height/2+SCAN_THICKNESS)], [int(scan_angle-BLOCK_SEPARATION), int(height/2-SCAN_THICKNESS)], [255, 0, 0])
    cv.rectangle(image, [int(scan_angle+BLOCK_WIDTH+BLOCK_SEPARATION), int(height/2+SCAN_THICKNESS)], [int(scan_angle+BLOCK_SEPARATION), int(height/2-SCAN_THICKNESS)], [0, 0, 255])
    
    #lower height
    cv.line(image, (0, int(height/2 - SCAN_THICKNESS/2 + SCAN_HEIGHT)), (width, int(height/2 - SCAN_THICKNESS/2 + SCAN_HEIGHT)), (0, 0, 255))
    #upper height
    cv.line(image, (0, int(height/2 + SCAN_THICKNESS/2 + SCAN_HEIGHT)), (width, int(height/2 + SCAN_THICKNESS/2 + SCAN_HEIGHT)), (0, 0, 255))
    cv.imshow("Showing", image)
    
    