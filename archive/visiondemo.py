import cv2 as cv
import numpy as np 


LOWER_SAT = 50
LOWER_VAL = 30
UPPER_SAT = 250
UPPER_VAL = 250

TARGET_H_AT_100MM = 100 #px

COLOURS = {
    "WHITE": (255, 255, 255),
    "RED": (0, 0, 255),
    "ORANGE": (0, 127, 255),
    "YELLOW": (0, 255, 255),
    "GREEN": (0, 255, 0),
    "AQUA": (255, 255, 0),
    "BLUE": (255, 0, 0),
    "PURPLE": (255, 0, 127),
    "PINK": (255, 0, 255),
}

COLOUR_RANGES = {
    "RED_LOW": {"MIN": 0, "MAX": 15},
    "ORANGE": {"MIN": 15, "MAX": 40},
    "YELLOW": {"MIN": 40, "MAX": 62},
    "GREEN": {"MIN": 62, "MAX": 165},
    "AQUA": {"MIN": 165, "MAX": 190},
    "BLUE": {"MIN": 190, "MAX": 255},
    "PURPLE": {"MIN": 255, "MAX": 280},
    "PINK": {"MIN": 280, "MAX": 335},
    "RED_HIGH": {"MIN": 335, "MAX": 360}
}


def Make_Colour_Frame(frame, colourMasks):
    colouredZones = Merge_Colour_Masks(frame, colourMasks)
    colouredFrame = CV_Combine(colouredZones)
    return colouredFrame


def Merge_Colour_Masks(frame, colourMasks):

    partialFrames = []
    for colour, value in COLOURS.items():

        mask = colourMasks[colour]
        partialFrame = CV_Mask_To_Frame(frame.shape, mask, value)

        partialFrames.append(partialFrame)
    return partialFrames


def Make_Colour_Masks(BGR_frame):
    HSV_Frame = cv.cvtColor(BGR_frame, cv.COLOR_BGR2HSV)
    masks = {}
    for colour in COLOURS:
        masks[colour] = Make_Colour_Mask(HSV_Frame, colour)
    return masks

def Make_Colour_Mask(frame, colourName):

    if colourName == "WHITE":
        mask = cv.inRange(frame, (0, 0, UPPER_VAL), (255, LOWER_SAT, 255))
    elif colourName == "GREEN":
        min = COLOUR_RANGES[colourName]["MIN"]
        max = COLOUR_RANGES[colourName]["MAX"]
        mask = cv.inRange(frame, (min/2, LOWER_SAT/2, LOWER_VAL/2), (max/2, UPPER_SAT, UPPER_VAL))
    elif colourName != "RED":
        min = COLOUR_RANGES[colourName]["MIN"]
        max = COLOUR_RANGES[colourName]["MAX"]
        mask = cv.inRange(frame, (min/2, LOWER_SAT, LOWER_VAL), (max/2, UPPER_SAT, UPPER_VAL))
    else:
        min = COLOUR_RANGES["RED_LOW"]["MIN"]
        max = COLOUR_RANGES["RED_LOW"]["MAX"]
        mask1 = cv.inRange(frame, (min/2, LOWER_SAT, LOWER_VAL), (max/2, UPPER_SAT, UPPER_VAL))

        min = COLOUR_RANGES["RED_HIGH"]["MIN"]
        max = COLOUR_RANGES["RED_HIGH"]["MAX"]
        mask2 = cv.inRange(frame, (min/2, LOWER_SAT, LOWER_VAL), (max/2, UPPER_SAT, UPPER_VAL))

        mask = mask1 | mask2
    return mask


def Make_Edge_Frame(frame):
    mask = Edge_Mask(frame)
    frame = CV_Mask_To_Frame(frame.shape, mask, COLOURS["WHITE"])
    return frame


def Edge_Mask(frame):
    LOWER_THRESHOLD = 100
    UPPER_THRESHOLD = 200
    APETURE = 5

    return cv.Canny(frame, LOWER_THRESHOLD, UPPER_THRESHOLD, APETURE)

def Make_Lines_Frame(frame):

    lines = Make_Lines(frame)
    mask = Make_Line_Mask(frame, lines)
    mask = mask.astype(np.uint8)
    frame = CV_Mask_To_Frame(frame.shape, mask, COLOURS["PURPLE"])
    return frame

def Make_Lines(frame):
    GREY_Frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    lines = Make_Line_Guesses(GREY_Frame)
    return lines


def Make_Line_Mask(frame, lines):
    THICKNESS = 2

    (height, width, depth) = frame.shape
    mask = np.zeros((height, width), dtype=np.uint8)

    if lines is not None:
        for line in lines:
            (x1, y1, x2, y2) = line
            cv.line(mask, (x1, y1,), (x2, y2), 255, thickness=THICKNESS)
    return mask

def Make_Line_Guesses(frame):
    PX_RESOLUTION = 1
    ANG_RESOLUTION = 2*np.pi/360
    MIN_HITS_PER_LINE = 30
    PX_MIN_LEN = 80
    PX_MAX_GAP = 20
    
    lines = []
    lines = cv.HoughLinesP(frame, PX_RESOLUTION, ANG_RESOLUTION, MIN_HITS_PER_LINE, minLineLength=PX_MIN_LEN, maxLineGap=PX_MAX_GAP)
    return lines

8


def Make_Rectangles_Frame(BGR_edgeFrame, colourMasks, rectangles):
    GREY_edgeFrame = cv.cvtColor(BGR_edgeFrame, cv.COLOR_BGR2GRAY)
    if rectangles is not None and len(rectangles) > 0:
        rectMasks = []
        for rectangle in rectangles:
            rectMasks.append(Make_Poly_Mask(GREY_edgeFrame,[rectangle]))
        colouredRectangles = Make_Colour_Rectangles(rectMasks, colourMasks, BGR_edgeFrame)
        if colouredRectangles is not None and len(colouredRectangles > 0):
            frame = colouredRectangles
        else:
            return np.zeros_like(BGR_edgeFrame, dtype=np.uint8)
        return frame
    return np.zeros_like(BGR_edgeFrame, dtype=np.uint8)

def Make_All_Rectangles_Frame(BGR_edgeFrame, rectangles):
    if rectangles is not None and len(rectangles) > 0:
        frame = np.zeros_like(BGR_edgeFrame, dtype=np.uint8)
        for rectangle in rectangles:
            cv.polylines(frame, rectangle, True, COLOURS["WHITE"], 2)
        return frame
    return np.zeros_like(BGR_edgeFrame, dtype=np.uint8)

def Make_Rectangles(edgeFrame):
    MIN_AREA = 500
    ALLOWED_DEVIANCE = 0.05

    edgeFrame = cv.GaussianBlur(edgeFrame, (3,3), 5, 5, sigmaY=5, borderType=cv.BORDER_CONSTANT)
    contours = cv.findContours(edgeFrame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]

    rectangles = []
    for contour in contours:
        if cv.contourArea(contour) > MIN_AREA:
            #for each contour, make it a polygon
            max_move = ALLOWED_DEVIANCE * cv.arcLength(contour, True)
            polygon = cv.approxPolyDP(contour, max_move, True)
            #check the polygon has 4 points and is convex
            if len(polygon) == 4 and cv.isContourConvex(polygon):
                rectangles.append(polygon)
    return rectangles

def Make_Colour_Rectangles(rectMasks, colourMasks, frame):
    colouredRects = []
    if rectMasks is not None:
        for mask in rectMasks:
            rectColour = Find_Rectangle_Colour(mask, colourMasks)
            if rectColour is not None:
                colouredRects.append(CV_Mask_To_Frame(frame.shape, mask, COLOURS[rectColour]))
        return CV_Combine(colouredRects)



def Find_Rectangle_Colour(rectMask, colourMasks):
    MIN_COLOURING = 0.6
    colourCounts = {}
    totalPixels = cv.countNonZero(rectMask)
    for colour in colourMasks.keys():
        colourCounts[colour] = Count_Coloured_Pixels(colourMasks[colour], rectMask)
    strongestColour = max(colourCounts.items(), key=lambda colour: colour[1])[0]
    if colourCounts[strongestColour] > totalPixels * MIN_COLOURING:
        return strongestColour
    else:
        return None
                

def Count_Coloured_Pixels(colourMask, mask):
    hitMask = cv.bitwise_and(colourMask, mask)
    colouredPixels = cv.countNonZero(hitMask)
    return colouredPixels
    
            

def Make_Poly_Mask(frame, polygons):
    mask = np.zeros(frame.shape, dtype=np.uint8)
    for polygon in polygons:
        cv.fillConvexPoly(mask, polygon, 255)
    return mask

def Pythag(a,b):
    return np.sqrt(a**2 + b**2)

def Det(a, b, c, d):
    return (a*d - b*c)

def CV_Mask_To_Frame(shape, mask, BGR_colour):
    colourFill = np.full(shape, BGR_colour, dtype= np.uint8)
    frame = cv.bitwise_and(colourFill,colourFill, mask=mask)
    return frame

def CV_Combine(images):
    if not images or len(images) == 0:
        return None
    image1 = images[0]
    for image2 in images[1:]:
        image1 = cv.bitwise_or(image1, image2)
    return image1


def CV_Convert(imagelist, conversion):
    newList = []
    for image in imagelist:
        newList.append(cv.cvtColor(image, conversion))
    return newList


rawVideo = cv.VideoCapture(0)
closing = False
success = False
BGR_newFrame = [()]

#Get first frame
while success == False:
    success, BGR_newFrame = rawVideo.read()


parkColour = None
#initial loop
while(True and not closing and (parkColour is None or parkColour == "WHITE")):
    #save the last frame as a buffer to average against
    BGR_lastFrame = BGR_newFrame
    #Get current frame
    success, BGR_newFrame = rawVideo.read()

    if (not success):
        continue
    else:
        #average the last two frames to reduce noise
        BGR_averagedFrame = cv.addWeighted(BGR_lastFrame, 0.1, BGR_newFrame, 0.9, 0)
        #crop the left half out so that we're only looking at the near half of the image
        (height, width, channels) = BGR_averagedFrame.shape
        BGR_rawFrame = BGR_averagedFrame[:, width // 2:]

        #seperate each hue into it's own mask
        colourMasks = Make_Colour_Masks(BGR_rawFrame)

        #find edges of the frame
        BGR_edges = Make_Edge_Frame(BGR_rawFrame)


        #make a maske for each rectangle that exists
        GREY_edgeFrame = cv.cvtColor(BGR_edges, cv.COLOR_BGR2GRAY)
        rectangles = Make_Rectangles(GREY_edgeFrame)


        BGR_rectangles = Make_All_Rectangles_Frame(BGR_edges, rectangles)

        cv.imshow("rectangles", BGR_rectangles)
        cv.imshow("edges", BGR_edges)


        #assume we only get one rectangle, and it's the right one ¯\_(•_•)_/¯
        if rectangles is not None and len(rectangles) > 0:
            rectMask = Make_Poly_Mask(GREY_edgeFrame, [rectangles[0]])
            parkColour = Find_Rectangle_Colour(rectMask, colourMasks)

        k = cv.waitKey(1)
        if k ==  ord("q"):
            closing = True

#Main loop
while(True and not closing):
    #save the last frame as a buffer to average against
    BGR_lastFrame = BGR_newFrame
    #Get current frame
    success, BGR_newFrame = rawVideo.read()
    if (not success):
        continue

    #Process the image
    else:
        #average the last two frames to reduce noise
        BGR_rawFrame = cv.addWeighted(BGR_lastFrame, 0.1, BGR_newFrame, 0.9, 0)
        # cv.imshow("Raw image", BGR_rawFrame)

        #Makes a frame with the colours eggagerated and homogenised
        colourMasks = Make_Colour_Masks(BGR_rawFrame)
        BGR_colouredFrame = Make_Colour_Frame(BGR_rawFrame, colourMasks)
        # cv.imshow("Coloured image", BGR_colouredFrame)

        #Makes a frame with the edges in white
        BGR_edges = Make_Edge_Frame(BGR_rawFrame)
        # cv.imshow("Edges", BGR_edges)

        # # #Makes a frame with lines showing:
        # BGR_lines = Make_Lines_Frame(BGR_edges)
        # cv.imshow("Lines", BGR_lines)

        #make a maske for each rectangle that exists
        GREY_edgeFrame = cv.cvtColor(BGR_edges, cv.COLOR_BGR2GRAY)
        rectangles = Make_Rectangles(GREY_edgeFrame)

        BGR_rectangles = Make_All_Rectangles_Frame(BGR_edges, rectangles)
        cv.imshow("All Rectangles", BGR_rectangles)

        BGR_colouredRectangles = Make_Rectangles_Frame(BGR_edges, colourMasks, rectangles)
        cv.imshow("Coloured Rectangles", BGR_colouredRectangles)

        BGR_correctlyColouredRectangles = Make_Rectangles_Frame(BGR_edges, {parkColour: colourMasks[parkColour]}, rectangles)
        cv.imshow("Correctly Coloured Rectangles", BGR_correctlyColouredRectangles)
        if np.count_nonzero(BGR_correctlyColouredRectangles) > 0:
            print(1)
        else:
            print(0)

        #Combines whatever frames I want to see as a final output.
        framesToCombine = [BGR_correctlyColouredRectangles, BGR_colouredFrame, BGR_edges]
        BGR_compositeFrame = CV_Combine(framesToCombine)
        cv.imshow("BGR_compositeFrame", BGR_compositeFrame)

        k = cv.waitKey(1)
        if k ==  ord("q"):
            closing = True

