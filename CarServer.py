import requests 
import numpy as np
   
turn_angle = 0
speed = 0 #speed in mm/s
direction = "front"


def Distance(new_dir):
    global direction
    direction = new_dir

def Turn(new_angle):
    global turn_angle
    turn_angle = 66.3 + new_angle * 1.4
    
def Drive(new_speed):
    global speed
    speed = new_speed 
    
def SendHTTP():
    try:
        #seems to be recieveing nothing? after decoding content is " "
        global turn_angle
        global speed
        global direction
        
        #check for inputs:
        # if (new_angle!= 999):
        #     turn_angle = new_angle
        # if (new_speed != 999):
        #     speed = new_speed
        # if (new_dir != "999"):
        #     direction = new_dir
            
            
        request_headers = {
            "turn":f"{turn_angle}","speed":f"{speed}","distance":f"{direction}"
        }
        print(request_headers)
        #if any input is not 0 then reassign that variable
        response = requests.post(IP + "/", headers=request_headers, timeout=3) #the / means the main get, top file directory
        #process the response
        content_type = response.headers.get("Response-Type")
        content = response.content.decode(encoding = "utf-8")
        
        #print(content_type)
        #print(content)
        response.close()
        
        if (content_type == "distance"):
            return content
        else:
            return "Response not in recognised format"
    except Exception as e:
        print("Error: ", e)
        return 0
    

IP = "http://10.42.0.58"
# count = 60
# g = True
#controller.start_controller()

# while True:
#     # if (g == True):
#     #     count += 5
#     # else:
#     #     count -= 5
#     # Distance("front1")
#     # print(SendHTTP())
#     turn = 0.2
#     speed = 1
#     print(speed)
#     Turn(60+turn*26)
#     Distance("right")
#     Drive(speed*2000 + 500 * np.sign(speed))
#     print(SendHTTP())
#     #time.sleep(0.1)
#     # if (count >= 60+26):
#     #     g = False
#     # elif (count <= 60-26):
#     #     g = True
#     #print(Turn(10))
    
 