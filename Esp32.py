import numpy as np
import Car
import requests

IP = "http://10.42.0.58"

class Esp32:
    def __init__(self, connectioninfo, parent, sensors):
        self.connectioninfo = connectioninfo
        self.parent = parent
        self.sensors = sensors
        self.x = parent.x
        self.y = parent.y
        self.sending_speed = self.SetMotorSpeed(parent.speed)
        self.sending_angle = self.SetTurningAngle(parent.wheel_dir)

    def Connect():
        pass

    def SetMotorSpeed(self, mmPerSec):
        CONVERSION_FACTOR = 255 / Car.MAX_SPEED
        self.sending_speed = mmPerSec * CONVERSION_FACTOR # takes 0-255

    def SetTurningAngle(self, rads):
        #servo takes angle from the end, and actuated angle doesn't line up with sent angle.
        degs = np.rad2deg(rads)
        self.sending_angle = 66.3 + degs * 1.4
        pass

    def SendRequest(self):
        try:
                
            request_headers = {
                "turn":f"{self.sending_angle}","speed":f"{self.sending_speed}","distance":f"{"front"}"
            }
            print(request_headers)
            #if any input is not 0 then reassign that variable
            response = requests.post(IP + "/", headers=request_headers, timeout=0.5) #the / means the main get, top file directory
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
    

