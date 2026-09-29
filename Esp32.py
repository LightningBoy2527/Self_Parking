import numpy as np
import Car
import requests

IP = "http://tacacar.local"

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
                "turn":f"{self.sending_angle}","speed":f"{self.sending_speed}"
            }
            print(request_headers)
            #Send an http post request with headers containing information
            response = requests.post(IP + "/", headers=request_headers, timeout=3) #the / means the main get, top file directory
            #process the response
            content_type = response.headers.get("Response-Type")
            content = response.content.decode(encoding = "utf-8")
            
            response.close()
            #split types of distance sensors (eg. BACK1, LEFT...)
            type_array = content_type.split(" ")
            #split content by spaces (floats but stored as a string)
            content_array = content.split(" ")
            #make a dictionary of content and types
            distances = dict(zip(type_array, content_array))
            if len(distances) > 0:
                for sensor in distances.keys():
                    self.sensors[sensor].last_sensed = distances[sensor]
            return
        except Exception as e:
            print("Error: ", e)
            return 0
    

