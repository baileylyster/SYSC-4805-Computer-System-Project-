import sim
import logging
import math
# import time
from enum import Enum

class plowMotors(Enum):
    left = 1
    right = 2

class plowControl():
    def __init__(self, clientID):
        self.clientID = clientID
        
        # Gets the object handles from coppelia sim
        returnCode1, self.motorLeft = sim.simxGetObjectHandle(self.clientID, "Ljoint", sim.simx_opmode_blocking)
        returnCode2, self.motorRight = sim.simxGetObjectHandle(self.clientID, "Rjoint", sim.simx_opmode_blocking)
        
        # If the handle(s) could not be found, return an error.
        if returnCode1 != 0 or returnCode2 != 0:
            logging.error(f"Could not get the object handle for the plow motors. Return codes ={returnCode1}, {returnCode2}")

    # extends both sides of the plow to a neutral scoop position
    def extend_plow(self):
        angle = 130
        sim.simxSetJointTargetPosition(self.clientID, self.motorLeft, (angle*math.pi)/180, sim.simx_opmode_blocking) 
        sim.simxSetJointTargetPosition(self.clientID, self.motorRight, -(angle*math.pi)/180, sim.simx_opmode_blocking)

    # angle parameter is given in degrees and converted later
    def extend_plow_side_to_angle(self, plowSide: plowMotors, angle_d: float):
        
        if(plowSide == plowMotors.left):
            plowMotor = self.motorLeft

        elif(plowSide == plowMotors.right):
            plowMotor = self.motorRight

        # Negative angle for right motor
        if(plowMotor == self.motorRight):
            angle_d*=-1.0

        try:
            sim.simxSetJointTargetPosition(self.clientID, plowMotor, angle_d*(math.pi/180), sim.simx_opmode_blocking)  
        except:
            logging.error(f"Error rotating plow joints to angle of {angle_d}")    


    #Never formally used in our code
    def retract_plow(self):
        try:
            sim.simxSetJointTargetVelocity(self.clientID, self.motorLeft, -30, sim.simx_opmode_blocking)  
            sim.simxSetJointTargetVelocity(self.clientID, self.motorRight, 30, sim.simx_opmode_blocking)
        except:
            logging.error("Error retracting plow pistons")        