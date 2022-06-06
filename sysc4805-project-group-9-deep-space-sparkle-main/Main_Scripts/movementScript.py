from cmath import pi
import sim
import logging

class movementScript():
    def __init__(self, clientID) -> None:
        
        self.clientID = clientID
        returnList = [1, 1, 1, 1, 1]
        # List of joint names
        jointNameList = ['Motor_LeftFront', 'Motor_RightFront', 'Motor_RightBack', 'Motor_LeftBack']
        self.jointList = [0, 0, 0, 0]

        # Gets the object handles from coppelia sim. Currently looking for 4 joints listed above.
        for i, joint in enumerate(jointNameList):
            returnList[i], self.jointList[i] = sim.simxGetObjectHandle(clientID, joint, sim.simx_opmode_blocking)
            
        # Gets the object handle for the robot body from the simulation.
        returnList[4], self.body = sim.simxGetObjectHandle(clientID, "MainBody", sim.simx_opmode_blocking)
    
        # If the handle could not be found, return an error.
        for returnCode in returnList:
            if returnCode != 0:
                logging.error(f"Could not get an object handle for a motor. Return code = {returnCode}")


    #A single call to drive will keep the wheels turning until a call set the joint velocity to anything else
    def drive(self, velocity = 1) -> bool:
        """Function which will start the plow moving forwards at a velocity. This velocity should be calculated based on wheel diameter (which can be negative to indicate reversing)"""
        # Blocking call for setting velocity on joints
        returnList = [100, 100, 100, 100]
        for i, joint in enumerate(self.jointList):
            returnList[i] = sim.simxSetJointTargetVelocity(self.clientID, joint, velocity, sim.simx_opmode_blocking)

        # If the return code is not good, then log an error and return False
        for i, returnCode in enumerate(returnList):
            if returnCode != 0:
                logging.error(f"Could not set a velocity on joint {i}, with {returnCode=}")
                return False
        return True


    
    def rotate(self, angle: int, smooth = False, fast = False) ->bool:
        """Rotate the robot by the given angle. This angle will be measured from 0 to 360 degrees starting from the left. 
                See the diagram below for a visual representation. This function will take non trivial time to complete as the rotation will take time.
                Will also turn smoothly if smooth = True. Will turn faster if fast = True.
                    0

            270   RobotBody     90

                  180
                """
        returnList = [100, 100, 100, 100]

        # Gets the current position of the robot
        returnValue, returnAngle = sim.simxGetObjectOrientation(self.clientID, self.body, -1, sim.simx_opmode_blocking)
        if returnValue != 0:
                logging.error(f"Could not get the body's orientation. {returnValue=}")

        currentAngle = self._radToAngle(returnAngle[2])
        # calculate the target angle
        targetAngle = currentAngle + angle
        if targetAngle < 0:
            targetAngle = targetAngle + 360

        if targetAngle > 360:
            targetAngle = targetAngle - 360
        # Calculate the direction and amount needed to rotate
        shortestAngle = min([targetAngle - currentAngle, targetAngle - currentAngle + 360, targetAngle - currentAngle - 360], key=abs)

        # Rotate in the required direction
        if shortestAngle > 0:
            leftDirection = 1
        else:
            leftDirection = -1
        # If fast = True, increase the speed we want to rotate.
        if fast:
            leftDirection = 6*leftDirection

        # Make the right value the negative of the left
        rightDirection = -1 * leftDirection

        # Initially, set all joints to idle, then set the left wheels to one direction, and the right wheels to the opposite.
        self.drive(0)
        # If smooth ==True, dont rotate on the spot, but smoothly turn.
        if smooth:
            if leftDirection > 0:
                leftDirection = 15
                rightDirection = -0.015
            else:
                rightDirection = 15
                leftDirection = -0.015

        # Start the turn
        returnList[0] = sim.simxSetJointTargetVelocity(self.clientID, self.jointList[0], leftDirection, sim.simx_opmode_blocking)
        returnList[1] = sim.simxSetJointTargetVelocity(self.clientID, self.jointList[1], rightDirection, sim.simx_opmode_blocking)

        returnList[3] = sim.simxSetJointTargetVelocity(self.clientID, self.jointList[3], leftDirection, sim.simx_opmode_blocking)
        returnList[2] = sim.simxSetJointTargetVelocity(self.clientID, self.jointList[2], rightDirection, sim.simx_opmode_blocking)

        for i, returnCode in enumerate(returnList):
            if returnCode != 0:
                logging.error(f"Could not set a velocity on joint {i}, with {returnCode=}")
                return False

        # Poll the position of the robot until the robot is within a reasonable margin to the target angle.   
        while abs(currentAngle - targetAngle) > 4:
            logging.debug(f"{abs(currentAngle - targetAngle)=}")
            returnValue, returnAngle = sim.simxGetObjectOrientation(self.clientID, self.body, -1, sim.simx_opmode_blocking)
            currentAngle = self._radToAngle(returnAngle[2])

        # Stop turning.
        self.drive(0)
        return True

    def _radToAngle(self, rad: float)->float:
        """Function to convert radians to a 0-360 degree value."""
        angle = (rad * (180/pi))
        if angle < 0:
            angle = 360 - abs(angle)
        return round(angle)            
        