from typing import List
import sim
import logging
class objectDetection():
    def __init__(self, clientID) -> None:
        self.clientID = clientID
         # Gets the object handles from coppelia sim. Looking for 4 proximity sensors
        returnCodes = [8] * 4
        self.proxSensors = [None] * 4
        returnCodes[0], self.proxSensors[0] = sim.simxGetObjectHandle(self.clientID, "Proximity_sensor_northeast", sim.simx_opmode_blocking)
        returnCodes[1], self.proxSensors[1] = sim.simxGetObjectHandle(self.clientID, "Proximity_sensor_northwest", sim.simx_opmode_blocking)

        returnCodes[2], self.proxSensors[2] = sim.simxGetObjectHandle(self.clientID, "Proximity_sensor_east", sim.simx_opmode_blocking)
        returnCodes[3], self.proxSensors[3] = sim.simxGetObjectHandle(self.clientID, "Proximity_sensor_west", sim.simx_opmode_blocking)
        
        # Iterate through return codes to check for failures. 
        for returnCode in returnCodes:
            # If the handle could not be found, return an error.
            if returnCode != 0:
                logging.error(f"Could not get the object handle for the proximity detection vision sensor. Return code = {returnCode}")

    def checkForObject(self) -> bool:
        """Method to check the front proximity sensors for any object. 
        Returns true if an object is detected, false otherwise."""
        detectionState = [0, 0]
        returnCode = [1, 1]

        # Blocking call for proximity sensor info
        returnCode[0], detectionState[0], _, _, _ = sim.simxReadProximitySensor(self.clientID, self.proxSensors[0], sim.simx_opmode_blocking)
        returnCode[1], detectionState[1], _, _, _ = sim.simxReadProximitySensor(self.clientID, self.proxSensors[1], sim.simx_opmode_blocking)
        
        # Check all return codes for errors
        for code in returnCode:
            if code != 0:
                logging.error("Could not get proximity sensor reading... Assuming there is an object currently in the path.")
                return True
        for detection in detectionState:
            # If the detection state is 1, the sensor is detecting an object
            if detection == 1:
                return True
        return False


        #Object Detection data ie [true, false, true, true]
    def checkForObjectList(self) -> List:
        objectDetectionData = [False] * 2
        """Method to check the side proximity sensors for any object. 
        Returns a list of detections."""
        # Blocking call for proximity sensor info
        detectionStates = [0] * 2
        returnCodes = [8] * 2

        # Checks side proximity sensors for objects
        returnCodes[0], detectionStates[0], _, _, _ = sim.simxReadProximitySensor(self.clientID, self.proxSensors[2], sim.simx_opmode_blocking)
        returnCodes[1], detectionStates[1], _, _, _ = sim.simxReadProximitySensor(self.clientID, self.proxSensors[3], sim.simx_opmode_blocking)

        # Check return codes for errors.
        for index, code in enumerate(returnCodes):
            if (code) != 0:
                logging.error("Could not get proximity sensor reading... Assuming there is an object currently in the path.")
                objectDetectionData[index] = True
        
        # Check for detections. 
        for index, detection in enumerate(detectionStates):
            if detection == 1:
                objectDetectionData[index] = True

        return objectDetectionData