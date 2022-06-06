import sim
import logging
class borderDetection():


    #Init for the class
    def __init__(self, clientID) -> None:
        self.clientID = clientID
        returnCodeFront, self.border_vision_sensor_under = sim.simxGetObjectHandle(self.clientID, "border_vision_sensor_under", sim.simx_opmode_blocking)
        # If the handle could not be found, return an error.
        if (returnCodeFront) != 0:
            logging.error(f"Could not get the object handle for the border detection vision sensor. {returnCodeFront=}")

    #Since there is only one border detector (vision sensor), we poll it each loop and returns a boolean
    def checkForBorder(self) -> bool:
        """Method to check the vision sensor for a black border. 
        Returns true for border detected, false otherwise."""
        # Blocking call for vision sensor info
        returnCode, _, data = sim.simxReadVisionSensor(self.clientID, self.border_vision_sensor_under, sim.simx_opmode_blocking)
        # If the return code is not good, then log an error and assume there is a border (Subject to change)
        if returnCode != 0:
            logging.error("Could not get vision sensor reading... Assuming there is a border.")
            return True
        # Data index 11 is the value for black objects. If this data is greater than 0.3, then there is probably a black object in view.
        if data[0][11] < 0.6:
            return True
        return False