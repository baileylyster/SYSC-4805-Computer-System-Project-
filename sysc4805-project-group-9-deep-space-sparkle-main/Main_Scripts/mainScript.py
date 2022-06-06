# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

from random import randint
from time import sleep
import logging
from objectdetection import objectDetection
from movementScript import movementScript
from borderDetection import borderDetection
from plowcontrol import plowControl
from plowcontrol import plowMotors


#Functions small left turn and small right turn are especially useful to dynamically avoid objects that the proximity sensors detect
def smallTurnLeft():
    plow.extend_plow_side_to_angle(plowMotors.right, 50)
    movement.rotate(352, True)

def smallTurnRight():
    plow.extend_plow_side_to_angle(plowMotors.left, 50)
    movement.rotate(8, True)

def turnLeft():
    plow.extend_plow_side_to_angle(plowMotors.right, 50)
    movement.rotate(270, fast=True)

def turnRight():
    plow.extend_plow_side_to_angle(plowMotors.left, 50)
    movement.rotate(90, fast=True)

#set plow to initial condition at script start, with the folding plows at 130 degrees
def resetPlow():
    plow.extend_plow_side_to_angle(plowMotors.left, 130)
    plow.extend_plow_side_to_angle(plowMotors.right, 130)


logging.info('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 2)  # Connect to CoppeliaSim


#If API connects correctly, begin setup of the Snow Plow
if clientID != -1:
    logging.info('Connected to remote API server')
    done = False
    border = borderDetection(clientID)
    movement = movementScript(clientID)
    object = objectDetection(clientID)
    plow = plowControl(clientID)
    velocity = 10
    lastTurn = [""] * 2
    plow.extend_plow()
    index = 0
    input("Press enter to start the run. (You can wait until the timer starts here)")

    # Main while loop to run all robot code.#
    #
    # On each iteration main loop will:
    #   1) Check for a border
    #       i)   Border is found, then drive across it, dump the snow, reverse back over the border.
    #       ii)  Try and rotate before proceeding, before rotating, check OBJECT sensors to see if the coast is clear
    #       iii) Finally rotate, if an object is detected dont rotate towards it, if no sensor data make a guess to introduce randomness and break out of Loops.
    #
    #   2) Check for an object
    #       i)   An object is detected, get the list of what all the sensors see, Slow down
    #       ii)  Based on the data in the list we will turn left or right when detection an object
    #       iii) We log each turned direction and to prevent feedback loops avoid turning the same way too many times
    #
    # 3) If the path is clear, drive forward Freely (a call to motors will "SET" it to move 'indefinetely' until set to zero)
    #
    ####################


    while(not done):     
        index = (index + 1) % 2

        if(border.checkForBorder()):  # if a border is detected
            # Stop, then reverse for 0.5 seconds
            movement.drive(0)
            movement.drive(-velocity)
            sleep(0.5)
            # Check the left and right proximity sensors to see if there is a correct way to rotate
            listOfProxSensors = object.checkForObjectList()

            # If there is an object left and not right
            if listOfProxSensors[0] and not listOfProxSensors[1]:
                turnRight()
            # If there is an object right and not left
            elif listOfProxSensors[1] and not listOfProxSensors[0]:
                turnLeft()
            else:
                # Randomly choose a rotation direction, but if the same choice has been chosen twice in a row, overwrite and turn the other way
                if lastTurn[0] == 0 and lastTurn[1] == 0:
                    coin = 1
                    logging.warning("Overwriting turn direction")
                elif lastTurn[0] == 1 and lastTurn[1] == 1:
                    coin = 0
                    logging.warning("Overwriting turn direction")
                else:
                    coin = randint(0, 10)
                if(coin % 2 == 0):
                    movement.rotate(130, fast=True)
                    lastTurn[index] = 0
                else:
                    movement.rotate(230, fast=True)
                    lastTurn[index] = 1
                    
        
        if(object.checkForObject()): # If an object is detected on the front sensors
            # Check the side sensors
            listOfProxSensors = object.checkForObjectList()
            # slow down while handing object
            velocity = 2

            # Again, if we need to overwrite the random direction because we turned the same direction twice in a row
            if lastTurn[0] == 0 and lastTurn[1] == 0:
                coin = 1
                logging.warning("Overwriting turn direction")

            elif lastTurn[0] == 1 and lastTurn[1] == 1:
                coin = 0
                logging.warning("Overwriting turn direction")
            else:
                coin = randint(0, 10)
            # If both side sensors are detecting something, randomly turn left or right quickly.
            if (listOfProxSensors[0] and listOfProxSensors[1]):
                if(coin % 2 == 0):
                    turnLeft()
                    lastTurn[index] = 0
                else:
                    turnRight()
                    lastTurn[index] = 1
            # If neither side sensor is detecting anything, this could be a person, so randomly do a quick left or right turn
            elif (not listOfProxSensors[0]) and (not listOfProxSensors[1]):
                if(coin % 2 == 0):
                    turnLeft()
                    lastTurn[index] = 0
                else:
                    turnRight()
                    lastTurn[index] = 1

            # If just the left sensor is picking up a object, slowly turn right
            elif listOfProxSensors[0] and not listOfProxSensors[1]:
                smallTurnRight()

            # If just the right sensor is picking up a object, slowly turn left
            elif listOfProxSensors[1] and not listOfProxSensors[0]:
                smallTurnLeft()
        else:
            # We have handled the object, so reextend the plow.
            resetPlow()

        # Drive normally
        movement.drive(velocity)
        velocity = 10

    sim.simxFinish(clientID)
else:
    logging.error('Failed connecting to remote API server')
logging.info('Program ended')

