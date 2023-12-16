# CS 330 Programming Assignment 1 Practice
# Jay Sebastian, Adjunct Lecturerl
# jms0147@uah.edu
# Computer Science Department
# University of Alabama in Huntsville
#Edited by Jeriel Leyble for Project 2

# Implementation of Dynamic Movement Update (NE1), Dynamic Seek, Flee, and Arrive

import numpy as np # In Python, this library is great for vector math. (numpy.org)
import math # This library allows me to access square root, pi, etc.

ASSIGNMENT = 2

# Initialize Steering Behavior Constants - Note that I only took what was needed from the R Code
CONTINUE = 1
SEEK = 6
FLEE = 7
ARRIVE = 8
FOLLOWPATH = 11

# Initialize general movement parameters and working variables
time = 0  # Current simulation time
deltaTime = 0.5 # Set time step duration
stopTime = 50 # time of last step

# Class definition for object to hold steering outputs (from Millington)
class SteeringOutput(object):
    def __init__(self):
        self.linear = np.array([0.0, 0.0]) # A 2D array representing the X, Z components of linear acceleration
        self.angular = 0.0 # Not required for this assignment, but here for completeness
# Class definition for a Character object
class Character(object):
    def __init__(self):
        self.ID = 0
        self.steer = 2
        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.linear = np.array([0.0, 0.0])
        self.orientation = 0.0
        self.rotation = 0.0
        self.angular = 0.0
        self.maxVelocity = 0.0
        self.maxLinear = 0.0
        self.maxRotation = 0.0
        self.maxAngular = 0.0
        self.target = self
        self.arriveRadius = 0.0
        self.arriveSlow = 0.0
        self.arriveTime = 0.0
        self.pathToFollow = 0.0
        self.pathOffset = 0.0
        self.colCollided = False

class Path(object):
    def __init__(self):
        self.ID = 0
        self.x = np.array([]) #array of x coords
        self.y = np.array([]) #array of y cords
        self.params = np.array([]) #array of path param at each vertex
        self.distance = np.array([]) #array of path distance at each vertex
        self.segments = 0 #number of segments in the path 

    def pathAssemble(self, ID, X, Y):
        pathSegments = len(X) - 1
        pathDistance = np.repeat(0.0, pathSegments + 1)
        
        for i in range(1, pathSegments + 1):
            pointA = np.array([X[i - 1], Y[i - 1]])
            pointB = np.array([X[i], Y[i]])
            pathDistance[i] = pathDistance[i - 1] + distancePoints(pointA, pointB)

        pathParam = np.repeat(0.0, pathSegments + 1)
        maxDistance = pathDistance[-1]
        
        for i in range(1, pathSegments + 1):
            pathParam[i] = pathDistance[i] / maxDistance
        
        self.ID = ID
        self.x = X #is this needed? 
        self.y = Y
        self.params = pathParam
        self.distance = pathDistance
        self.segments = pathSegments


    def GetParam(self, position):
        closestDistance = float("inf")
        closestPoint = np.array([0.0, 0.0])
        closestSegment = 0

        for i in range(self.segments):
            A = np.array([self.x[i], self.y[i]])
            B = np.array([self.x[i + 1], self.y[i + 1]])
            checkPoint = closestPointSegment(position, A, B)
            checkDistance = distancePoints(position, checkPoint)

            if checkDistance < closestDistance:
                closestPoint = checkPoint
                closestDistance = checkDistance
                closestSegment = i

        A = np.array([self.x[closestSegment], self.y[closestSegment]])
        A_param = self.params[closestSegment]
        B = np.array([self.x[closestSegment + 1], self.y[closestSegment + 1]])
        B_param = self.params[closestSegment + 1]
        C = closestPoint
        T = length(C - A) / length(B - A)
        C_param = A_param + T * (B_param - A_param)

        return C_param

    def getPosition(self, param):
        i = np.max(np.where(param > self.params))
        A = np.array([self.x[i], self.y[i]])
        B = np.array([self.x[i + 1], self.y[i + 1]])
        T = (param - self.params[i]) / (self.params[i + 1] - self.params[i])
        P = A + T * (B - A)
        return P

# a helper function to calculate the length of a vector
def length(vector):
    length = math.sqrt(vector[0]*vector[0] + vector[1]*vector[1])
    return length

# a helper function to normalize a vector (i.e., keep its direction but make its length = 1)
def normalize(vector):
    vLength = length(vector)
    if vLength == 0:
         return vector
    result = np.array([vector[0]/vLength,vector[1]/vLength])
    return result

#vector dot product
def vectorDot(A, B):
    return sum(A * B)
#return Euclidean Distance between two points
def distancePoints(A, B):
    return np.sqrt(((B[0] - A[0]) ** 2 + (B[1] - A[1]) ** 2))
#caculate point on line segment closest to the query point in 2D
def closestPointSegment(Q, A, B):
    T = vectorDot(Q - A, B - A) / vectorDot(B - A, B - A)
    if T <= 0:
        return A
    elif T >= 1:
        return B
    else:
        return A + T * (B - A)

# update function receives a SteeringOutput, the character's max Speed, and the time duration interval (NE1 Update)
def update(character, steering, time): 
    # update the position and orientation
    character.position = np.add(character.position, character.velocity * time)
    character.orientation += character.rotation * time

    # update the velocity and rotation
    character.velocity = np.add(character.velocity, steering.linear * time)
    character.rotation += steering.angular * time

    # check for speed above max and clip
    if length(character.velocity) > character.maxVelocity:
        character.velocity = normalize(character.velocity)
        character.velocity *= character.maxVelocity

    character.linear = steering.linear
    character.angular = steering.angular
    return character

def getSteeringContinue (character):
    result = SteeringOutput()
    result.linear = character.linear
    result.angular = character.angular
    return result

def getSteeringSeek(character, target):

    
    
    result = SteeringOutput() # Creates output structure
    result.linear = target - character.position # Direction to movement target
    # Accelerate at max rate
    result.linear = normalize(result.linear) # Normalizes the vector for linear acceleration
    result.linear *= character.maxLinear
    # Output steering
    result.angular = 0.0
    return result

def getSteeringFlee(character, target):
    result = SteeringOutput()
    result.linear = character.position - target.position
    # Accelerate at max rate
    result.linear = normalize(result.linear) # Normalizes the vector for linear acceleration
    result.linear *= character.maxLinear
    # Output steering
    result.angular = 0.0
    return result

def getSteeringArrive(character, target):
    # Create output structure
    result = SteeringOutput()
    # Get direction and distance to movement target
    direction = target.position - character.position
    distance = length(direction)
    # Test for arrival
    if distance < character.arriveRadius:
        character.velocity = np.array([0, 0])
        return result
    # Outside slowing-down (outer) raduis, move at max speed
    elif distance > character.arriveSlow:
        targetSpeed = character.maxVelocity
    # Between radii, scale speed to slow down
    else:
        targetSpeed = character.maxVelocity * distance / character.arriveSlow
    
    # Target velocity combines speed and direction
    targetVelocity = direction
    targetVelocity = normalize(targetVelocity)
    targetVelocity *= targetSpeed

    # Accelerate to target velocity
    result.linear = targetVelocity - character.velocity
    result.linear /= character.arriveTime

    # Test for too fast acceleration
    if length(result.linear) > character.maxLinear:
        result.linear = normalize(result.linear)
        result.linear *= character.maxLinear
    
    # Output steering
    result.angular = 0
    return result

def getSteeringFollowPath(character, path):
    #Calculate target to delegate to Seek
    #Find current position on path
    currentParam = path.GetParam(character.position)

    #Offset it
    targetParam = min(1, currentParam + character.pathOffset)

    #Get the target position
    targetPosition = path.getPosition(targetParam)
    #tempTarget = Character()
    #tempTarget.position = targetPosition

    #Delegate to seek
    #print(f"DEBUG: tempTarget = {type(tempTarget)}, tempTarget.position = {tempTarget.position}")   #debug stuff
    #return getSteeringSeek(character, tempTarget)
    return getSteeringSeek(character, targetPosition)

# for Programming Assignment 1, set up the four characters and their initial conditions
if ASSIGNMENT == 1:
    character01 = Character()
    character01.ID = 2601
    character01.steer = CONTINUE

    character02 = Character()
    character02.ID = 2602
    character02.steer = FLEE
    character02.position = np.array([-30, -50])
    character02.velocity = np.array([2, 7])
    character02.orientation = (math.pi / 4)
    character02.maxVelocity = 8
    character02.maxLinear = 1.5
    character02.target = character01

    character03 = Character()
    character03.ID = 2603
    character03.steer = SEEK
    character03.position = np.array([-50, 40])
    character03.velocity = np.array([0, 8])
    character03.orientation = (3 * math.pi / 2)
    character03.maxVelocity = 8
    character03.maxLinear = 2
    character03.target = character01

    character04 = Character()
    character04.ID = 2604
    character04.steer = ARRIVE
    character04.position = np.array([50, 75])
    character04.velocity = np.array([-9, 4])
    character04.orientation = math.pi
    character04.maxVelocity = 10
    character04.maxLinear = 2
    character04.target = character01
    character04.arriveRadius = 4
    character04.arriveSlow = 32 
    character04.arriveTime = 1

    characters = [character01, character02, character03, character04] # list of all characters to iterate through

elif ASSIGNMENT == 2:
    deltaTime = 0.5
    stopTime = 125

    #New character for PA#2
    character05 = Character()
    character05.ID = 2701
    character05.steer = FOLLOWPATH
    character05.position = np.array([20,95])
    character05.velocity = np.array([0,0])
    character05.orientation = 0
    character05.maxVelocity = 4
    character05.maxLinear = 2
    character05.pathToFollow = 1
    character05.pathOffset = 0.04

    characters = [character05]

# output initial conditions for each character
f = open('output.txt', 'w') # create file if one does not exist
for character in characters: # print time 0 initial conditions to console
    print(time, character.ID, character.position[0], character.position[1], character.velocity[0],
    character.velocity[1], character.linear[0], character.linear[1], character.orientation, 
    character.steer, character.colCollided, sep = ", ", end = "\n" ,file=f)
f.close()

# main code loop
while (time < stopTime):
    time = time + deltaTime
    # For each timestep, for each character, update the character as follows:
    # 1. Call the character's steering (movement) behavior to get linear and angular accelerations
    # 2. Update the character's position, orientation, velocity, and rotation
    #    using the linear and angular accelerations returned by the steering behavior
    # 3. If collision detection active for the scenario, determine if any characters have collided (not needed for PA#1)
    # 4. Write the character's updated data to the output trajectory file

    # loop through all characters and execute relevant steering behavior
    for character in characters:
        if character.steer == CONTINUE:
            steering = getSteeringContinue(character)
        elif character.steer == SEEK:
            steering = getSteeringSeek(character, character.target)
        elif character.steer == FLEE:
            steering = getSteeringFlee(character, character.target)
        elif character.steer == ARRIVE:   
            steering = getSteeringArrive(character, character.target)
        elif character.steer == FOLLOWPATH:
            if character.pathToFollow == 1:
                path = Path()
                X = (0.0, -20.0, 20.0, -40.0, 40.0, -60.0, 60.0, 0.0)
                Y = (90.0, 65.0, 40.0, 15.0, -10.0, -35.0, -60.0, -85.0)
                path.pathAssemble(1, X, Y)
            steering = getSteeringFollowPath(character, path)
        
        # calculate updates
        character = update(character, steering, deltaTime)

    # cycle through characters and print updated conditions
    f = open('output.txt', 'a') # append to output file
    for character in characters:
        print(time, character.ID, character.position[0], character.position[1], character.velocity[0],
        character.velocity[1], character.linear[0], character.linear[1], character.orientation, 
        character.steer, character.colCollided, sep = ", ", end = "\n" ,file=f)
    f.close()