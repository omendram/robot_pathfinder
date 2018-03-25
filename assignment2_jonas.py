import pygame
import random, sys, time, math, numpy, shapely
from numpy import dot, sum, tile, linalg, array
from numpy.linalg import inv
from random import randint, uniform
from shapely.geometry import LineString

# Initialize the game engine
pygame.init()
pygame.display.set_caption("Maze")

#init constants (global variables in capital letters will not change their value, once assigned)
BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREEN = [0, 255, 0]
PINK = [255,200,220]
SPEED = 10
RADIUS = 50
WALLLIST = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[267,10,267,236],[534,236,534,472],[10,472,534,472],[801,10,801,472]]
# Set the height and width of the screen
SCREEN = pygame.display.set_mode((1080,720),0,32)
CLOCK = pygame.time.Clock()
# Noise
ROTNOISE = 0.2
SPEEDNOISE = 0.2
SENSORNOISE = 0.1

#init global variables (global variables in lowercase letters may change their value during the programms runtime)
pointsToVisit = [[130, 350],[400,350],[400,100],[650,100],[650,600],[950,600],[950,100]]
stuckInWall = 0
done = False
# actual robot
angle1 = 0
x1 = 90#position x
y1 = 90#position y
# robot position without noise
angle2 = 0
x2 = 90#position x
y2 = 90#position y
# for sensor evaluation:
oldMeasurements = []
#KALMAN variables
kf_P = array([[1000,0,0,0],[0,1000,0,0],[0,0,1000,0],[0,0,0,1000]])#initial state covariance matrix
kf_Q = array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])#sensor offset (non relative error)
kf_A = array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]])
kf_B = array([[1/2,0],[0,1/2],[1,0],[0,1]])
kf_H = array([[1,0,0,0],[0,1,0,0]])
# define functions

### KALMAN FILTER ALGORITHM
### PREDICTION STEP
# INPUT:
# X(mean state estimate of previous step);
# P(state covariance of previous step);
# A(transition n x n matrix);
# Q(process noise covariance matrix);
# B(input effect matrix);
# U(control input)
def predicitionKF(X, P, A, Q, B, U):
    X = dot(A,X) + dot(B,U) # predict mean X
    P = dot(A, dot(P, A.T)) + Q # predict covariance P
    return (X,P)

### CORRECTION STEP
# INPUT:
# X(predicted x matrix);
# P(predicted P matrix);
# Y(measurment vector);
# H(measurement matrix);
# R(measurement covariance)
def correctionKF(X, P, Y, H, R):
    V = dot(H,X) # mean of predictive distribution of Y
    S = R + dot(H, dot(P, H.T)) # covariance of Y
    K = dot(P, dot(H.T, inv(S))) # Kalman gain matrix
    X = X + dot(K, (Y - V)) # Update X
    P = P - dot(K, dot(S, K.T)) # Update P
    G = gaussian(Y, V, S) # predictive probability using gaussian function
    return(X, P, K, V, S, G)

# GAUSSIAN FUNCTION
def gaussian(Z, N, U):
    if N.shape()[1] == 1:
        DX = Z - tile(N, Z.shape()[1])
        E = 0.5 * sum(DX * (dot(inv(U), DX)), axis = 0)
        E = E + 0.5 * N.shape()[0] * log(2*pi) + 0.5 * log(det(U))
        P = exp(-E)
    elif Z.shape()[1] == 1:
        DX = tile(Z, N.shape()[1]) - N
        E = 0.5 * sum(DX * (dot(inv(U), DX)), axis = 0)
        E = E + 0.5 * N.shape()[0] * log(2 * pi) + 0.5 * log(det(U))
        P = exp(-E)
    else:
        E = 0.5 * dot(DX.T, dot(inv(U), (Z - N)))
        E = E + 0.5 * N.shape()[0] * log(2 * pi) + 0.5 * log(det(U))
        P = exp(-E)

    return (P[0], E[0])

#draws a wall
def makeWall(wall):
    pygame.draw.line(SCREEN, BLACK,(wall[0],wall[1]),(wall[2],wall[3]),2)

#Checks if there is a collision
def CheckCollision(wall, robotX, robotY):
    startX, startY = wall[0], wall[1]
    endX, endY = wall[2], wall[3]
    nom = abs((endY - startY) * robotX - (endX - startX) * robotY + endX * startY - endY * startX)
    denom = ((endY - startY)**2 + (endX - startX) ** 2) ** 0.5
    result = (nom / denom) - 50;
    if (
    result <= 0 and
    robotX >= startX-50 and
    robotX <= endX+50 and
    robotY >= startY-50 and
    robotY <= endY+50
    ):
        return True
    else:
        return False

# After a wall hit, I move it back in the field as it sometimes likes to jump out
def adjustPosition(pAngle,wall,pX,pY):
    rot = math.degrees(pAngle)
    x,y = pX,pY
    if rot >= 180 and wall[1] == wall[3]:
        y = pY + SPEED + 1
    if rot < 180 and wall[1] == wall[3]:
        y = pY - SPEED - 1
    if (rot > 270 or (rot > 0 and rot < 90)) and wall[0] == wall[2]:
        x = pX - SPEED - 1
    if (rot <= 270 and rot > 90) and wall[0] == wall[2]:
        x = pX + SPEED + 1
    return x,y

#Changes direction randomnly based on the angles it comes in and which wall it hits
def changeDirection(xMotion,yMotion,directionAngle,wall):
    if directionAngle > 180 and wall[1] == wall[3]:
        directionAngle2 = randint(20,160)
    if directionAngle <= 180 and wall[1] == wall[3]:
        directionAngle2 = randint(200,340)
    if (directionAngle > 270 or (directionAngle >= 0 and directionAngle <= 90)) and wall[0] == wall[2]:
        directionAngle2 = randint(110,250)
    if (directionAngle <= 270 and directionAngle > 90) and wall[0] == wall[2]:
        directionAngle2 = randint(-70,70)%360

    xMotion = SPEED* math.cos(math.radians(directionAngle2))
    yMotion = SPEED* math.sin(math.radians(directionAngle2))
    return xMotion,yMotion, directionAngle2

# Finds the next direction of the robot
def findNextDirection(pX,pY,pXMotion,pYMotion,pAngle,d,wall):
    xMotion,xMotion,dir = pXMotion,pYMotion,pAngle
    # search for a new direction without collision for as long as necessary
    while True:
        xMotion,xMotion,dir = changeDirection(xMotion,xMotion,dir,wall)
        collision = False
        for i in range(0,8):
            if CheckCollision(WALLLIST[i],pX+xMotion,pY+yMotion):
                collision = True
                break
        if not collision:
            break
    return xMotion,xMotion,dir

# Sensor Inputs, returns the endpoints of the sensor lines
def sensors(centerX, centerY, pAngle):
    offset = math.degrees(pAngle)
    sensorEndpoints = []

    for i in range(12):
        point1 = centerX, centerY
        # distance[0] with noise, distance[1] without noise
        distance = getSensorDistance(i,centerX,centerY,offset)
        point2 = centerX + distance[0] * math.cos(i * 30 + offset), centerY + distance[0] * math.sin(i * 30 + offset)
        pygame.draw.line(SCREEN,  BLACK, point1, point2, 1)
        sensorEndpoints.append([point2, distance])

    # centerXYEstimatesFromCensor(sensorEndpoints)
    return sensorEndpoints

def getSensorDistance(sensor,x,y,offset):
    closestWallDistance = 1000
    distance = 0

    for i in range(0,8):
        x1 = WALLLIST[i][0]
        y1 = WALLLIST[i][1]
        x2 = WALLLIST[i][2]
        y2 = WALLLIST[i][3]

        line1 = LineString([(x1,y1), (x2,y2)])
        line2 = LineString([(x,y), (x + 1000 * math.cos(sensor * 30 + offset), y + 1000 * math.sin(sensor * 30 + offset))])
        point = line1.intersection(line2)
        point = array(point)

        if len(point) > 0:
            distance = ((point[0] - x)**2 + (point[1] - y)**2)**(1/2)
            if distance < closestWallDistance:
                closestWallDistance = distance

    # Introduce Noise
    return [uniform(1-SENSORNOISE,1+SENSORNOISE)*closestWallDistance, closestWallDistance]

#updates the angle of the pointer
def updateAngle(x,y,nextx,nexty):
    return math.atan2(nexty-y,nextx-x)

# splits an angle into movements along the x-axis and y-axis
def Motion(angle):
    xM = SPEED * math.cos(angle)
    yM = SPEED * math.sin(angle)
    return xM, yM

#updates the position of the robot based on its motion
def updatePosition(x,y,xMotion,yMotion):
    return x + xMotion, y + yMotion

def calcDirection(x,y,angle):
    rotx=x + (RADIUS-1) * math.cos(angle)
    roty=y + (RADIUS-1) * math.sin(angle)
    return int(rotx), int(roty)

def moveWithNoise(pX,pY,pAngle):
    noisySpeed = SPEED * (1+sampleNormalDistribution(SPEEDNOISE))
    noisyAngle = ((pAngle + 2 * math.pi) * (1+sampleNormalDistribution(ROTNOISE))) % (2*math.pi)
    x = pX + noisySpeed * math.cos(noisyAngle)
    y = pY + noisySpeed * math.sin(noisyAngle)
    return x,y,noisyAngle

def prepareKFPredParams(x,y,xM,yM):
    kf_X = array([x,y,xM,yM])
    kf_U = array([xM,yM])
    return kf_X,kf_U

def sampleNormalDistribution(b) :
    value = 0
    for i in range(0,12):
        value = value + uniform(-b,b)
        value = value/2
    return value

# Loop until the user clicks the close button.
while not done:
    # boolean to see if a step was taken in the current itteration
    stepTaken = False

    for event in pygame.event.get():   # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True   # Flag that we are done so we exit this loop

    # Set the screen background
    SCREEN.fill(WHITE)
    # draw walls
    for wall in WALLLIST:
        makeWall(wall)

    for wall in WALLLIST:# for each wall
        # we have to use the estimated position instead of the real one,
        #because the robot does not know its real position
        collision = CheckCollision(wall,x1,y1)
        if collision:
            # removed some code here (re-add if stuck)
            stuckInWall = 3
            oldX,oldY = x1,y1
            x1,y1 = adjustPosition(angle1,wall,x1,y1)
            xMotion, yMotion, angle2 = findNextDirection(x1,y1,xMotion,yMotion,angle1,collision,wall)
            # if we adjust our real position, we also have to modify the estimated position by the same amount
            x2 -= oldX - x1
            y2 -= oldY - y1
            # Kalman Filter
            x2,y2 = updatePosition(x2,y2,xMotion,yMotion)
            # update real position with noise
            x1,y1,angle1 = moveWithNoise(x1,y1,angle2)
            stepTaken = True
            break

    if not stepTaken:
        #list of points..
        if len(pointsToVisit)>0:
            if (
            x2 < pointsToVisit[0][0] - SPEED or
            x2 > pointsToVisit[0][0] + SPEED or
            y2 < pointsToVisit[0][1] - SPEED or
            y2 > pointsToVisit[0][1] + SPEED
            ):
                # angle from estimated position to goal position
                angle2 = updateAngle(x2,y2,pointsToVisit[0][0],pointsToVisit[0][1])
                xMotion,yMotion = Motion(angle2)
                # Use Kalman filter
                kf_X,kf_U = prepareKFPredParams(x1,y1,xMotion,yMotion)
                kf_X,kf_P = predicitionKF(kf_X,kf_P,kf_A,kf_Q,kf_B,kf_U)
                print("before: "+str(x1)+" "+str(y1))
                print("predicted: "+str(kf_X[0])+" "+str(kf_X[1]))
                # update real position with noise
                x1,y1,angle1 = moveWithNoise(x1,y1,angle2)
                measurements = sensors(x1, y1, angle1)
                pygame.draw.circle(SCREEN, WHITE, (int(x1),int(y1)), RADIUS,0)
                pygame.draw.circle(SCREEN, BLACK, (int(x1),int(y1)), RADIUS,2)
                pygame.draw.line(SCREEN,BLACK, (int(x1),int(y1)),calcDirection(x1,y1,angle1),2)
                # This vector should contain our position according to our sensor information:
                # The only useful evaluation  for our sensors I can imagine is relative distance in comparison to the previous step
                # The sensors cannot give us the absolute position, since they only measure distances to walls
                #kf_Y = array([measuredX,measuredY])
                #kf_X,kf_P,kf_K,kf_V,kf_S,kf_G = correctionKF(kf_X,kf_P,kf_Y,kf_H,kf_R)
                x2 = kf_X[0]
                y2 = kf_X[1]
                print("after: "+str(x1)+" "+str(y1))
                print("----------------------------")
                x2,y2 = updatePosition(x2,y2,xMotion,yMotion)
            else:
                del pointsToVisit[0] # Removes index 0 from the list
        else:
            done = True

    # estimated position
    pygame.draw.circle(SCREEN, PINK, (int(x2),int(y2)), RADIUS, 0)
    pygame.draw.circle(SCREEN, BLACK, (int(x2),int(y2)), RADIUS, 2)
    pygame.draw.line(SCREEN, BLACK, (int(x2),int(y2)), calcDirection(x2,y2,angle2), 2)



    # not shure wether this should be real position or estimated position
    # I do not care, because this should not happen if we steer correctly
    if (x1 > 1080 or x1< 0 or y1 >720 or y1 < 0):
        x1 = 74
        y1 = 100
    if stuckInWall > 0:
        stuckInWall = stuckInWall -1

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    CLOCK.tick(60)
# Be IDLE friendly. If you forget this line, the program will 'hang' on exit.
pygame.quit()
