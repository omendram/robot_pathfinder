# Jacob Salam i6184256
# Omendra Manhar i6131589
# Kira Ruschmeier i6181835
# Jonas Schmidt i6185307


import pygame
import random, sys, time, math, numpy, shapely
from numpy import linalg
import numpy as np
from random import randint, uniform
from shapely.geometry import LineString

# Initialize the game engine
pygame.init()

BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREEN = [0, 255, 0]
PINK = [247,98,106]

# Set the height and width of the screen
SIZE = [1080, 720]

screen = pygame.display.set_mode((1080,720),0,32)
pygame.display.set_caption("Maze")


clock = pygame.time.Clock()
# initial angle and speed
angle = 0
speed = 5

#initial position
x = 90
y = 90
#intitial position which the robot believes its in
believeX = 90
believeY = 90

#motion is the movement in x and y axis 
xBelieveMotion = speed* math.cos(math.radians(angle))
yBelieveMotion = speed* math.sin(math.radians(angle))

xMotion =speed* math.cos(math.radians(angle))
yMotion =speed* math.sin(math.radians(angle))
#bunch of variable
radius = 50
rotx=0
roty=0
xpos=0
ypos=0
rot1=0
rot2=0
trans =0
stuckInWall = 0
directionAngle =0
done = 0
trailOfPositions = []
trailSize = 20


#noise used in the odomotry motion model
noise1 = 0.2 # noise for rot1 and 2
noise2 = 0.1 # trans noise for rot1 and 2
noise3 = 0.4 # trans noise for trans
noise4 = 0.2 # rot noise for trans

#Wallist is a list of walls which the robot must move around
wallList = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[267,10,267,236],[534,236,534,472],[10,472,534,472],[801,10,801,472]]

#Beacons (Jonas Schmidt)
pointsToVisit = [[130, 350],[400,350],[400,100],[650,100],[650,600],[950,600],[950,100]]



'''
Kalman Filter definitions
'''
#Jacob Salam, Omendra Manhar, Kira Ruschmeier
X = np.matrix('90. 90. 1. 1.').T
P = np.matrix(np.eye(4))*1000 # initial uncertainty
R = 0.01**2

def kalman_xy(x, P, measurement, R,
              motion = np.matrix('0. 0. 0. 0.').T,
              Q = np.matrix(np.eye(4))):
    return kalman(x, P, measurement, R, motion, Q,
                  F = np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H = np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))

#Jacob Salam, Omendra Manhar, Kira Ruschmeier
def kalman(x, P, measurement, R, motion, Q, F, H):
    
    # Update x, P
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R
    K = P * H.T * S.I
    x = x + K*y
    I = np.matrix(np.eye(F.shape[0]))
    P = (I - K*H)*P

    # Predict x, P based on motion
    x = F*x + motion
    P = F*P*F.T + Q

    return x, P


# Updates the angle of the pointer (Jacob Salam)
def updateAngle(x,y,nextx,nexty):
    return math.atan2(nexty-y,nextx-x)

# Calculates the direction of the robot pointer (Jacob Salam)
def calcDirection(x,y,angle):
    rotx=x + (radius-1) * math.cos(math.radians(angle))
    roty=y + (radius-1)* math.sin(math.radians(angle))
    return rotx, roty

# Draws the walls
def makeWalls(wall):
    pygame.draw.line(screen, BLACK,(wall[0],wall[1]),(wall[2],wall[3]),2)

# Checks if there is a collision (Jacob Salam)
def CheckCollision(wall,xpos1,ypos1):
    x1, y1 = wall[0],wall[1]
    x2, y2 = wall[2],wall[3]
    x0, y0 = xpos1,ypos1
    nom = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denom = ((y2 - y1)**2 + (x2 - x1) ** 2) ** 0.5
    result = (nom / denom) - 50;
    if result <= 0 and ((x0 >= x1-50 and x0 <= x2+50) and (y0 >= y1-50 and y0 <= y2+50)):
        return True
    else:
        return False

# Updates the position of the robot based on its motion (Jacob Salam)
def updatePosition(x,y,xMotion,yMotion):
    x = x + xMotion
    y = y + yMotion
    xpos = int(x)
    ypos = int(y)
    return x,y

#calculates the motion the robot initialy thinks its going (Only used when Kaalman filter is not in use) (Jacob Salam)
def Motion(angle):
    xBelieveMotion = speed* math.cos(math.radians(angle))
    yBelieveMotion = speed* math.sin(math.radians(angle))

    return xBelieveMotion, yBelieveMotion

#Changes direction randomnly based on the angles it comes in and which wall it hits (Jacob Salam)
def changeDirection(xMotion,yMotion,directionAngle,wall):
    if directionAngle > 180 and wallList[wall][1] == wallList[wall][3]:
        directionAngle2 = randint(20,160)
    if directionAngle <= 180 and wallList[wall][1] == wallList[wall][3]:
        directionAngle2 = randint(200,340)
    if (directionAngle > 270 or (directionAngle >= 0 and directionAngle <= 90)) and wallList[wall][0] == wallList[wall][2]:
        directionAngle2 = randint(110,250)
    if (directionAngle <= 270 and directionAngle > 90) and wallList[wall][0] == wallList[wall][2]:
        directionAngle2 = randint(-70,70)%360

    xMotion =speed* math.cos(math.radians(directionAngle2))
    yMotion =speed* math.sin(math.radians(directionAngle2))
    return xMotion,yMotion, directionAngle2


# Finds the next direction of the robot when it hits a wall (Jacob Salam)
def findNextDirection(xMotion,yMotion,directionAngle,d,wall):

    xMotion,yMotion,directionAngle = changeDirection(xMotion,yMotion,directionAngle,wall)
    for i in range(0,8):
        d = CheckCollision(wallList[i],x+xMotion,y+yMotion)
    return xMotion,yMotion,directionAngle

# After a wall hit, I move it back in the field as it sometimes likes to jump out (Jacob Salam)
def adjustPosition(directionAngle,wall,x,y):
    if directionAngle >= 180 and wallList[wall][1] == wallList[wall][3]:
        y = y+speed+1
    if directionAngle < 180 and wallList[wall][1] == wallList[wall][3]:
        y = y -speed-1
    if (directionAngle > 270 or (directionAngle > 0 and directionAngle < 90)) and wallList[wall][0] == wallList[wall][2]:
        x = x-speed-1
    if (directionAngle <= 270 and directionAngle > 90) and wallList[wall][0] == wallList[wall][2]:
        x= x+speed+1
    return x,y

# If it gets stuck in a wall, which it sometimes does on corners, move it a little in the correct direction (Jacob Salam)
def stuck(directionAngle,wall,x,y):
    if directionAngle >= 180 and wallList[wall][0] == wallList[wall][2]:
        y= y+radius*0.5
    if directionAngle < 180 and wallList[wall][0] == wallList[wall][2]:
        y = y -radius*0.5
    if (directionAngle > 270 or (directionAngle > 0 and directionAngle < 90)) and wallList[wall][1] == wallList[wall][3]:
        x = x-radius*0.5
    if (directionAngle <= 270 and directionAngle > 90) and wallList[wall][1] == wallList[wall][3]:
        x= x+radius*0.5
    return x,y

# Sensor Inputs, returns the endpoints of the sensor lines (Omendra Manhar)
def sensors(centerX, centerY):
    sensorX = 0
    sensorY = 0

    for i in range(24):
        #the sensor can be visualized when the commented code is uncommented
        #point1 = centerX, centerY
        distance = getSensorDistance(i,centerX,centerY)
        #point2 = centerX + distance[0] * math.cos(i * 15*math.pi/180+ math.radians(angle)), centerY + distance[0] * math.sin(i * 15 * math.pi/180+math.radians(angle))
        #pygame.draw.line(screen,  BLACK, point1, point2, 1)
        sensorEpX = centerX + distance[0] * math.cos(i * 15*math.pi/180+math.radians(angle))
        sensorEpY = centerY + distance[0] * math.sin(i * 15 * math.pi/180+math.radians(angle))

        sensorX = sensorX + centerXEstimatesFromSensor(i, sensorEpX, distance[1], math.radians(angle))
        sensorY = sensorY + centerYEstimatesFromSensor(i, sensorEpY, distance[1], math.radians(angle))

    return sensorX/24, sensorY/24


# Return estimates of X and Y based on the sensor readings, including the noise
# Use this along with odometry estimates to feed into KF and , determine the estimated position of the robot (Omendra Manhar)
def centerXEstimatesFromSensor(i, sensorEpX, distance, angle):
    return sensorEpX - distance*math.cos(i * 15*math.pi/180 + angle)

def centerYEstimatesFromSensor(i, sensorEpY, distance, angle):
    return sensorEpY - distance*math.sin(i * 15*math.pi/180 + angle)

def getSensorDistance(sensor,x,y): #Jacob Salam, Omendra Manhar
    closestWall = 0
    closestWallDistance =1000
    distance = 0


    for i in range(0,8):

        x1 = wallList[i][0]
        y1 = wallList[i][1]
        x2 = wallList[i][2]
        y2 = wallList[i][3]


        line1 = LineString([(x1,y1), (x2,y2)])
        line2 = LineString([(x,y), (x + 1000 * math.cos(sensor * 15*math.pi/180+math.radians(angle)), y + 1000 * math.sin(sensor * 15 * math.pi/180+math.radians(angle)))])
        point = line1.intersection(line2)
        point = numpy.array(point)

        if len(point) > 0:
            distance = ((point[0] - x)**2 + (point[1] - y)**2)**(1/2)
            if distance < closestWallDistance:
                closestWallDistance = distance
                closestWall = i

    # Introduce Noise to the sensors
    return [randint(90, 110)*closestWallDistance/100, closestWallDistance]

# odomotry motion model (Jacob Salam, Omendra Manhar)
def SampleNormalDistribution(b) :
    value = 0
    for i in range(0,12):
        value = value + uniform(-b,b)
    value = value/2
    return value

def sampleMotionModel(rot1,rot2,trans,x,y,angle):
    rot1 = math.radians(rot1) + SampleNormalDistribution(noise1*abs(math.radians(rot1)) + noise2*trans)
    t = SampleNormalDistribution(noise3*trans + noise4*(abs(math.radians(rot1)))+abs(math.radians(rot2)))
    trans = trans + t
    rot2 = math.radians(rot2)+ SampleNormalDistribution(noise1*abs(math.radians(rot2)) + noise2*trans)
    x = x + trans*math.cos(math.radians((angle + math.degrees(rot1))%360))
    y = y + trans*math.sin(math.radians((angle + math.degrees(rot1))%360))
    angle = (angle + math.degrees(rot1) + math.degrees(rot2))%360
    return x,y,angle



# Loop until the user clicks the close button.

# The while loop is used to simulate time (Jonas Schmidt)

done = False
while done < 7:
    # Set the screen background
    screen.fill(WHITE)

    #wallList Makes walls
    for wall in wallList:
        makeWalls(wall)
    #check for collision with any of the walls, if so find new direction (Jacob Salam)
    for i in range(0,8):
        d = CheckCollision(wallList[i],x,y)
        if d == True:
            if (stuckInWall > 0 and i >3):
                x,y = stuck(directionAngle,i,x,y)
                xMotion,yMotion,directionAngle = changeDirection(xMotion,yMotion,directionAngle,i)
                updatePosition(x,y,xMotion,yMotion)
                break

            stuckInWall = 3
            x,y = adjustPosition(directionAngle,i,x,y)
            xMotion, yMotion,directionAngle = findNextDirection(xMotion,yMotion,directionAngle,d,i)
            break



    # This part controls the motion of the robot and also calls the Kalman filter routine in order to predict and correct the robots belief of its position (Jacob Salam, Omendra Manhar)
    if (believeY < pointsToVisit[done][1]-speed or believeY > pointsToVisit[done][1]+speed) or (believeX < pointsToVisit[done][0]-speed or believeX >pointsToVisit[done][0]+speed):
        rot1 = updateAngle(believeX, believeY, pointsToVisit[done][0],pointsToVisit[done][1])
        rot2 = 0
        rot1 = (math.degrees(rot1) - angle)
        xBelieveMotion, yBelieveMotion = Motion(rot1+angle)
        believeX, believeY = updatePosition(believeX,believeY,xBelieveMotion,yBelieveMotion)
        # Calculates the new x,y positions and angle using the odomotry motion model
        x,y,angle = sampleMotionModel(rot1,rot2,speed,x,y,angle)
        rotx,roty = calcDirection(x,y,angle)
        xpos = int(x)
        ypos = int(y)

        # Get the x and y measurement values from the robots sensors
        measurementX, measurementY = sensors(xpos,ypos)

        #Feed the Kalman filter with measurements and the previous coordinates
        X, P = kalman_xy(X, P, (measurementX, measurementY) , R)

        # New location the robot believes its in based on the Kalman filter
        believeX = X[0, 0]
        believeY = X[1, 0]

        xpos2 = int(believeX)
        ypos2 = int(believeY)
        trailOfPositions.append([xpos2,ypos2])
        if(len(trailOfPositions)>trailSize):
            del trailOfPositions[0]
        rotx2, roty2 = calcDirection(believeX,believeY, angle+rot1)

    else:
        done = done + 1

    # Draw the previous robot belief positions (Jonas Schmidt, Kira Ruschmeier)
    for pos in trailOfPositions:
        pygame.draw.circle(screen, PINK, (pos[0],pos[1]), radius,0)
        pygame.draw.circle(screen, BLACK , (pos[0],pos[1]), radius,2)

    # Draw the new believed position of the robot
    pygame.draw.circle(screen, GREEN , (xpos2,ypos2), radius,0)
    pygame.draw.circle(screen, BLACK , (xpos2,ypos2), radius,2)

    # Draw the true position of the robot
    pygame.draw.circle(screen, WHITE , (xpos,ypos), radius,0)
    pygame.draw.circle(screen, BLACK , (xpos,ypos), radius,2)
    pygame.draw.line(screen,BLACK, (xpos,ypos),(rotx, roty),2)



    # if by chance out of bounds, reintroduce the robot in the initial position
    if (x > 1080 or x< 0 or y >720 or y < 0):
        x = 74
        y= 100
        
    if stuckInWall > 0:
        stuckInWall = stuckInWall -1
        
    for event in pygame.event.get():   # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = 8
               # Flag that we are done so we exit this loop
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    clock.tick(60)

# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit()
