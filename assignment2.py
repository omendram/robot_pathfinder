# Import a library of functions called 'pygame'
import pygame
import random, sys, time, math, numpy, shapely
from numpy import linalg
from random import randint, uniform
from shapely.geometry import LineString
from random import *
 
# Initialize the game engine
pygame.init()
 
BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREEN = [0, 255, 0]
PINK = [255,200,220]
 
# Set the height and width of the screen
SIZE = [1080, 720]
 
screen = pygame.display.set_mode((1080,720),0,32)
pygame.display.set_caption("Maze")
 
 
clock = pygame.time.Clock()

angle = 0
speed = 2


x = 90
y = 90
believeX = 90
believeY = 90
xBelieveMotion = speed* math.cos(math.radians(angle))
yBelieveMotion = speed* math.sin(math.radians(angle))

xMotion =speed* math.cos(math.radians(angle))
yMotion =speed* math.sin(math.radians(angle))
radius = 50
rotx=0
roty=0
xpos=0
ypos=0
rot1=0
rot2=0
trans =0
directionAngle =0
noise1 = 0.4 # noise for rot1 and 2
noise2 = 0.2 # trans noise for rot1 and 2
noise3 = 0.4 # trans noise for trans
noise4 = 0.2 # rot noise for trans

#    or (y0 >= y1-50 and y0 <= y2+50) wallList = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[265,10,265,238],[269,10,269,234],[269,234,536,234],
 #          [265,238,532,238],[532,238,532,470],[536,238,536,474],[532,470,265,470],[536,474,265,474],[799,710,799,236],[803,710,803,236],[799,236,803,236]]
wallList = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[267,10,267,236],[534,236,534,472],[10,472,534,472],[801,10,801,472]]
pointsToVisit = [[130, 350],[400,350],[400,100],[650,100],[650,600],[950,600],[950,100]]
done = 0

'''
KALMAN STUFF
'''
import numpy as np


X = np.matrix('90. 90. 1. 1.').T 
P = np.matrix(np.eye(4))*1000 # initial uncertainty
R = 0.01**2

def kalman_xy(x, P, measurement, R,
              motion = np.matrix('0. 0. 0. 0.').T,
              Q = np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
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

def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
        Parameters:
        x: initial state
        P: initial uncertainty convariance matrix
        measurement: observed position (same shape as H*x)
        R: measurement noise (same shape as H)
        motion: external motion added to state vector x
        Q: motion noise (same shape as P)
        F: next state function: x_prime = F*x
        H: measurement function: position = H*x
        Return: the updated and predicted new values for (x, P)
        See also http://en.wikipedia.org/wiki/Kalman_filter
        This version of kalman can be applied to many different situations by
        appropriately defining F and H 
    '''
    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K*y
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P = (I - K*H)*P

    # PREDICT x, P based on motion
    x = F*x + motion
    P = F*P*F.T + Q

    return x, P

'''
def demo_kalman_xy():
    x = np.matrix('0. 0. 0. 0.').T 
    P = np.matrix(np.eye(4))*1000 # initial uncertainty
    result = []
    R = 0.01**2
    for meas in zip(observed_x, observed_y):
        x, P = kalman_xy(x, P, meas, R)
        result.append((x[:2]).tolist())
    kalman_x, kalman_y = zip(*result)
'''



'''
KALMAN STUFF ENDS HERE
'''



#updates the angle of the pointer
def updateAngle(x,y,nextx,nexty):
    return math.atan2(nexty-y,nextx-x)

def calcDirection(x,y,angle):
    rotx=x + (radius-1) * math.cos(math.radians(angle))
    roty=y + (radius-1)* math.sin(math.radians(angle))
    return rotx, roty

def makeWalls(wall):
    pygame.draw.line(screen, BLACK,(wall[0],wall[1]),(wall[2],wall[3]),2)
#Checks if there is a collision
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

#updates the position of the robot based on its motion
def updatePosition(x,y,xMotion,yMotion):
    x = x + xMotion
    y = y + yMotion
    xpos = int(x)
    ypos = int(y)

    # Maybe plot points later

    return x,y

def Motion(angle):
    xBelieveMotion = speed* math.cos(math.radians(angle))
    yBelieveMotion = speed* math.sin(math.radians(angle))
    
    return xBelieveMotion, yBelieveMotion

#Changes direction randomnly based on the angles it comes in and which wall it hits
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


# Finds the next direction of the robot
def findNextDirection(xMotion,yMotion,directionAngle,d,wall):
    
    xMotion,yMotion,directionAngle = changeDirection(xMotion,yMotion,directionAngle,wall)
    for i in range(0,8):
        d = CheckCollision(wallList[i],x+xMotion,y+yMotion)           
    return xMotion,yMotion,directionAngle

# After a wall hit, I move it back in the field as it sometimes likes to jump out
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

# If it gets stuck in a wall, which it sometimes does on corners, I move it a little in the right direction
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

# Sensor Inputs, returns the endpoints of the sensor lines
def sensors(centerX, centerY):
    sensorX = 0
    sensorY = 0

    for i in range(24):
        #point1 = centerX, centerY
        distance = getSensorDistance(i,centerX,centerY)
        #point2 = centerX + distance[0] * math.cos(i * 15*3.14/180+ math.radians(angle)), centerY + distance[0] * math.sin(i * 15 * 3.14/180+math.radians(angle))
        #pygame.draw.line(screen,  BLACK, point1, point2, 1)
        sensorEpX = centerX + distance[0] * math.cos(i * 15*3.14/180+math.radians(angle))
        sensorEpY = centerY + distance[0] * math.sin(i * 15 * 3.14/180+math.radians(angle))

        sensorX = sensorX + centerXEstimatesFromCensor(i, sensorEpX, distance[1], math.radians(angle))
        sensorY = sensorY + centerYEstimatesFromCensor(i, sensorEpY, distance[1], math.radians(angle))

    return sensorX/24, sensorY/24


# Return estimates of X and Y based on the sensor readings, including the noise
# Use this along with odometry estimates to feed into KF and , determine the position of the robot
def centerXEstimatesFromCensor(i, sensorEpX, distance, angle):
    return sensorEpX - distance*math.cos(i * 15*3.14/180 + angle)

def centerYEstimatesFromCensor(i, sensorEpY, distance, angle):
    return sensorEpY - distance*math.sin(i * 15*3.14/180 + angle)

def getSensorDistance(sensor,x,y):
    closestWall = 0
    closestWallDistance =1000
    distance = 0
    
    
    for i in range(0,8):
        
        x1 = wallList[i][0]
        y1 = wallList[i][1]
        x2 = wallList[i][2]
        y2 = wallList[i][3]

 
        line1 = LineString([(x1,y1), (x2,y2)])
        line2 = LineString([(x,y), (x + 1000 * math.cos(sensor * 15*3.14/180+math.radians(angle)), y + 1000 * math.sin(sensor * 15 * 3.14/180+math.radians(angle)))])
        point = line1.intersection(line2)
        point = numpy.array(point)
        
        if len(point) > 0:
            distance = ((point[0] - x)**2 + (point[1] - y)**2)**(1/2)
            if distance < closestWallDistance:
                closestWallDistance = distance
                closestWall = i

    # Introduce Noise
    return [randint(85, 115)*closestWallDistance*(1/100), closestWallDistance]

def SampleNormalDistribution(b) :
    value =0
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
    

    return x ,y,angle



#angle = updateAngle(x,y,x+1,y)
#rotx, roty = calcDirection(x,y)
stuckInWall = 0;





# Loop until the user clicks the close button.
done = False
while done < 7:
    # Set the screen background
    screen.fill(WHITE)

    #wallList Makes walls
    for wall in wallList:
        makeWalls(wall)

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


    

    if (believeY < pointsToVisit[done][1]-3 or believeY > pointsToVisit[done][1]+3) or (believeX < pointsToVisit[done][0]-3 or believeX >pointsToVisit[done][0]+3):
        rot1 = updateAngle(believeX, believeY, pointsToVisit[done][0],pointsToVisit[done][1])
        rot2 = 0
        rot1 = (math.degrees(rot1) - angle)
        xBelieveMotion, yBelieveMotion = Motion(rot1+angle)
        believeX, believeY = updatePosition(believeX,believeY,xBelieveMotion,yBelieveMotion)

        x,y,angle = sampleMotionModel(rot1,rot2,speed,x,y,angle)
        rotx,roty = calcDirection(x,y,angle)
        xpos = int(x)
        ypos = int(y)

        measurementX, measurementY = sensors(xpos,ypos)
        X, P = kalman_xy(X, P, (measurementX, measurementY) , R)

        believeX = X[0, 0]
        believeY = X[1, 0]

        xpos2 = int(believeX)
        ypos2 = int(believeY)
        rotx2, roty2 = calcDirection(believeX,believeY, angle+rot1)

    else:
        done = done + 1

    
    
 #   xpos,ypos,x,y = updatePosition(x,y,xMotion,yMotion,angle)
 #   angle = updateAngle(oldx,oldy,x,y)
 #   rotx,roty= calcDirection(x,y) # this is for the pointer

    circle = pygame.draw.circle(screen, GREEN , (xpos2,ypos2), radius,0)
    circle = pygame.draw.circle(screen, BLACK , (xpos2,ypos2), radius,2)
    #line = pygame.draw.line(screen,BLACK, (xpos2,ypos2),(rotx2, roty2),2)

    
    circle = pygame.draw.circle(screen, WHITE , (xpos,ypos), radius,0)
    circle = pygame.draw.circle(screen, BLACK , (xpos,ypos), radius,2)
 #   line = pygame.draw.line(screen,BLACK, (xpos,ypos),(rotx, roty),2)


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
