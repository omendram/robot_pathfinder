# Import a library of functions called 'pygame'
import pygame
import random, sys, time, math, numpy
from numpy import linalg
from random import randint
 
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

angle = 50
speed = 3


x = 74
y = 100
xMotion =speed* math.cos(math.radians(angle))
yMotion =speed* math.cos(math.radians(angle));
radius = 50
rotx=0
roty=0
xpos=75
ypos=60
directionAngle =0
#    or (y0 >= y1-50 and y0 <= y2+50) wallList = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[265,10,265,238],[269,10,269,234],[269,234,536,234],
 #          [265,238,532,238],[532,238,532,470],[536,238,536,474],[532,470,265,470],[536,474,265,474],[799,710,799,236],[803,710,803,236],[799,236,803,236]]
wallList = [[10,10,1070,10],[10,10,10,710],[10,710,1070,710],[1070,10,1070,710],[267,10,267,236],[534,236,534,472],[267,472,534,472],[801,236,801,710]]




#updates the angle of the pointer
def updateAngle(x,y,nextx,nexty):
    return math.atan2(nexty-y,nextx-x)

def calcDirection(x,y):
    rotx=x + (radius-1) * math.cos(angle)
    roty=y + (radius-1)* math.sin(angle)
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
    return xpos,ypos,x,y


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
        y= y+speed
    if directionAngle < 180 and wallList[wall][1] == wallList[wall][3]:
        y = y -speed
    if (directionAngle > 270 or (directionAngle > 0 and directionAngle < 90)) and wallList[wall][0] == wallList[wall][2]:
        x = x-speed
    if (directionAngle <= 270 and directionAngle > 90) and wallList[wall][0] == wallList[wall][2]:
        x= x+speed
    return x,y

# If it gets stuck in a wall, which it sometimes does on corners, I move it a little in the right direction
def stuck(directionAngle,wall,x,y):
    if directionAngle >= 180 and wallList[wall][0] == wallList[wall][2]:
        y= y+radius/2
    if directionAngle < 180 and wallList[wall][0] == wallList[wall][2]:
        y = y -radius/2
    if (directionAngle > 270 or (directionAngle > 0 and directionAngle < 90)) and wallList[wall][1] == wallList[wall][3]:
        x = x-radius/2
    if (directionAngle <= 270 and directionAngle > 90) and wallList[wall][1] == wallList[wall][3]:
        x= x+radius/2
    return x,y
  

        
    
        
    
    
angle = updateAngle(x,y,x+1,y)
rotx, roty = calcDirection(x,y)
stuckInWall = 0;





# Loop until the user clicks the close button.
done = False
while not done:
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

            if(stuckInWall > 0 and i <= 3):
                if i ==0:
                    x = x + radius/2
                    y = y + radius/2
                    
                if i  == 1:
                    x = x - radius/2
                    y = y - radius/2
                if i == 2:
                    x = x - radius/2
                    y = y - radius/2
                if i == 3:
                    x = x + radius/2
                    y = y + radius/2
                break   
            stuckInWall = 3
            x,y = adjustPosition(directionAngle,i,x,y)
            xMotion, yMotion,directionAngle = findNextDirection(xMotion,yMotion,directionAngle,d,i)
            
            break


    for event in pygame.event.get():   # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True   # Flag that we are done so we exit this loop
    oldx = x
    oldy = y
    
    xpos,ypos,x,y = updatePosition(x,y,xMotion,yMotion)
    angle = updateAngle(oldx,oldy,x,y)
    rotx,roty= calcDirection(x,y) # this is for the pointer

    circle = pygame.draw.circle(screen, PINK , (xpos,ypos), radius,0)
    circle = pygame.draw.circle(screen, BLACK , (xpos,ypos), radius,2)
    line = pygame.draw.line(screen,BLACK, (xpos,ypos),(rotx, roty),2)

    if (x > 1080 or x< 0 or y >720 or y < 0):
        x = 74
        y=100
    if stuckInWall > 0:
        stuckInWall = stuckInWall -1
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    clock.tick(60)
 
# Be IDLE friendly. If you forget this line, the program will 'hang'
# on exit.
pygame.quit()






