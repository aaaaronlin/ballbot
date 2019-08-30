import cv2, imutils
import numpy as np
import time, serial, sys

class PID:
    def __init__(self, P, I, D):
        self.kP = P
        self.kI = I
        self.kD = D
        self.E = 0
        self.e = 0
        self.e_old = 0
        self.e_dot = 0
        
    def update(self, e_in):
        self.e = e_in
        self.E = self.E + self.e
        self.e_dot = self.e - self.e_old
        return self.kP*self.e + self.kI*self.E + self.kD*self.e_dot
    def restart(self):
        self.E = 0
        self.e = 0
        self.e_old = 0
        self.e_dot = 0
        
# define ball parameters
lower_blue = np.array([91, 120, 38])
upper_blue = np.array([126, 255, 255])
ballDia = 0.056 # ball diameter in m
frameWidth = 500

# define algorithm parameters
sampleTime = 0.5 # how long (sec) to find contours before calculating
zScoreThreshold = 3 # removing wrong contours
kP = 8
kI = 0.5
kD = 0.1
BotAngle = PID(kP, kI, kD)

# define robot
vel = 0.2 # m/s
wheelRad = 0.035 # meters
wheelBase = 0.125 # meters
searchAngVel = 1.2 # rad/s
brakeTime = 0.5 # time in s to brake
clawClose = 50 # angle at which claw is closed
clawOpen = 2 # angle at which claw is open
grabDist = 0.07 # dist in m to grab ball

# toggle Video
showVideo = True

# helpful variables
focalLength = frameWidth*0.065 # held ball up to camera and measured distance in meters
ballMax = (150000/56)*ballDia
frameHeight = (480/640)*frameWidth

# setup
ser = serial.Serial('/dev/ttyACM0', 9600);
cap = cv2.VideoCapture(0)
frame = None
states = ["SEARCH", "GO TO BALL", "GRAB"]
state = "SEARCH"
ballAchieved = False

def getBall():
    global cap, frame
    t = time.time()
    frames = 0 # get ~18 fps on Raspberry Pi 4
    distArray = []
    while time.time() - t < sampleTime:
        # get next frame
        [ret, frame] = cap.read()
        frame = imutils.resize(frame, width=frameWidth)
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV,lower_blue,upper_blue)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        if len(contours) == 0:
            continue
        for contour in contours:
	        perimeter = cv2.arcLength(contour, True)
	        area = cv2.contourArea(contour)
	        # make sure the contour is of good size and circularity
	        if perimeter != 0 and (4*3.1415*area/perimeter/perimeter) > 0.72 and area > 200:
	            ((pixelX, pixelY), pixelRad) = cv2.minEnclosingCircle(contour)
	            cv2.circle(frame, (int(pixelX), int(pixelY)), int(pixelRad), (0, 255, 255))
	            zDist = focalLength/(pixelRad*2)
	            xDist = (pixelX - frameWidth/2)*ballDia/(pixelRad*2)
	            distArray.append([xDist, zDist])
	            break; # next frame, tracking just one ball
        if showVideo:
            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.waitKey(1)
        frames+=1
    # z score test
    distArray = np.array(distArray)
    if len(distArray) <= 1:
        return distArray
    mean = np.mean(distArray, axis=0)
    std = np.std(distArray, axis=0, ddof=1)
    z_scores = np.array([(dist - mean)/std for dist in distArray])
    xDist = np.mean(distArray[:,0][abs(z_scores[:,0])<zScoreThreshold])
    zDist = np.mean(distArray[:,1][abs(z_scores[:,1])<zScoreThreshold])
    
    return np.array([xDist, zDist])

def updateMotorVel(v, w): # differential drive dynamics
    vel_r = (2*v + w*wheelBase)/2
    vel_l = (2*v - w*wheelBase)/2
    msg = "D," + str(np.round(vel_r, 4)) + "," + str(np.round(vel_l, 4)) + "n"
    print(msg)
    ser.write(msg.encode())
    
# wait for arduino callibration
v = ser.readline()
while v[0] != 'R':
    v = ser.readline().decode('utf-8')
    print(v)

print("Searching for ball...")

updateMotorVel(vel, searchAngVel)

while not ballAchieved:
    if state == "SEARCH":
        nearestBall = getBall()
        if len(nearestBall) == 0:
            continue;
        else:
            BotAngle.restart()
            state = "GO TO BALL"
    elif state == "GO TO BALL":
        print("Tracking Ball")
        ball = getBall()
        if len(ball) < 2:
            updateMotorVel(vel, searchAngVel)
            state == "SEARCH"
        else:
            dx = ball[0]
            dz = ball[1]
            if dz < grabDist:
                state = "GRAB"
                continue;
            theta = -np.arctan2(dx, dz)
            angVel = BotAngle.update(theta)
            updateMotorVel(vel, angVel)
    elif state == "GRAB":
        brakeMsg = "S," + str(brakeTime) + "n"
        ser.write(brakeMsg.encode())
        grabMsg = "C," + str(clawClose) + "n"
        ser.write(grabMsg.encode())
        grabMsg = "C," + str(clawOpen) + "n"
        ser.write(grabMsg.encode())
        ballAchieved = True
        print("Ball Found!")

    

