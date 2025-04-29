import cv2
import numpy as np
import time
import serial
from picamera2 import Picamera2
cam = Picamera2()
cam.preview_configuration.main.size=(1920,1080)
cam.preview_configuration.main.format = "RGB888"
cam.preview_configuration.align()
cam.configure("preview")
cam.start() 

### GLOBAL VARIABLES
# HSV ranges for various colours used
lower_magenta = np.array([140, 50, 50])
upper_magenta = np.array([170, 255, 255])

lower_green = np.array([58, 97, 10])
upper_green = np.array([85, 255, 255])

lower_blue = np.array([90, 123, 97])
upper_blue = np.array([122, 255, 255])

#transformation parameters
R=0
T=0
kinv=0


#FUNCTION TO TRANSFORM TO WORLD COORDINATES
def worldcoord(xp,yp,ZW):
    Pc = np.array([[xp],[yp],[1]])
    a = np.matmul(np.transpose(R),np.matmul(kinv,Pc))
    scale = (ZW+b[2])/a[2]
    Pw = scale*np.matmul(np.transpose(R),np.matmul(kinv,Pc)) - np.matmul(np.transpose(R),T)
    return (Pw[0],Pw[1])
    
    

#PID CONTROL
def moveto(x_t,y_t):
    #PID error terms
    ie=0    #integral error
    e_prev=0    #error at previous time
    prev_t=time.time()  #previous time
    curr_t=0    #variable to store current time
    #PID Paramteres
    kp=1
    ki=0
    kd=0
    while True:
        # Read a frame from the camera
        frame = cam.capture_array()
        
        # Convert the frame from BGR to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Threshold the HSV image to get only magenta color
        mask = cv2.inRange(hsv, lower_magenta, upper_magenta)
        mask=cv2.dilate(mask,(3,3))

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through the contours and estimate the coordinates of the robot midpoint 
        for contour in contours:
            # Approximate the contour to reduce the number of points
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximation has three points
            if len(approx) == 3: #If triangle
                xm=0 #coordinates in world frame
                ym=0
                xmc=0 #coordinates in camera frame
                ymc=0
                corW = [] #for temporarily storing coordinates of the triangle's all 3 vertices 
                for point in approx:
                    Pc = np.array([[point[0][0]],[point[0][1]],[1]])
                    a = np.matmul(np.transpose(R),np.matmul(kinv,Pc))
                    ZW=12 # Zw of robot's top
                    scale = (ZW+b[2])/a[2]
                    Pw = scale*np.matmul(np.transpose(R),np.matmul(kinv,Pc)) - np.matmul(np.transpose(R),T)
                    xmc+=point[0][0]
                    ymc+=point[0][1]
                    xm+=Pw[0]
                    ym+=Pw[1]
                    corW.append([Pw[0],Pw[1]])
                    
                xm/=3 # for stroing midpoint of the bot
                ym/=3
                xmc/=3
                ymc/=3
                maxd=0 # distance from the cenntroid to the apex of the isoceles triangle 
                i=0
                idx=0
                for point in corW:
                    d = np.sqrt(np.power(point[0]-xm,2)+np.power(point[1]-ym,2))
                    if d>maxd:
                        maxd=d
                        idx=i
                    i+=1
                    
                theta_t = (np.arctan2(y_t-ym,x_t-xm)) # desired orientation 
                theta_c = ((np.arctan2(((corW[idx])[1]-ym),((corW[idx])[0]-xm)))) # current body orientation
                if(np.sqrt(np.power(y_t-ym,2)+np.power(x_t-xm,2))<17): #stops when bot is near the target
                    ser.write(int(255).to_bytes(2,byteorder='little'))
                    return True
                else:
                    e = theta_t-theta_c # current error value
                    t_now = time.time() 
                    de = (e-e_prev)/(t_now-prev_t) # differential error
                    ie+=e*(t_now-prev_t) # integral error computation
                    u = (kp*e)+ki*ie+kd*de # PID controller output

                    if(u==1):
                        LS=10
                        RS=10
                    else:
                        LS=int(min(max(10*(1-u), -50), 50))
                        RS=int(min(max(10*(1+u), -50), 50))
                    print(LS,RS)
                    sendbl(LS,RS)
                    prev_t = t_now
                    e_prev = e



#BLUETOOTH
serial_port = "/dev/rfcomm7"
baud_rate=9600

try:
    ser = serial.Serial(serial_port,baud_rate,timeout=5)
    print("connected")

except serial.SerialException as e:
    print("failed:",e)
    exit()

#Function to encode and send the motor speeds to the propeller 
def sendbl(l,r):
    ld= np.sign(l)
    rd = np.sign(r)
    l=np.abs(l)
    r=np.abs(r)
    l=int((((l<<1)|np.sign(ld+np.abs(ld)))<<1)|0)
    r=int((((r<<1)|np.sign(rd+np.abs(rd)))<<1)|1)
    lbyt = l.to_bytes(2,byteorder='little')
    rbyt = r.to_bytes(2,byteorder='little')
    ser.write(lbyt)
    ser.write(rbyt)



#CALIBRATION WITH FOUR CHECKERS
#CAMERA COORDINATES
xc=[] 
yc=[]
K = np.array([[1449.414367,0,979.0583239],[0,1448.418209,530.4624053],[0,0,1]])
kinv = np.linalg.inv(K)
fx=[0,960,960,0]
fy=[540,540,0,0]
for fno in range(4):
    print(fno)
    while True:
        img = cam.capture_array()
        frame = img[fy[fno]:fy[fno]+540,fx[fno]:fx[fno]+960]
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,corners = cv2.findChessboardCorners(gray,(8,6),None)
        if ret:
            i=0
            for corner in corners:
                x=int(corner[0][0])
                y=int(corner[0][1])
                cv2.circle(frame,(x,y),2,(0,0,255),1)
                xc.append(x+fx[fno])
                yc.append(y+fy[fno])
                i+=1
            break
xw=[]
yw=[]
oxw=[82,82,0]#81.5,
oyw=[0,71,71]#-2.4,75.1,75
for i in range(0,8):
    xw.append(i*2.4)
xw = xw*6
for i in  range(0,6):
    yw=[*yw,*([(5-i)*2.4]*8)]

for bn in range(3):
    for i in range(48):
        xw.append(xw[i]+oxw[bn])
        yw.append(yw[i]+oyw[bn])

print(kinv)
print(len(xc),len(yc),len(xw),len(yw))

#Finding h
A=[]
for i in range(len(xw)):
    A.append([xw[i],yw[i],1,0,0,0,-xc[i]*xw[i],-xc[i]*yw[i],-xc[i]])
    A.append([0,0,0,xw[i],yw[i],1,-yc[i]*xw[i],-yc[i]*yw[i],-yc[i]])
A=np.array(A)
print(A.shape)
U1,S1,Vt1 = np.linalg.svd(A,full_matrices=False)
H=[Vt1[8,0:3],Vt1[8,3:6],Vt1[8,6:9]]

#Finding Rotation and Tranlation matrices from h
h = np.matmul(kinv,H)
R1 = np.array([h[0,0],h[1,0],h[2,0]])
R2 = np.array([h[0,1],h[1,1],h[2,1]])
T = np.array([h[0,2],h[1,2],h[2,2]])
U,_,Vt = np.linalg.svd(np.transpose([R1,R2,np.cross(R1,R2)]))
R = np.matmul(U,np.matmul(np.array([[1,0,0],[0,1,0],[0,0,np.linalg.det(np.matmul(U,Vt))]]),Vt))
T = T/np.linalg.norm(R1)
T= np.transpose([T])
print(T)
b = np.matmul(np.transpose(R),T)
print('calibrated')
###CALIBRATION DONE

# FINDING CUBES AND PATCHES

# PATCHES
GP=0
BP=0
GC=[]
BC=[]

frame = cam.capture_array()    
# Convert the frame from BGR to HSV color space
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_green, upper_green)
contours_green, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

mask = cv2.inRange(hsv, lower_blue, upper_blue)
contours_blue, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours_green:
        contour_area = cv2.contourArea(cnt)
        
        if contour_area > 500 and contour_area < 2000:  # Adjust the threshold as needed
           
            M = cv2.moments(cnt)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            GC.append(worldcoord(centroid_x,centroid_y,2.5))
        
    
        elif contour_area > 5000:
            M = cv2.moments(cnt)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            GP = worldcoord(centroid_x,centroid_y,2.5)

for cnt in contours_blue:
        contour_area = cv2.contourArea(cnt)
        
        if contour_area > 500 and contour_area < 2000:  # Adjust the threshold as needed
           
            M = cv2.moments(cnt)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            BC.append(worldcoord(centroid_x,centroid_y,2.5))
        
    
        elif contour_area > 5000:
            M = cv2.moments(cnt)
            centroid_x = int(M['m10'] / M['m00'])
            centroid_y = int(M['m01'] / M['m00'])
            BP = worldcoord(centroid_x,centroid_y,2.5)
    
for C in GC:
    moveto(C[0],C[1])
    ser.write(int(253).to_bytes(2,byteorder='little')) #pickup
    time.sleep(3)
    moveto(GP[0],GP[1])
    ser.write(int(254).to_bytes(2,byteorder='little')) #place
    time.sleep(3)

for C in BC:
    moveto(C[0],C[1])
    ser.write(int(253).to_bytes(2,byteorder='little')) #pickup
    time.sleep(3)
    moveto(BP[0],BP[1])
    ser.write(int(254).to_bytes(2,byteorder='little')) #place
    time.sleep(3)
    



