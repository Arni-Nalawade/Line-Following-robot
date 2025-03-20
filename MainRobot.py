from Motormodule import Motor
from PID_Final import getLaneCurve
import WebcamModule
import cv2
 
##################################################
motor = Motor(12,5,6,13,0,1)
##################################################
 
def main():
 
    img = WebcamModule.getImg()
    curveVal= getLaneCurve(img,1)
 
    sen = 1.3  # SENSITIVITY (Steering amount)
    maxVAl= 0.3 # MAX SPEED (Speed of motors)
    if curveVal>maxVAl:curveVal = maxVAl
    if curveVal<-maxVAl: curveVal =-maxVAl
    #print(curveVal)
    if curveVal>0:
        sen =1.7
        if curveVal<0.05: curveVal=0 #Deadzones
    else:
        if curveVal>-0.08: curveVal=0 #Deadzones
    motor.move(0.20 ,-curveVal*sen,0.05) #Base speed
    cv2.waitKey(1)
     
 
if __name__ == '__main__':
    while True:
        main()