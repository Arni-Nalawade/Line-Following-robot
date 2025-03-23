from gpiozero import Button
from time import sleep

#Libraries for line follower
from MotorModule import Motor
import WebcamModule
import cv2
from LaneDetectionModule import getLaneCurve

#Global Variables
global prev_curveVal
grip = DigitalOutputDevice(25)
grip.off()
request_encode = DigitalOutputDevice(23)
complete_encode = DigitalInputDevice(24)
request_encode.off()

# Initialize Motor object globally
motor = Motor(12, 5, 6, 13, 0, 1)

# Button to GPIO pin
White = Button(20,pull_up = True) #Servo
Green = Button(21,pull_up = True) #Encoder
Red = Button(16,pull_up = True) #PID controller

#Servo Segment
def Servo_run():
    grip.on()
    sleep(1) #wait for gripper press
    Motor.move(0.05,0,1) #move forward
    sleep(0.1)
    Motor.move(0, 0.5, 1) #turn left
    sleep(0.1)
    grip.off()
    sleep(1) #wait for gripper release
    return None

def Encoder_run():
    request_encode.on()
    while complete_encode.value == 0: #Check if the pin is low.
        Motor.move(0, 0.05, 1)  # turn left
        sleep(0.01) #Small delay to prevent CPU overload.
    Motor.stop() #Stop the motor once the pin goes high.
    request_encode.off() #Turn off the request.
    return None

# Line follow segment
def MainRobotLane():
    import utlis
    import numpy as np
    import time  # for sleep

    initialTrackBarVals = [197, 115, 170, 231]
    utlis.initializeTrackbars(initialTrackBarVals)

    # Smoothing variables
    prev_curveVal = 0
    smoothing_factor = 0.8  # Increased smoothing
    max_curve_change = 0.01  # Reduced max curve change

    img = WebcamModule.getImg()
    curveVal = getLaneCurve(img, 1)

    # Red Line Detection
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red_lower = np.array([0, 130, 130])
    red_upper = np.array([10, 255, 255])
    red_lower_2 = np.array([160, 100, 100])
    red_upper_2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, red_lower, red_upper)
    mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    mask = cv2.bitwise_or(mask1, mask2)
    red_pixels = cv2.countNonZero(mask)

    sen = 0.15  # Further reduced sensitivity
    maxVAl = 1

    curveVal = max(min(curveVal, maxVAl), -maxVAl)

    # Apply smoothing
    smoothed_curveVal = (smoothing_factor * curveVal) + ((1 - smoothing_factor) * prev_curveVal)

    # Limit rate of change
    curve_change = smoothed_curveVal - prev_curveVal
    if abs(curve_change) > max_curve_change:
        smoothed_curveVal = prev_curveVal + np.sign(curve_change) * max_curve_change

    curveVal = smoothed_curveVal
    prev_curveVal = curveVal

    print("Curve Value:", curveVal)

    if abs(curveVal) < 0.02:  # Adjusted Deadzone
        curveVal = 0

        turnVal = curveVal / maxVAl if maxVAl != 0 else 0

        baseSpeed = 0.04  # Reduced base speed

    # Conditional Motor Control (Follow red line)
    if red_pixels > 1000:
        motor.move(baseSpeed, turnVal * sen)
    else:
        print("Red line lost! Searching...")
        search_time = 1.0  # Time to search (adjust as needed)
        start_time = time.time()
        while time.time() - start_time < search_time:
            # Rotate in place (adjust speed and direction)
            motor.move(0, 0.3)  # Reduced search rotation speed
            img = WebcamModule.getImg()
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, red_lower, red_upper)
            mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
            mask = cv2.bitwise_or(mask1, mask2)
            red_pixels = cv2.countNonZero(mask)
            if red_pixels > 1000:
                print("Red line found!")
                return  # exit main function, and return to normal line following.
        motor.stop()  # if search fails, stop.

    cv2.waitKey(1)



try:
    while True:
        if White.is_pressed:
            while White.is_pressed:
                sleep(0.01)
            print("Servo start")
            Servo_run()
        elif Green.is_pressed:
            while Green.is_pressed:
                sleep(0.01)
            print("Encoder start")
            # Add code here to execute when the button is not pressed.
            # For example, you could turn off an LED.
        elif Red.is_pressed:
            while Red.is_pressed:
                sleep(0.01)
            print("Line follower activated")
            MainRobotLane()

        sleep(0.5)  # Check every 0.5 seconds 

except KeyboardInterrupt:
    print("Program stopped by user.")

finally:
    # Cleanup
    White.close()
    Green.close()
    Red.close()