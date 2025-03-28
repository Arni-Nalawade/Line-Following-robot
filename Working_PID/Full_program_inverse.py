import cv2
import numpy as np
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice, Button
from time import sleep

#### INITIALIZATION ####
# Camera setup
cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

# Motor setup using GPIOZero
fdL = DigitalOutputDevice(5) #motor 1 forward
rvL = DigitalOutputDevice(6) #motor 1 backward
fdR = DigitalOutputDevice(0) #motor 2 forward
rvR = DigitalOutputDevice(1) #motor 2 backward
en1 = PWMOutputDevice(12) #motor 1 PWM
en2 = PWMOutputDevice(13) #motor 2 PWM

# PID parameters
kp = 0.0035  # Proportional gain
ki = 0 # Integral gain
kd = 0.007  # Derivative gain

# Target horizontal position (center of the frame)
target_cx = 80  # Half of the frame width

# PID variables
integral = 0
previous_error = 0
bufferL = 5
bufferR = 5
left_speed = 0
right_speed = 0

# Motor speed (0 to 1)
base_speed = 0
base_speed_max = 0.1
accel = 0.007 # Increase by this speed every cycle
deccel = 0.007 # Decrease by this speed every 

# Initial motor state (stop)
en1.value = 0
en2.value = 0
fdL.on()
rvL.off()
fdR.on()
rvR.off()

#Global Variables
global prev_curveVal
grip = DigitalOutputDevice(25)
grip.on()
request_encode = DigitalOutputDevice(23)
complete_encode = DigitalInputDevice(24)
request_encode.on()
counter = 0

# Button to GPIO pin
White = Button(20,pull_up = True) #Servo
Green = Button(21,pull_up = True) #Encoder
Red = Button(16,pull_up = True) #PID controller

#Stop All motors
def stop_robot():
    en1.value = 0
    en2.value = 0
    fdL.off()
    rvL.off()
    fdR.off()
    rvR.off()
    return None

#Encoder function to turn
def Encoder_run():
    request_encode.off()
    sleep(0.5)
    request_encode.on() #Turn off the request.
    en1.value = base_speed_max
    en2.value = base_speed_max
    fdL.off()
    rvL.on()
    fdR.on()
    rvR.off() 
    while complete_encode.value == 1: #Check if the pin is low.
        print ("in progress")
        sleep(0.01)
    print ("Encoder complete")
    stop_robot() #Stop the motor once the pin goes high.
    #request_encode.off() #Turn off the request.
    return None

#Servo function to grip
def Servo_grip():
    grip.off()
    sleep(1) #wait for gripper press
    return None

#Servo function to grip
def Servo_release():
    grip.on()
    sleep(1) #wait for gripper press
    return None

#PID function
def PID(frame):
    #Giving access to global variables
    global kp
    global ki
    global kd
    global target_cx
    global integral
    global previous_error
    global bufferL
    global bufferR
    global base_speed 
    global base_speed_max 
    global accel 
    global deccel
    global drive
    global left_speed
    global right_speed

    ret, frame = cap.read()
    finish = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color detection (two ranges)
    red_mask_1 = cv2.inRange(hsv, np.array([0, 130, 130]), np.array([10, 255, 255]))
    red_mask_2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)  # Combine masks

    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            #base_speed = base_speed_max
            if base_speed < base_speed_max:
                base_speed = base_speed + accel

            if base_speed > base_speed_max:
                base_speed = base_speed_max

            fdL.on()
            rvL.off()
            fdR.on()
            rvR.off()

            # Calculate error
            error = target_cx - cx +70
            #if abs(error) > 110:
                #error = 0

            # PID calculations
            integral += error
            derivative = error - previous_error

            output = kp * error + ki * integral + kd * derivative

            # Motor control
            left_speed = base_speed + output
            right_speed = base_speed - output

            # Clamp speeds to valid range (0 to 1)
            left_speed = max(0, min(1, left_speed))
            right_speed = max(0, min(1, right_speed))

            if error < -bufferL:  # Turn left
                en1.value = left_speed
                en2.value = right_speed
            elif error > bufferR:  # Turn right
                en1.value = left_speed
                en2.value = right_speed
            else:  # On track
                en1.value = base_speed
                en2.value = base_speed

            # Display error
           # if abs(error) > 100:
            print("Error:", error)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
            previous_error = error

        cv2.drawContours(frame, c, -1, (0, 255, 0), 1)

    else:
        print("I don't see the line")
    
        base_speed = base_speed - deccel

        if base_speed < 0:
            base_speed = 0

        #en1.value = base_speed
        #en2.value = base_speed
        en1.value = left_speed
        en2.value = right_speed
        #counter += 1

        if base_speed == 0:
            #fdL.off()
            #fdR.off()
            stop_robot()
            finish = 1
    return finish
    
# Function to stop if blue line
def detect_blue(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
     # Blue color detection (two ranges)
    blue_mask_1 = cv2.inRange(hsv, np.array([109,82,139]), np.array([135,255,216]))
    blue_mask_2 = cv2.inRange(hsv, np.array([109,82,139]), np.array([135,255,216]))
    blue_mask = cv2.bitwise_or(blue_mask_1, blue_mask_2)  # Combine masks

    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        return 1  # Blue line detected
    else:
        return 0  # No blue line detected
    

## Main Code
if __name__ == '__main__':
    try:
        while True:
            if Green.is_pressed:
                while Green.is_pressed:
                    sleep(0.01)
                print("Line follower activated")
                blue_detected = 0
                while blue_detected == 0: #Keep doing PID until Blue mark
                    ret, frame = cap.read()
                    if not ret:
                        break
                    PID(frame)

                    blue_detected = detect_blue(frame)
                    #cv2.imshow("Frame", frame)

                stop_robot() #Stop robot

                #Drive forward
                fdL.on()
                fdR.on()
                en1.value = base_speed_max
                en2.value = base_speed_max
                sleep(1)
                stop_robot()

                Servo_grip() #Grab lego man
                Encoder_run() #180 turn

                stop_robot() #stop robot

                #Drive backwards
                #rvL.on()
                #rvR.on()
                #en1.value = base_speed_max
                #en2.value = base_speed_max
                #sleep(0.6)

                #stop_robot()
                sleep(0.1)

                #sleep(5)
                finish = 0

                while finish == 0:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    finish = PID(frame)
                    #cv2.imshow("Frame", frame)

                if finish == 1:
                    rvL.on()
                    en1.value = base_speed_max
                    en2.value = base_speed_max
                    sleep(0.8)
                    stop_robot()
                    Servo_release() #drop lego man

                print("Done!!")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        stop_robot()
        cap.release()
        cv2.destroyAllWindows()