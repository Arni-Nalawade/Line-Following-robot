import cv2
import numpy as np
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

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
kp = 0.005  # Proportional gain
ki = 0 # Integral gain
kd = 0.005  # Derivative gain

# Target horizontal position (center of the frame)
target_cx = 80  # Half of the frame width

# PID variables
integral = 0
previous_error = 0
bufferL = 5
bufferR = 5

# Motor speed (0 to 1)
base_speed = 0
base_speed_max = 0.2
accel = 0.007 # Increase by this speed every cycle
deccel = 0.007 # Decrease by this speed every 

# Initial motor state (stop)
en1.value = 0
en2.value = 0
fdL.on()
rvL.off()
fdR.on()
rvR.off()


while True:
    ret, frame = cap.read()
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
            print("Error:", error)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
            previous_error = error

        cv2.drawContours(frame, c, -1, (0, 255, 0), 1)

    else:
        print("I don't see the line")
        
        base_speed = base_speed - deccel

        if base_speed < 0:
            base_speed = 0

        en1.value = base_speed
        en2.value = base_speed
        

    cv2.imshow("Mask", red_mask)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        fdL.off()
        rvL.off()
        fdR.off()
        rvR.off()
        break

cap.release()
cv2.destroyAllWindows()