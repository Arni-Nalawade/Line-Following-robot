from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

class Motor:
    def __init__(self, ena_a, in1_a, in2_a, ena_b, in1_b, in2_b):
        """
        Initializes the Motor object with GPIO pins for enable and direction.

        Args:
            ena_a (int): GPIO pin for enable A (PWM).
            in1_a (int): GPIO pin for input 1A (direction).
            in2_a (int): GPIO pin for input 2A (direction).
            ena_b (int): GPIO pin for enable B (PWM).
            in1_b (int): GPIO pin for input 1B (direction).
            in2_b (int): GPIO pin for input 2B (direction).
        """
        self.ena_a = PWMOutputDevice(ena_a)
        self.in1_a = DigitalOutputDevice(in1_a)
        self.in2_a = DigitalOutputDevice(in2_a)
        self.ena_b = PWMOutputDevice(ena_b)
        self.in1_b = DigitalOutputDevice(in1_b)
        self.in2_b = DigitalOutputDevice(in2_b)
        self.my_speed = 0

    def move(self, speed=0.5, turn=0, t=0):
        """
        Moves the motor with specified speed and turn values.

        Args:
            speed (float): Forward/backward speed (-1 to 1).
            turn (float): Turn value (-1 to 1).
            t (float): Time to move (seconds).
        """
        left_speed = speed - turn
        right_speed = speed + turn

        left_speed = max(min(left_speed, 1), -1)
        right_speed = max(min(right_speed, 1), -1)

        self.ena_a.value = abs(left_speed)
        self.ena_b.value = abs(right_speed)

        if left_speed > 0:
            self.in1_a.on()
            self.in2_a.off()
        else:
            self.in1_a.off()
            self.in2_a.on()

        if right_speed > 0:
            self.in1_b.on()
            self.in2_b.off()
        else:
            self.in1_b.off()
            self.in2_b.on()

        sleep(t)

    def stop(self, t=0):
        """
        Stops the motor.

        Args:
            t (float): Time to stop (seconds).
        """
        self.ena_a.value = 0
        self.ena_b.value = 0
        self.my_speed = 0
        sleep(t)

def main():
    motor.move(0.5, 0, 2)
    motor.stop(2)
    motor.move(-0.5, 0, 2)
    motor.stop(2)
    motor.move(0, 0.5, 2)
    motor.stop(2)
    motor.move(0, -0.5, 2)
    motor.stop(2)

if __name__ == '__main__':
    motor = Motor(12,5,6,13,0,1)  # Replace with actual GPIO pins
    main()