import serial
import time
from threading import Thread

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# Define the GPIO pin for the servo signal
SERVO_PIN = 18

# Set up the GPIO pin for PWM
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz (20 ms PWM period)
pwm.start(0)  # Initialize the PWM with 0 duty cycle

def set_angle(angle):
    # Calculate duty cycle from angle
    duty = 2 + (angle / 18)  # Adjust this calculation if necessary
    pwm.ChangeDutyCycle(duty)

def send_command(command, ser):
    ser.write(command.encode())


def motor_control():
        
    ser = serial.Serial('/dev/ttyAMA0')
    print("connected Pi to Arduino")
    try:
        while True:
            command = input("Enter controls(A S W D)")
            if command in ['A','S','W','D','Q','a','s','w','d','q']:
                send_command(command,ser)

            else:
                print("Invalid command")
    except KeyboardInterrupt:
        print("Program terminated")

    finally:
        ser.close()      

def servo_control():

    try:
        while True:
            # Rotate servo to 180 degrees
            set_angle(180)
            time.sleep(1)  # Wait for 1 second

            # Rotate servo to 0 degrees
            set_angle(0)
            time.sleep(1)  # Wait for 1 second

    except KeyboardInterrupt:
        pass

    finally:
        pwm.stop()
        GPIO.cleanup()


motor_thread = Thread(target=motor_control)
servo_thread = Thread(target=servo_control)

motor_thread.start()
servo_thread.start()

motor_thread.join()
servo_thread.join()



