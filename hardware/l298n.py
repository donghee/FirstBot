import RPi.GPIO as GPIO
import time

# Set pin output direction
GPIO.setmode(GPIO.BCM)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# Forward 
GPIO.output(12, GPIO.HIGH)
GPIO.output(13, GPIO.HIGH)

GPIO.output(21, GPIO.HIGH)
GPIO.output(20, GPIO.LOW)
GPIO.output(23, GPIO.HIGH)
GPIO.output(24, GPIO.LOW)

time.sleep(3)

GPIO.output(12, GPIO.LOW)
GPIO.output(13, GPIO.LOW)

# Backward
GPIO.output(12, GPIO.HIGH)
GPIO.output(13, GPIO.HIGH)

GPIO.output(21, GPIO.LOW)
GPIO.output(20, GPIO.HIGH)
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.HIGH)

time.sleep(3)

GPIO.output(12, GPIO.LOW)
GPIO.output(13, GPIO.LOW)
