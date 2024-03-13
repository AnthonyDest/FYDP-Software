import RPi.GPIO as gpio
from time import sleep

# pins use GPIO Number convention (BCM)

gpio.setmode(gpio.BCM)

MOTOR_PWM_PIN = 25 # goes to enable
MOTOR_IN1_PIN = 23
MOTOR_IN2_PIN = 24

# gpio.setup(self.pwm_pin, gpio.OUT)
gpio.setup(MOTOR_IN1_PIN, gpio.OUT)
gpio.setup(MOTOR_IN2_PIN, gpio.OUT)
gpio.setup(MOTOR_PWM_PIN, gpio.OUT)


p=gpio.PWM(MOTOR_PWM_PIN,125)
p.start(0)
# initalize to forward
gpio.output(MOTOR_IN1_PIN,gpio.HIGH)
gpio.output(MOTOR_IN2_PIN,gpio.LOW)
print("Setup")

sleep(1)
print("RUN")
p.start(80)
# current_time = t

sleep(3)

# while True:
#     print("RUN")
print("END")
gpio.cleanup()


# while True:
#     print("RUN")
