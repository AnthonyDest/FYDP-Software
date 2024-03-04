import RPi.GPIO as gpio
from time import sleep

# pins use GPIO Number convention (BCM)

gpio.setmode(gpio.BCM)

LEFT_MOTOR_PWM_PIN = 8 # goes to enable
LEFT_MOTOR_IN1_PIN = 7
LEFT_MOTOR_IN2_PIN = 12

# gpio.setup(self.pwm_pin, gpio.OUT)
gpio.setup(LEFT_MOTOR_IN1_PIN, gpio.OUT)
gpio.setup(LEFT_MOTOR_IN2_PIN, gpio.OUT)
gpio.setup(LEFT_MOTOR_PWM_PIN, gpio.OUT)


p=gpio.PWM(LEFT_MOTOR_PWM_PIN,1000)
p.start(0)
# initalize to forward
gpio.output(LEFT_MOTOR_IN1_PIN,gpio.HIGH)
gpio.output(LEFT_MOTOR_IN2_PIN,gpio.LOW)
print("Setup")
p.start(50)
sleep(1)
# current_time = t
print("RUN")
sleep(5)

# while True:
#     print("RUN")
print("END")
gpio.cleanup()


# while True:
#     print("RUN")