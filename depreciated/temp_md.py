import motor_driver
from time import sleep
# pins use GPIO Number convention (BCM)



MOTOR_PWM_PIN = 23 # goes to enable
MOTOR_IN1_PIN = 25
MOTOR_IN2_PIN = 24

motor = motor_driver.Motor_Driver(pwm_pin=MOTOR_PWM_PIN, in_1_pin=MOTOR_IN1_PIN, in_2_pin= MOTOR_IN2_PIN)

print("Setup")

sleep(1)
print("RUN")
motor.set_speed(80)
# current_time = t

sleep(1)

# while True:
#     print("RUN")
print("END")
motor.close()


# while True:
#     print("RUN")
