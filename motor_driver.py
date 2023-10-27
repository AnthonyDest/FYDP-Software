import RPi.GPIO as gpio

class Motor:
    def __init__(self, pwm_pin, en_pin):
        self.pwm_pin = pwm_pin
        self.en_pin = en_pin
        self.speed = 0
        gpio.setmode(gpio.BCM)
        gpio.setup(self.pwm_pin, gpio.OUT)
        gpio.setup(self.en_pin, gpio.OUT)
        self.pwm = gpio.PWM(self.pwm_pin, 1000)
        self.pwm.start(0)

    def set_speed(self, speed):
        # Set the motor speed using PWM with a duty cycle from 0 to 100
        self.speed = speed
        self.pwm.ChangeDutyCycle(speed)

    def move_forward(self):
        # Move the motor forward
        gpio.output(self.en_pin, gpio.HIGH)

    def move_backward(self):
        # Move the motor backward
        gpio.output(self.en_pin, gpio.LOW)

    def stop(self):
        # Stop the motor
        self.pwm.ChangeDutyCycle(0)

LEFT_MOTOR_PWM_PIN = 17
LEFT_MOTOR_EN_PIN = 22
left_motor = Motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_EN_PIN)

# def main():
#     left_motor = Motor(17, 22)

#     try:
#         # Move forward at 50% speed for 2 seconds
#         left_motor.move_forward()
#         left_motor.set_speed(50)
#         time.sleep(2)

#         # Stop the motor
#         left_motor.stop()

#     except KeyboardInterrupt:
#         pass

#     gpio.cleanup()

# if __name__ == "__main__":
#     main()
