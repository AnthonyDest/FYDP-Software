try:
    import RPi.GPIO as gpio
except ImportError:
    gpio = None
    print("RPi.GPIO not available. GPIO functionality will be disabled.")

# import RPi.GPIO as gpio


def check_hardware_OK():
    if gpio is None:
        print("GPIO Disabled")
        return False
    return True


# pins use GPIO Number convention (BCM)
LEFT_MOTOR_PWM_PIN = 16  # goes to enable
LEFT_MOTOR_IN1_PIN = 20
LEFT_MOTOR_IN2_PIN = 21


class Motor:
    def __init__(self, pwm_pin, in_1_pin, in_2_pin):
        self.pwm_pin = pwm_pin
        self.in_1 = in_1_pin
        self.in_2 = in_2_pin

        self.speed = 0
        if gpio is None:
            print("GPIO Disabled")
            return None
        # configure pins
        gpio.setmode(gpio.BCM)
        gpio.setup(self.pwm_pin, gpio.OUT)
        gpio.setup(self.in_1, gpio.OUT)
        gpio.setup(self.in_2, gpio.OUT)

        # initialize to forward
        gpio.output(self.in_1, gpio.HIGH)
        gpio.output(self.in_2, gpio.LOW)

        # start PWM, without input
        # self.pwm = gpio.PWM(self.pwm_pin, 50)
        self.pwm = gpio.PWM(self.pwm_pin, 125)
        self.pwm.start(0)

    def set_speed(self, duty_cycle):
        # limit duty cycle
        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle > 100:
            duty_cycle = 100

        # Set the duty cycle of the PWM signal
        self.pwm.ChangeDutyCycle(duty_cycle)

    def spin_clockwise(self):
        # zz tune ensure stopped/direction command
        # self.stop()

        gpio.output(self.in_1, gpio.HIGH)
        gpio.output(self.in_2, gpio.LOW)

    def spin_counter_clockwise(self):
        # zz tune ensure stopped/direction command
        # self.stop()

        gpio.output(self.in_1, gpio.LOW)
        gpio.output(self.in_2, gpio.HIGH)

    def stop(self):
        # Stop the motor
        self.set_speed(0)

    # close all inputs
    # might need a 10k ohm resistor for pwm
    def close(self):
        self.pwm.stop()
        gpio.cleanup()


# left_motor = Motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN1_PIN, LEFT_MOTOR_IN2_PIN)
