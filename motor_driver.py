from helper import check_simulate

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


# TODO add name parameter and property to all object classes (to identify L/R/Steering Motor)
class Motor:
    def __init__(self, pwm_pin, in_1_pin, in_2_pin, simulate=False):
        self.pwm_pin = pwm_pin
        self.in_1 = in_1_pin
        self.in_2 = in_2_pin
        self.simulate = simulate

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

    @check_simulate
    def set_speed(self, duty_cycle):
        # limit duty cycle

        # set pins based on duty cycle
        # "Forward"
        if duty_cycle > 0:
            self.spin_clockwise()
        elif duty_cycle < 0:
            self.spin_counter_clockwise()

        duty_cycle = abs(duty_cycle)

        if duty_cycle < 0:
            duty_cycle = 0
        elif duty_cycle > 100:
            duty_cycle = 100

        # Set the duty cycle of the PWM signal
        self.pwm.ChangeDutyCycle(duty_cycle)

    @check_simulate
    def spin_clockwise(self):
        # zz tune ensure stopped/direction command
        # self.stop()

        gpio.output(self.in_1, gpio.HIGH)
        gpio.output(self.in_2, gpio.LOW)

    @check_simulate
    def spin_counter_clockwise(self):
        # zz tune ensure stopped/direction command
        # self.stop()

        gpio.output(self.in_1, gpio.LOW)
        gpio.output(self.in_2, gpio.HIGH)

    @check_simulate
    def stop(self):
        # Stop the motor
        self.set_speed(0)

    # close all inputs
    # might need a 10k ohm resistor for pwm
    @check_simulate
    def close(self):
        self.pwm.stop()
        gpio.cleanup()


# left_motor = Motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_IN1_PIN, LEFT_MOTOR_IN2_PIN)
