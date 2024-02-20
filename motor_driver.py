from helper import *


class Motor:
    def __init__(
        self,
        pwm_pin,
        in_1_pin,
        in_2_pin,
        name="No_name_motor",
        simulate=False,
        lowest_pwm=20,
        pwm_step=10,
    ):
        self.pwm_pin = pwm_pin
        self.in_1 = in_1_pin
        self.in_2 = in_2_pin
        self.name = name
        self.simulate = simulate
        self.speed = 0  # zz depreciated?
        self.lowest_pwm = lowest_pwm
        self.pwm_step = pwm_step
        self.current_pwm = 0

        if self.simulate:
            print(f"{self.name} is Simulated")
        # configure pins
        self.init_pins()

    @check_simulate
    def init_pins(self):
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

    def linear_ramp_speed(self, desired_pwm):

        delta_pwm = desired_pwm - self.current_pwm

        new_pwm = 0

        if abs(delta_pwm) < self.pwm_step:

            new_pwm = desired_pwm
        else:
            new_pwm = self.current_pwm + self.pwm_step * np.sign(delta_pwm)

        self.set_speed(new_pwm)
        return new_pwm

    @check_simulate
    def set_speed(self, duty_cycle):
        "CW is +, CCW is -, 0 is stop, 100 is max speed, -100 is max reverse speed, 50 is half speed, etc."
        # limit duty cycle0
        self.current_pwm = duty_cycle
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
