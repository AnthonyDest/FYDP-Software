from helper import check_simulate
from gpiozero import RotaryEncoder

try:
    import RPi.GPIO as gpio
except ImportError:
    gpio = None
    print("RPi.gpio not available. gpio functionality will be disabled.")


class encoder_driver:

    # TODO incorporate max steps and steps/rotation into parameters for steering vs drive
    def __init__(self, pin_A, pin_B, simulate=False):
        self.pin_A = pin_A
        self.pin_B = pin_B

        self.simulate = simulate

        if gpio is None:
            print("GPIO Disabled")
            return None
        self.init_pins()

    @check_simulate
    def init_pins(self):
        # TODO update max steps
        # zz max steps in this RotaryEncoder are for Value and when the encoder is active, it doesnt care that much about # of rotations (we need to do ourselves)
        # zz ^ it may be useful for steering, but a later problem
        # zz documentation shows that when max_steps = 0 has: CW +=1, CCW -=1

        self.encoder = RotaryEncoder(self.pin_A, self.pin_B, max_steps=0)

        # TODO Update steps per rotation
        self.steps_per_rotation = 96
        # zz need either steps/angle or angle/rotation to calculate angle (do math)

    # for steering, when it hits the limit switch, it should home
    @check_simulate
    def home_left(self):
        self.encoder.steps = 0

    @check_simulate
    def home_right(self):
        self.max_steering_steps = self.encoder.steps

    @check_simulate
    def get_steps(self):
        return self.encoder.steps
