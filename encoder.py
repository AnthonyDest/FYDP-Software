from helper import check_simulate
from gpiozero import RotaryEncoder

try:
    import RPi.GPIO as gpio
except ImportError:
    gpio = None
    print("RPi.gpio not available. gpio functionality will be disabled.")


class encoder:

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
    # zz add a tolerance? eg, instead of set to 0, set to -TOL. And max is actually max - TOL
    # this way if you always operate between 0 and max, you will have a slight gap
    # also needs a, if ever hit limit switch, rehome?
    @check_simulate
    def home_left(self):
        self.encoder.steps = 0
        print(f"Left Encoder Homed, Steps: {self.encoder.steps}")

    @check_simulate
    def home_right(self):
        self.max_steering_steps = self.encoder.steps
        print(f"Right Encoder Homed, Steps: {self.encoder.steps}")

    @check_simulate
    def get_steps(self):
        print(f"Encoder Steps: {self.encoder.steps}")
        return self.encoder.steps
