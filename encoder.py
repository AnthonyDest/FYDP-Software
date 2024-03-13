from helper import *
from gpiozero import RotaryEncoder


# TODO make this a parent class with children steering and drive encoders
class encoder:

    # TODO incorporate max steps and steps/rotation into parameters for steering vs drive
    def __init__(
        self, pin_A, pin_B, name="No_name_encoder", simulate=False, center_angle_rad=0
    ):
        self.pin_A = pin_A
        self.pin_B = pin_B
        self.name = name
        self.center_angle_rad = center_angle_rad
        self.simulate = simulate
        self.prev_step_count = 0
        self.center_angle_rad = 0.1
        self.angle_rad_per_step = 0.1
        if self.simulate:
            print(f"{self.name} is Simulated")
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
        self.encoder.steps = self.encoder.steps
        print(f"Left Encoder Homed, Steps: {self.encoder.steps}")

    @check_simulate
    def home_right(self):
        self.encoder.steps = 0
        print(f"Right Encoder Homed, Steps: {self.encoder.steps}")

    @check_simulate
    def get_steps(self):
        print(f"Encoder Steps: {self.encoder.steps}")
        return self.encoder.steps

    @check_simulate
    # TODO change max_steering_angle_lock_rad based on what is easiest to measure
    def update_steering_angle_per_step(self, max_steering_angle_lock_rad):
        # TODO scale down max_steering_angle_lock_rad based on a tolerance (or just input with tolerance?)

        self.angle_rad_per_step = (
            max_steering_angle_lock_rad * 2 / self.max_steering_steps
        )

        self.center_angle_rad = max_steering_angle_lock_rad

    @check_simulate
    def get_steering_angle_rad(self):
        "left is positive, center is 0, right is negative"
        angle = self._get_steering_angle_rad()
        if angle is False:
            return self.center_angle_rad / 10
        return angle

    @check_simulate
    def _get_steering_angle_rad(self):
        angle = self.encoder.steps * self.angle_rad_per_step - self.center_angle_rad
        return angle

    @check_simulate
    def get_distance_m_since_last_call(self):

        delta_steps = self.encoder.steps - self.prev_step_count
        # zz hyperparameter
        distance_meter_per_step = 0.05  # TODO do experimental testing to do this (can also use wheel radius and steps/rotation)

        distance_meter = delta_steps * distance_meter_per_step

        self.prev_step_count = self.encoder.steps
        return distance_meter

