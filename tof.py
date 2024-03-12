from helper import *


# TODO make this a parent class with children steering and drive encoders
class TOF:

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
        pass
