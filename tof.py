from helper import *

import busio

import adafruit_vl53l0x

# Initialize I2C bus and sensor.


# TODO make this a parent class with children steering and drive encoders
class TOF:

    # TODO incorporate max steps and steps/rotation into parameters for steering vs drive
    def __init__(
        self, name="No_name_TOF", threshold=800, debounce_time_s=0.25, simulate=False
    ):

        self.name = name
        self.simulate = simulate
        self.threshold = threshold

        self.debounce_time_s = debounce_time_s
        self.last_detection_time = time.monotonic()
        self.last_distance = 0

        if self.simulate:
            print(f"{self.name} is Simulated")
        self.init_i2c()

    @check_simulate
    def init_i2c(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.vl53 = adafruit_vl53l0x.VL53L0X(self.i2c)

    @check_simulate
    def get_distance(self):
        return self.vl53.range

    # zz debouncing code if needed
    # @check_simulate
    # def get_filtered_distance(self):
    #     current_time = time.monotonic()
    #     distance = self.get_distance()

    #     if (current_time - self.last_reading_time) >= self.debounce_time:
    #         if distance is not None: # zz check under 2000
    #             self.last_distance = distance
    #         self.last_reading_time = current_time

    #     return self.last_distance

    @check_simulate
    def is_object_detected(self):

        # current_time = time.monotonic()  # Get current time
        # if current_time - self.last_detection_time < self.debounce_time_s:
        #     return False
        distance = self.get_distance()
        if distance < self.threshold:
            # print(distance)
            print(f"OBJECT DETECTED AT {distance}")
            return True
        return False
