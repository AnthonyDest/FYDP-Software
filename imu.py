from helper import *
import adafruit_icm20x

# zz may need board instead of gpio library


class imu:

    def __init__(self, SDA_pin=2, SCL_pin=3, name="No_name_imu", simulate=False):

        self.SDA_PIN = SDA_pin
        self.SCL_PIN = SCL_pin
        self.name = name

        if self.simulate:
            print(f"{self.name} is Simulated")
        self.init_pins()

    @check_simulate
    def init_pins(self):

        # configure pins
        gpio.setmode(gpio.BCM)

        # Initialize I2C
        gpio.setup(self.SDA_PIN, gpio.IN)
        gpio.setup(self.SCL_PIN, gpio.IN)

        self.contact_state = gpio.input(self.contact_pin)

        # ICM20948 initialization
        i2c = adafruit_icm20x.I2CDevice(self.SCL_PIN, self.SDA_PIN)
        self.icm = adafruit_icm20x.ICM20948(i2c)

    @check_simulate
    def print_all(self):
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (self.icm.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (self.icm.gyro))
        print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (self.icm.magnetic))
        print("")

    @check_simulate
    def calibrate_heading_zero(self):
        # Read the IMU data to get the current heading
        current_heading = self.icm.gyro

        # Set the current heading as the new reference (0 heading), zz may need to change if not starting going right
        self.heading_offset = current_heading

    @check_simulate
    def get_heading(self):
        # Read the raw heading from the IMU
        raw_heading = self.icm.gyro

        # Apply the offset to get the adjusted heading
        adjusted_heading = raw_heading - self.heading_offset

        # Ensure the adjusted heading is within the range [0, 360)
        adjusted_heading %= 360

        # zz convert to radians of [-pi, pi]
        adjusted_heading = np.radians(adjusted_heading)

        return adjusted_heading
