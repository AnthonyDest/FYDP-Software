try:
    import RPi.GPIO as gpio
except ImportError:
    gpio = None
    print("RPi.GPIO not available. GPIO functionality will be disabled.")

# TODO integrate better
def check_hardware_OK():
    if gpio is None:
        print("GPIO Disabled")
        return False
    return True

# TODO add name parameter and property to all object classes (to identify L/R switches)
class limit_switch:
    def __init__(self, contact_pin):
        self.contact_pin = contact_pin

        if gpio is None:
            print("GPIO Disabled")
            return None

        # configure pins
        gpio.setmode(gpio.BCM)
        gpio.setup(self.contact_pin, gpio.IN)

        self.contact_state = gpio.input(self.contact_pin)

    def is_pressed(self):
        self.contact_state = gpio.input(self.contact_pin)
        return self.contact_state

    def print_contact(self):

        # TODO replace print pin with name
        self.is_pressed()
        print(f"Pin {self.contact_pin} contact status: {self.contact_state}")
