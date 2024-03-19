from helper import *


# TODO add name parameter and property to all object classes (to identify L/R switches)
class limit_switch:
    "1 is pressed/removed, 0 is good"
    def __init__(self, contact_pin, name="No_name_limit_switch", simulate=False):
        self.contact_pin = contact_pin
        self.simulate = simulate
        self.name = name
        self.DEBOUNCE_TIME_MS = 200

        if self.simulate:
            print(f"{self.name} is Simulated")
        self.init_pins()

    @check_simulate
    def init_pins(self):

        # configure pins
        gpio.setmode(gpio.BCM)
        gpio.setup(self.contact_pin, gpio.IN, pull_up_down=gpio.PUD_UP)

        self.contact_state = gpio.input(self.contact_pin)

    @check_simulate
    def is_pressed(self):
        """
        Input pins: 1 is pressed/removed, 0 is good
        """
        self.contact_state = not gpio.input(self.contact_pin)
        return self.contact_state

    @check_simulate
    def print_contact(self):

        # TODO replace print pin with name
        self.is_pressed()
        print(f"Limit switch {self.name} contact status: {self.contact_state}")

    @check_simulate
    def print_contact_if_pressed(self):
        # TODO replace print pin with name
        if self.is_pressed():
            print(f"Limit switch {self.name} contact status: {self.contact_state}")

