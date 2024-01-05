import image_processing
import path_planning
import robot_control

# Code is split into 4 modules:
# image_processing.py   - used for machine vision and localization
# path_planning.py      - used to determine path to take
# robot_control.py      - used to control robot hardware
# main.py               - used to merge all modules together


class Robot:
    # split up to test modules separately
    def __init__(self):
        self.image_processing = image_processing.image_processing()
        self.path_planning = path_planning.path_planning(rink_length=60, rink_width=40)
        self.robot_control = robot_control.robot_control()
        self.robot_control.initialize_modules(self.image_processing, self.path_planning)

    def clear_rink(self):
        pass

    # TODO state machine during runtime, facilitates what the robot should do
    def state_machine(self):
        self.robot_control.path_planning.create_rink_border()
        self.robot_control.path_planning.generate_path()
        self.robot_control.path_planning.plot_path()

        # TODO enter all states and how to handle here
        while True:
            return
        pass


# Accomodate robot turning on, how to enter script, necessary hardware... (https://raspberrypi-guide.github.io/programming/run-script-on-boot)
def turn_on_robot():
    # handle turn on state
    pass


# initalize all
if __name__ == "__main__":
    robot = Robot().state_machine()
