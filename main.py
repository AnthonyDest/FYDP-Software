import image_processing
import path_planning
import robot_control

# Code is split into 4 modules:
# image_processing.py   - used for machine vision and localization
# path_planning.py      - used to determine path to take
# robot_control.py      - used to control robot hardware
# main.py               - used to merge all modules together

class Robot:

    # split up to test modules seperately
    def __init__(self):
        self.image_processing = image_processing.ImageProcessing()
        self.path_planning = path_planning.PathPlanning()
        self.robot_control = robot_control.RobotControl()

        self.robot_control.initalize_modules(self.image_processing,self.path_planning)


    def clear_rink(self):
        pass

    # TODO state machine during runtime, facilitates what the robot should do
    def state_machine():
        # TODO enter all states and how to handle here
        while (True):
            pass
        pass


# Accomodate robot turning on, how to enter script, necessary hardware... (https://raspberrypi-guide.github.io/programming/run-script-on-boot)
def turn_on_robot():
    # handle turn on state
    pass


# initalize all 
if __name__ == "__main__":
    robot = Robot().state_machine()


