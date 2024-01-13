from motor_driver import left_motor, right_motor, steering_motor

class Rover:
    def __init__(self):

        self._left_motor = left_motor
        self._right_motor = right_motor
        self._steering_motor = steering_motor
        self._speed = 0
        self._heading = 0 # Viewed topdown: CW +, CCW -

## How the rover should be controlled
# Path is provided in terms of vectors at points
# Rover internally modifies heading and distance to match accordingly
# Make sure high level path planning is reasonable to simplify rover following
