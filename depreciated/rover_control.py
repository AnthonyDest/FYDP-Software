from motor_driver import left_motor, right_motor, steering_motor

class Rover:
    def __init__(self):

        self._left_motor = left_motor
        self._right_motor = right_motor
        self._steering_motor = steering_motor
        self._speed = 0
        self._heading = 0 # Following atan2 rules
        # Right: 0, Up: pi/2, Left: pi, Down: -pi/2

        # OLD
        # self._heading = 0 # Viewed topdown: CW +, CCW -
        # N: 0, E: 90, S: 180, W: 270

## How the rover should be controlled
# Path is provided in terms of vectors at points
# Rover internally modifies heading and distance to match accordingly
# Make sure high level path planning is reasonable to simplify rover following






# Highest to lowest level
        
- two nodes, go from one to the next

- Steering loop and driving loop

- Steering loop: PID on steering correction, constanyl refreshing current to desired heading to get steering angle required
- Distance loop: PID on speed correction


-"Driving": 1. Ignore overshoot, just pick a speed and drive until the distance is met (track distance of each wheel? How to deal with turns? Math?)
            2. Have a gentle ramp up speeds to get to desired speed. OK to overshoot distance
            3. Have gentle ramp down transition speed to get to desired speed, ok to overshoot distance
            4. Have a gentle ramp up and ramp down to stop at end distance if node = stop node


ASSUMPTION: Robot will still work if assumption not met, but ideally have coords spaced reasonably for rover to to follow path easily
