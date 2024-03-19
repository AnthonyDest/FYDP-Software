from enum import Enum, unique, Flag, auto


class state(Enum):
    initialization = auto()
    follow_path = auto()
    wait_for_refill = auto()
    travel_to_refill = auto()
    travel_to_path = auto()
    end = auto()
    pylon = auto()
    teleop = auto()
    tune_steering_pid = auto()
    test = auto()
    