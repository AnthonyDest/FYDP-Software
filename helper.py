import math
import time
import matplotlib.pyplot as plt
import numpy as np


# contains information relating to robot current or desired kinematic state
# [x_coord, y_coord, heading, velocity]
class Node:
    # zz consider desired vs actual, or just use robot state
    def __init__(self, x_coord, y_coord, velocity=0, lap_number=0):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.velocity = velocity
        self.step_number = 0
        self.water_flow_rate = 0

        self.x_range = self.state_tolerance_range(self.x_coord)
        self.y_range = self.state_tolerance_range(self.y_coord)

        # zz consider adding a time stamp
        # zz consider other properties: flowrate, "state" (regular flood, refilling, repositioning...), etc

    def node_print(self):  # print everything
        print(
            f"Node: x_coord={self.x_coord}, y_coord={self.y_coord}, heading={self.heading}, velocity={self.velocity}, lap_number={self.lap_number}"
        )

    # TODO accept float tolerance, hyperparameter
    def state_tolerance_range(self, desired_coords):
        tolerance = 1
        tolerance_range = [desired_coords - tolerance, desired_coords + tolerance]
        return tolerance_range


class Path:
    def __init__(self):
        self.nodes = []
        self.current_step_number = 0
        self.path_length = 0

    def add_node(self, node):
        self.nodes.append(node)
        self.path_length += 1

    def print_path(self):
        for node in self.nodes:
            node.node_print()

    def check_if_position_in_path(self, position: Node):
        for node in self.nodes:
            if position == node.x_coord and position == node.y_coord:
                return True
        return False

    def get_x_y_positions(self):
        self.x_y_pair = []
        for node in self.nodes:
            self.x_y_pair.append([node.x_coord, node.y_coord])

        return self.x_y_pair


# zz may need speed improvements
def check_nodes_in_path(sublist: Path, parent_list: Path):
    sublist_pairs = nested_list_to_set(sublist.get_x_y_positions())
    parent_list_pairs = nested_list_to_set(parent_list.get_x_y_positions())

    return sublist_pairs.issubset(parent_list_pairs)


def nested_list_to_set(nested_list):
    return set(map(tuple, nested_list))


class timer:
    def __init__(self):
        self.start_time = self.get_current_time()
        self.delta_time = 0

    # get current time in seconds
    def get_current_time(self):
        return time.time()

    # get change in time in seconds
    def get_delta_time(self):
        self.delta_time = self.get_current_time() - self.start_time
        # print(f"start_time={self.start_time}, delta_time={self.delta_time}")
        self.start_time = self.get_current_time()

        return self.delta_time


class heading:
    def __init__(self):
        self.desired_heading = 0
        self.current_heading = 0

        self.desired_steering_angle = 0
        self.current_steering_angle = 0

    # TODO get current heading
    def get_current_heading(self):
        pass

    # TODO get desired heading
    def get_desired_heading(self):
        pass

    def get_x_component(self, heading, degrees=False):
        "gets angle in degrees, converts to rads, and returns multiplier"
        if degrees:
            heading = math.radians(heading)
        return math.cos(heading)

    def get_y_component(self, heading, degrees=False):
        "gets angle in degrees, converts to rads, and returns multiplier"

        if degrees:
            heading = math.radians(heading)
        return math.sin(heading)

    def print_heading(self):
        # print(
        #     f"To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), Current Coord: ({self.current_position_node.x_coord}, {self.current_position_node.y_coord}), Desired Heading: {self.desired_heading}, Current heading: {self.current_position_node.heading}, Required Steering Angle: {self.required_steering_angle}, Current Steering Angle: {self.steering_angle}"
        # )
        pass


def check_simulate(func):
    def wrapper(self, *args, **kwargs):
        # print(f"Simulate: {self.simulate}")
        if self.simulate:
            return False
        else:
            # Call the original function
            return func(self, *args, **kwargs)

    return wrapper
