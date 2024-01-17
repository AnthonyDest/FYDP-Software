import matplotlib.pyplot as plt


# contains information relating to robot current or desired kinematic state
# [x_coord, y_coord, heading, velocity]
class Node:
    # zz consider desired vs actual, or just use robot state
    def __init__(self, x_coord, y_coord, heading, velocity, lap_number):
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.heading = heading
        self.velocity = velocity
        self.lap_number = lap_number  # zz remove?
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

    def state_tolerance_range(self, desired_coords):
        tolerance = 5

        return range(desired_coords - tolerance, desired_coords + tolerance)
