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
        self.lap_number = lap_number

        # zz consider adding a time stamp
        # zz consider other properties: flowrate, "state" (regular flood, refilling, repositioning...), etc

    def node_print(self):  # print everything
        print(
            f"Node: x_coord={self.x_coord}, y_coord={self.y_coord}, heading={self.heading}, velocity={self.velocity}, lap_number={self.lap_number}"
        )
