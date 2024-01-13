import numpy as np
import helper
import matplotlib.pyplot as plt
import copy


class path_planning:
    def __init__(self, rink_length, rink_width):
        self.rink_length = rink_length
        self.rink_width = rink_width
        # hyperparameters:
        self.turn_radius_m = 1
        self.path_width_m = 1
        self.path_overlap_m = 0.2
        self.normal_velocity_straight_mps = 1
        self.normal_velocity_turning_mps = 1
        self.path = []

    # zz perhaps input rink dimensions here
    # zz how to deal with refilling water: does generate path determine at init or insert a go to the end of rink and back when detecting water full
    # zz how to specify water refill location. Refill at the end of the rink behind robot's start direction?
    # zz GUI, laptop or bluetooth app?
    # zz current assumptions: laptop GUI or hardcoded dimensions. Water refill at end of rink behind robot's start direction, stops if water isnt enough to complete next full lap + 1 lap tolerance

    def create_rink_border(self):
        # Define the vertices of the rectangle
        self.rink_corners = [
            (0, 0),
            (self.rink_length, 0),
            (self.rink_length, self.rink_width),
            (0, self.rink_width),
            (0, 0),  # Close the rectangle
        ]

        # Plot the rink
        self.x_rink_coords, self.y_rink_coords = zip(*self.rink_corners)

    # TODO generate a path of nodes to follow
    def generate_path(self):
        # zz first generate x-y positions of all nodes
        # self.path = self.generate_path_x_y()\
        self.generate_path_x_y()
        # zz second

        # TODO create a map of the rink, with the path overlaid
        self.map = None

        return self.path

    # TODO improve path planning algorithm (better pattern, refilling, water level)
    def generate_path_x_y(self):
        path = []

        x_itr = 0
        y_itr = 0
        dir_toggle = 1

        while not set(self.rink_corners).issubset(set(path)):
            path.append((x_itr, y_itr))

            if (
                x_itr == self.rink_length or (x_itr == 0 and not y_itr == 0)
            ) and y_itr < self.rink_width:
                y_itr += 10
                dir_toggle *= -1
                path.append((x_itr, y_itr))

                # zz need to test with the robot control code to see if more or less nodes better, if less nodes:
                # if both path lookback (idx of -1 and -2) have the same coord, do not include the current coord

            x_itr += 5 * dir_toggle

        path = np.array(path)
        self.path = copy.deepcopy(path)

        return self.path

    # TODO analyze the path node layout, determine what the optimal waterlevel at each node should be
    def generate_path_water_rate(self):
        pass

    # TODO use matplotlib, plot the path
    def plot_path(self, show_rink=False):
        plt.plot(
            self.path[:, 0], self.path[:, 1], marker="o", linestyle="-", label="Path"
        )
        plt.title("Path with Square Wave Pattern")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.legend()
        plt.grid(True)

        if show_rink:
            self.plot_rink_border()
        else:
            plt.show()

    def plot_rink_border(self):
        # Plot the path
        plt.title("Path of the Robot")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.grid(True)

        plt.plot(
            self.x_rink_coords,
            self.y_rink_coords,
            marker="o",
            linestyle="-",
            label="Rink Border",
        )

        plt.show()

        pass

    # TODO use matplotlib, plot robot trajectory on path. Use map and path (truncate passed nodes?)
    def plot_robot(self):
        pass
