import numpy as np
import helper
import matplotlib.pyplot as plt


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
        self.path = []

        # zz first generate x-y positions of all nodes
        self.path = self.generate_path_x_y()
        # zz second

        # TODO create a map of the rink, with the path overlaid
        self.map = None

        return self.path

    # TODO plot outline of rink path
    def generate_path_x_y(self):
        path = []

        x_itr = 0
        y_itr = 0
        dir_toggle = 1

        while y_itr <= self.rink_width:
            x_itr += 1 * dir_toggle

            path.append((x_itr, y_itr))

            if x_itr in (self.rink_length, 0):
                y_itr += 5
                dir_toggle *= -1
                path.append((x_itr, y_itr))

        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], marker="o", linestyle="-", label="Path")
        plt.title("Path with Square Wave Pattern")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.legend()
        plt.grid(True)
        plt.show()

        self.path = path

    def generate_path_water_rate(self):
        pass

    # TODO use matplotlib, plot the path
    def plot_path(self):
        # Extract x and y coordinates from the Node instances
        # x_coords = [node.x_coord for node in self.path]
        # y_coords = [node.y_coord for node in self.path]

        # Plot the path
        # plt.plot(x_coords, y_coords, marker="o", linestyle="-")
        plt.title("Path of the Robot")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.grid(True)

        plt.plot(
            self.x_rink_coords,
            self.y_rink_coords,
            marker="o",
            linestyle="-",
            label="Rectangle",
        )

        plt.show()

        pass

    # TODO use matplotlib, plot robot trajectory on path. Use map and path (truncate passed nodes?)
    def plot_robot(self):
        pass
