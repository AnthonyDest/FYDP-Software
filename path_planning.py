import math
import helper
import matplotlib.pyplot as plt


class path_planning:
    def __init__(self, rink_length, rink_width):
        self.rink_length = rink_length
        self.rink_width = rink_width
        # hyperparameters:
        self.max_steering_lock_angle_rad = 1
        self.path_width_m = 1
        self.path_overlap_m = 0.2
        self.normal_velocity_straight_mps = 1
        self.normal_velocity_turning_mps = 1
        # self.path = []
        self.path = helper.Path()
        self.rink_corners = None

    # zz perhaps input rink dimensions here
    # zz how to deal with refilling water: does generate path determine at init or insert a go to the end of rink and back when detecting water full
    # zz how to specify water refill location. Refill at the end of the rink behind robot's start direction?
    # zz GUI, laptop or bluetooth app?
    # zz current assumptions: laptop GUI or hardcoded dimensions. Water refill at end of rink behind robot's start direction, stops if water isnt enough to complete next full lap + 1 lap tolerance

    def create_rink_border(self):
        # Define the vertices of the rectangle
        self.rink_corners = helper.Path()
        self.rink_corners.add_node(helper.Node(0, 0))
        self.rink_corners.add_node(helper.Node(self.rink_length, 0))
        self.rink_corners.add_node(helper.Node(self.rink_length, self.rink_width))
        self.rink_corners.add_node(helper.Node(0, self.rink_width))
        # for visual purposes, close the rink
        self.rink_corners.add_node(helper.Node(0, 0))

        # Plot the rink
        self.x_rink_coords, self.y_rink_coords = zip(
            *self.rink_corners.get_x_y_positions()
        )

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
        x_itr = 0
        y_itr = 0
        dir_toggle = 1

        while not helper.check_nodes_in_path(self.rink_corners, self.path):
            # path.append((x_itr, y_itr))
            self.path.add_node(helper.Node(x_itr, y_itr, 0, 0))

            if (
                x_itr == self.rink_length or (x_itr == 0 and not y_itr == 0)
            ) and y_itr < self.rink_width:
                y_itr += 10
                dir_toggle *= -1
                self.path.add_node(helper.Node(x_itr, y_itr, 0, 0))

                # zz need to test with the robot control code to see if more or less nodes better, if less nodes:
                # if both path lookback (idx of -1 and -2) have the same coord, do not include the current coord

            x_itr += 5 * dir_toggle

        self.add_lookahead_radius_to_path(self.max_steering_lock_angle_rad)
        self.label_path_steps()

    def add_lookahead_radius_to_path(self, max_radius_radians):
        heading_simulator = helper.heading()

        for idx, node in enumerate(self.path.nodes[:-1]):
            (
                desired_heading,
                steering_angle,
            ) = self.get_desired_heading_steering_between_nodes(
                self.path.nodes[idx + 1], node, heading_simulator.current_heading
            )

            # if turn is too sharp, make it wider
            # if abs(steering_angle) > max_radius_radians:

            # if self.get_desired_heading_steering_between_nodes(
            #     self.path.nodes[idx + 1], node
            # ) >
            # pass

        pass

    def label_path_steps(self):
        for idx, node in enumerate(self.path.nodes):
            node.step_number = idx

    def get_desired_heading_steering_between_nodes(
        self, desired_node, current_node, current_heading
    ):
        delta_y = desired_node.y_coord - current_node.y_coord
        delta_x = desired_node.x_coord - current_node.x_coord

        # ensure no divide by 0 error
        if delta_x == 0:
            delta_x = 1

        desired_heading = math.atan2(delta_y, delta_x)

        # if current_heading is None:
        #     return desired_heading

        update_heading = desired_heading - current_heading

        return desired_heading, update_heading

    # TODO analyze the path node layout, determine what the optimal waterlevel at each node should be
    def generate_path_water_rate(self):
        pass

    # TODO use matplotlib, plot the path
    def plot_path(self, show_rink=False):
        if show_rink:
            self.plot_rink_border(wait=True)

        x_coords, y_coords = zip(*self.path.get_x_y_positions())

        plt.plot(x_coords, y_coords, marker="o", linestyle="-", label="Path")
        plt.title("Path with Square Wave Pattern")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.legend()
        plt.grid(True)

        # ax = plt.gca()
        # set xlimit of plot

        plt.xlim([-10, self.rink_length + 10])
        plt.ylim([-10, self.rink_width + 10])

        plt.show()

    def plot_rink_border(self, wait=False):
        # Plot the path
        plt.title("Rink Border")
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
        if not wait:
            plt.show()

    # TODO use matplotlib, plot robot trajectory on path. Use map and path (truncate passed nodes?)
    # currently just plot current location
    def plot_robot(self, current_position_node, show_rink=False):
        existing_labels = [line.get_label() for line in plt.gca().get_children()]
        label_to_check = "Rink Border"

        if show_rink and not label_to_check in existing_labels:
            self.plot_rink_border(wait=True)

        x_coords, y_coords = (
            current_position_node.x_coord,
            current_position_node.y_coord,
        )

        plt.plot(
            x_coords,
            y_coords,
            marker="o",
            color="red",
            zorder=10,
        )
        plt.title("Robot Current Position")
        plt.xlabel("X-coordinate")
        plt.ylabel("Y-coordinate")
        plt.legend()
        plt.grid(True)
        plt.xlim([-10, self.rink_length + 10])
        plt.ylim([-10, self.rink_width + 10])

        plt.ion()

        plt.pause(0.001)

    def stop_interactive_plot(self):
        plt.ioff()
        plt.show()
