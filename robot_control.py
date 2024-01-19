import math
import numpy as np
from simple_pid import PID
import helper
import image_processing
import path_planning
import motor_driver
import logging


class robot_control:
    def __init__(self):
        # Initialize any variables or resources here
        self.current_position_node = helper.Node(0, 0, 0)
        self.desired_node = helper.Node(0, 0, 0)
        self.current_step_number = 0
        self.steering_velocity = 0  # angle per second
        self.steering_lock_angle_rad = math.radians(30)  # rads
        self.drive_velocity = 0  # meters per second
        self.time_at_last_update = 0
        self.timer = helper.timer()
        self.heading = helper.heading()
        # self.desired_heading = 0
        # self.current_heading = 0

    # zz depreciated
    def initialize_modules_pass_objs(self, image_processing, path_planning):
        self.image_processing = image_processing
        self.path_planning = path_planning

    def initialize_modules(self):
        self.initalize_image_processing()
        self.initalize_path_planning()
        self.initalize_hardware()

    def initalize_image_processing(self):
        self.image_processing = image_processing.image_processing()

    # make hyperparameter
    def initalize_path_planning(self):
        self.path_planning = path_planning.path_planning(rink_length=60, rink_width=40)

    def initalize_hardware(self):
        # initalize motors

        # check hardware status
        hardware_OK = motor_driver.check_hardware_OK()
        if not hardware_OK:
            print("Hardware not OK")
            return None

        LEFT_MOTOR_PWM_PIN = 16  # goes to enable
        LEFT_MOTOR_IN1_PIN = 20
        LEFT_MOTOR_IN2_PIN = 21

        left_motor = motor_driver.Motor(
            pwm_pin=LEFT_MOTOR_PWM_PIN,
            in_1_pin=LEFT_MOTOR_IN1_PIN,
            in_2_pin=LEFT_MOTOR_IN2_PIN,
        )

    # TODO drive to a node, calculate relative coords
    def drive_to_node(self, node: helper.Node):
        # Code to drive the robot to a node

        # # TODO get relative coords between two nodes
        # relative_angle = current - desired
        # relative_distance = current - desired

        # self.drive_parameters(relative_angle, relative_distance)

        # Given two nodes (current and desired), calculate the relative angle and distance between them
        # Then

        pass

    # TODO read in all sensor data
    def read_in_all_sensor_data(self):
        pass

    # TODO merge all sensor data into robot node / more useable values
    def merge_sensor_data(self):
        pass

    def check_in_range(self, number_to_check, range):
        # get the max and min values of a range
        up_range = max(range)
        low_range = min(range)
        if low_range <= number_to_check <= up_range:
            return True

            # if range[0] <= number_to_check <= range[1]:
            return True
        return False

    # TODO check x and y coords of current node and desired node, if within tolerance, return true
    def is_robot_near_desired_node(self):
        if self.check_in_range(
            self.current_position_node.x_coord, self.desired_node.x_range
        ) and self.check_in_range(
            self.current_position_node.y_coord, self.desired_node.y_range
        ):
            # if (
            #     self.current_position_node.x_coord in self.desired_node.x_range
            #     and self.current_position_node.y_coord in self.desired_node.y_range
            # ):
            return True
        return False

    def update_next_node(self):
        self.current_step_number += 1
        if self.current_step_number >= self.path_planning.path.path_length:
            return False
        self.desired_node = self.path_planning.path.nodes[self.current_step_number]
        return True

    def plot_robot_position(self):
        self.path_planning.plot_robot(self.current_position_node, show_rink=True)

    def reset_timer(self):
        self.time_at_last_update = self.timer.get_current_time()

    def drive_path(self, simulate_feedback=False):
        'has PID "loops" for steering and drive motors. also has the option to simulate motor feedback'

        zz_temp_new_velocity = 1
        # zz_temp_new_heading = self.current_position_node.heading
        self.steer_robot()
        # self.steering_angle = math.atan2(0, 10)
        # print(self.steering_angle)
        # self.steering_angle = math.atan2(10, 0)
        # print(self.steering_angle)
        # self.steering_angle = math.atan2(0, -10)
        # print(self.steering_angle)
        # self.steering_angle = math.atan2(-10, 0)
        # print(self.steering_angle)

        # self.steering_angle = self.desired_steering_heading

        # we have  self.heading.current_steering_angle
        # need to figure out how to use relative steering angle and current heading to get new current heading

        if simulate_feedback:
            # use velocity * change in elapsed time to calculate distance moved
            # TODO replace velocity with a function that uses encoders and pose to calculate velocity

            if self.drive_velocity != zz_temp_new_velocity:
                self.drive_velocity = zz_temp_new_velocity
            time = self.timer.get_delta_time()
            # print(time)
            # zz time removed debugging
            # distance = self.drive_velocity * time

            # Driving Simulation
            distance = self.drive_velocity

            self.heading.current_heading += self.heading.current_steering_angle
            if abs(self.heading.current_heading) > np.pi:
                self.heading.current_heading -= (
                    np.sign(self.heading.current_heading) * 2 * np.pi
                )
                # self.heading.current_heading = np.mod(
                #     self.heading.current_heading + np.pi, np.pi
                # )
            # self.heading

            ###

            x_dist = (
                self.heading.get_x_component(self.heading.current_heading) * distance
            )
            y_dist = (
                self.heading.get_y_component(self.heading.current_heading) * distance
            )
            self.current_position_node.x_coord += x_dist
            self.current_position_node.y_coord += y_dist

            # Heading Simulation

            # heading.

            # logging.debug(f"x_dist={x_dist}, y_dist={y_dist}")
            # print(f"x_dist={x_dist}, y_dist={y_dist}")
            # print(
            #     f"Heading: {zz_temp_new_heading}, To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), Current Coord: ({self.current_position_node.x_coord}, {self.current_position_node.y_coord}), X_range = {self.desired_node.x_range}, Y_range = {self.desired_node.y_range}"
            # )
            print(
                f"To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), Current Coord: ({self.current_position_node.x_coord:.2f}, {self.current_position_node.y_coord:.2f}), Desired Heading: {self.heading.desired_heading:.2f}, Current heading: {self.heading.current_heading:.2f}, Required Steering Angle: {self.heading.desired_steering_angle:.2f}, Current Steering Angle: {self.heading.current_steering_angle:.2f}"
            )

    # TODO init PID
    def init_PID(self):
        self.steering_pid = PID(1, 0.1, 0.05, setpoint=1)

    # TODO steer robot to desired heading
    def steer_robot(self):
        # self.get_steering_velocity()
        delta_y = self.desired_node.y_coord - self.current_position_node.y_coord
        delta_x = self.desired_node.x_coord - self.current_position_node.x_coord

        if delta_x == 0:
            delta_x = 1

        # delta_x = min(delta_x, 0)  # ensure no divide by zero error
        # self.steering_angle = delta_y / delta_x
        self.heading.desired_heading = math.atan2(delta_y, delta_x)

        # determine if relative angle is larger than steering lock
        self.heading.desired_steering_angle = (
            self.heading.desired_heading - self.heading.current_heading
        )

        # zz steering overflow
        if abs(self.heading.desired_steering_angle) > np.pi:
            self.heading.desired_steering_angle -= (
                np.sign(self.heading.desired_steering_angle) * 2 * np.pi
            )

        # max steering lock
        if abs(self.heading.desired_steering_angle) > self.steering_lock_angle_rad:
            self.heading.current_steering_angle = (
                np.sign(self.heading.desired_steering_angle)
                * self.steering_lock_angle_rad
            )
        else:
            self.heading.current_steering_angle = self.heading.desired_steering_angle
            # self.heading.current_steering_angle = np.mod(
            #     self.heading.current_steering_angle + np.pi, np.pi
            # )

    # TODO determine how to get water level. Sensor, or integral of time and flow rate?
    def get_water_level(self):
        self.current_water_level = 0
        return self.current_water_level

    # TODO all low level drive commands, when function receives relative coords between two nodes

    # TODO get velocity from merge sensor data, primarily use encoders and pose?
    def get_velocity(self):
        pass

    # TODO get encoder values
    def get_encoder_values(self):
        pass
