import math
import keyboard
import numpy as np
from simple_pid import PID
import encoder
import helper
import image_processing
import path_planning
import motor_driver
import limit_switch
import logging


class robot_control:
    def __init__(self):
        # Initialize any variables or resources here
        self.current_position_node = helper.Node(0, 0, 0)
        self.desired_node = helper.Node(0, 0, 0)
        self.current_step_number = 0
        # self.steering_velocity = 0  # angle per second
        self.steering_lock_angle_rad = math.radians(30)  # rads # zz make hyperparameter
        self.max_speed_mps = 1  # rads # zz make hyperparameter
        self.desired_drive_velocity = 0  # meters per second
        self.current_drive_velocity = 0  # meters per second
        self.time_at_last_update = 0
        self.timer = helper.timer()
        self.heading = helper.heading()
        self.steering_motor = None
        self.left_motor = None
        self.right_motor = None

    # zz depreciated
    def initialize_modules_pass_objs(self, image_processing, path_planning):
        self.image_processing = image_processing
        self.path_planning = path_planning

    def initialize_modules(self, simulate_all=False):
        self.initialize_image_processing()
        self.initialize_path_planning()
        self.initialize_hardware(simulate_all)

    def initialize_image_processing(self):
        self.image_processing = image_processing.image_processing()

    # make hyperparameter
    def initialize_path_planning(self):
        self.path_planning = path_planning.path_planning(rink_length=60, rink_width=40)

    def initialize_hardware(self, simulate_all=False):
        # zz temp config section
        # zz make hyperparameter, maybe make simulate object and clean this up
        # make below TRUE if disabled / simulated during regular run time
        simulate_left_limit_switch = False
        simulate_right_limit_switch = False
        simulate_steering_motor = False
        simulate_left_motor = False
        simulate_right_motor = False
        simulate_steering_motor_encoder = False
        simulate_left_motor_encoder = False
        simulate_right_motor_encoder = False

        LEFT_LIMIT_SWITCH_PIN = 22
        RIGHT_LIMIT_SWITCH_PIN = 27

        STEERING_MOTOR_PWM_PIN = 8  # goes to enable
        STEERING_MOTOR_IN1_PIN = 7
        STEERING_MOTOR_IN2_PIN = 12

        LEFT_MOTOR_PWM_PIN = 18  # goes to enable
        LEFT_MOTOR_IN1_PIN = 17
        LEFT_MOTOR_IN2_PIN = 19

        RIGHT_MOTOR_PWM_PIN = 19  # goes to enable
        RIGHT_MOTOR_IN1_PIN = 23
        RIGHT_MOTOR_IN2_PIN = 24

        STEERING_MOTOR_ENCODER_PIN_A = 25
        STEERING_MOTOR_ENCODER_PIN_B = 26
        LEFT_MOTOR_ENCODER_PIN_A = 5
        LEFT_MOTOR_ENCODER_PIN_B = 6
        RIGHT_MOTOR_ENCODER_PIN_A = 13
        RIGHT_MOTOR_ENCODER_PIN_B = 16

        # All below should not be modified (out of temp config file)

        # auto simulate all if rpi.gpio is not available
        if not simulate_all:
            simulate_all = not helper.is_hardware_OK()

        if simulate_all:
            simulate_left_limit_switch = True
            simulate_right_limit_switch = True
            simulate_steering_motor = True
            simulate_left_motor = True
            simulate_right_motor = True
            simulate_steering_motor_encoder = True
            simulate_left_motor_encoder = True
            simulate_right_motor_encoder = True

        # TODO make all hyperparameter

        # initalize limit switch

        self.left_limit_switch = limit_switch.limit_switch(
            LEFT_LIMIT_SWITCH_PIN,
            name="left_limit_switch",
            simulate=simulate_left_limit_switch,
        )
        self.right_limit_switch = limit_switch.limit_switch(
            RIGHT_LIMIT_SWITCH_PIN,
            name="right_limit_switch",
            simulate=simulate_right_limit_switch,
        )

        # initalize motors

        self.steering_motor = motor_driver.Motor(
            pwm_pin=STEERING_MOTOR_PWM_PIN,
            in_1_pin=STEERING_MOTOR_IN1_PIN,
            in_2_pin=STEERING_MOTOR_IN2_PIN,
            name="steering_motor",
            simulate=simulate_steering_motor,
        )

        self.left_motor = motor_driver.Motor(
            pwm_pin=LEFT_MOTOR_PWM_PIN,
            in_1_pin=LEFT_MOTOR_IN1_PIN,
            in_2_pin=LEFT_MOTOR_IN2_PIN,
            name="left_motor",
            simulate=simulate_left_motor,
        )

        self.right_motor = motor_driver.Motor(
            pwm_pin=RIGHT_MOTOR_PWM_PIN,
            in_1_pin=RIGHT_MOTOR_IN1_PIN,
            in_2_pin=RIGHT_MOTOR_IN2_PIN,
            name="right_motor",
            simulate=simulate_right_motor,
        )

        self.steering_motor.speed = 0
        self.left_motor.speed = 0
        self.right_motor.speed = 0

        # initialize encoders
        self.steering_motor_encoder = encoder.encoder(
            STEERING_MOTOR_ENCODER_PIN_A,
            STEERING_MOTOR_ENCODER_PIN_B,
            name="steering_motor_encoder",
            simulate=simulate_steering_motor_encoder,
            center_angle_rad=self.steering_lock_angle_rad,
        )

        self.left_motor_encoder = encoder.encoder(
            LEFT_MOTOR_ENCODER_PIN_A,
            LEFT_MOTOR_ENCODER_PIN_B,
            name="left_motor_encoder",
            simulate=simulate_left_motor_encoder,
        )

        self.right_motor_encoder = encoder.encoder(
            RIGHT_MOTOR_ENCODER_PIN_A,
            RIGHT_MOTOR_ENCODER_PIN_B,
            name="right_motor_encoder",
            simulate=simulate_right_motor_encoder,
        )

        # TODO get Kp, Ki, Kd values from tuning
        self._steering_pid_controller = PID(
            Kp=1, Ki=0, Kd=0, setpoint=0, output_limits=(-100, 100)
        )

        # check hardware status
        # hardware_OK = motor_driver.check_hardware_OK()
        # if not hardware_OK:
        #     print("Hardware not OK")
        # return None

    def close_modules(self):
        self.steering_motor.close()
        self.left_motor.close()
        self.right_motor.close()
        self.left_limit_switch.close()
        self.right_limit_switch.close()
        self.steering_motor_encoder.close()
        self.left_motor_encoder.close()
        self.right_motor_encoder.close()

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
        # zz perhaps minor performance improvements here
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
        print(
            f"To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), C.Coord: ({self.current_position_node.x_coord:.2f}, {self.current_position_node.y_coord:.2f}), D.Heading: {self.heading.desired_heading:.2f}, C.Heading: {self.heading.current_heading:.2f}, R.Steering Angle: {self.heading.desired_steering_angle:.2f}, C.Steering Angle: {self.heading.current_steering_angle:.2f}, D.Speed: {self.desired_drive_velocity:.2f}, C.Speed: {self.current_drive_velocity:.2f}"
        )

    def reset_timer(self):
        self.time_at_last_update = self.timer.get_current_time()

    def drive_path(self):
        'has PID "loops" for steering and drive motors. also has the option to simulate motor feedback'

        # controls steering, either auto path follow to next node, or teleop
        # if driving path, it will never be teleop
        self.steer_robot(teleop_enable=False)
        self.speed_robot(teleop_enable=False)

        # sets current = desired (both steering a)
        self.execute_desired()

    def execute_desired(self):
        """Drive: Receives velocity and steering angle, updates position based on velocity and heading
        If simulate = True, provide a velocity of 1"""

        # zz execute steering commands to motor hardware
        self.execute_steering()

        # zz execute drive commands to motor hardware
        # self.execute_velocity()
        self.execute_drive()

        # TODO use velocity x timestep to calculate distance based on encoder values
        distance = self.current_drive_velocity / 2

        # use velocity * change in elapsed time to calculate distance moved
        # TODO replace velocity with a function that uses encoders and pose to calculate velocity
        # zz_temp_new_velocity = 1
        # if self.drive_velocity != zz_temp_new_velocity:
        #     self.drive_velocity = zz_temp_new_velocity
        time = self.timer.get_delta_time()
        # print(time)
        # zz time removed debugging
        # distance = self.drive_velocity * time

        # Driving Simulation (zz updating current node status)

        # heading orientation overflow
        self.heading.current_heading += self.heading.current_steering_angle
        if abs(self.heading.current_heading) > np.pi:
            self.heading.current_heading -= (
                np.sign(self.heading.current_heading) * 2 * np.pi
            )

        x_dist = self.heading.get_x_component(self.heading.current_heading) * distance
        y_dist = self.heading.get_y_component(self.heading.current_heading) * distance
        self.current_position_node.x_coord += x_dist
        self.current_position_node.y_coord += y_dist

        # zz temp disable printout coords
        # print(
        #     f"To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), C.Coord: ({self.current_position_node.x_coord:.2f}, {self.current_position_node.y_coord:.2f}), D.Heading: {self.heading.desired_heading:.2f}, C.Heading: {self.heading.current_heading:.2f}, R.Steering Angle: {self.heading.desired_steering_angle:.2f}, C.Steering Angle: {self.heading.current_steering_angle:.2f}, D.Speed: {self.desired_drive_velocity:.2f}, C.Speed: {self.current_drive_velocity:.2f}"
        # )

    # TODO init PID
    def init_PID(self):
        self.steering_pid = PID(1, 0.1, 0.05, setpoint=1)

    # TODO steer robot to desired heading
    def steer_robot(self, teleop_enable=False):
        """Determines required coords between current node and next, updates desired movements accordingly
        Teleop_enable: if true, allows for manual control of robot via arrow keys"""
        # self.get_steering_velocity()

        # Path VS Teleop
        # determine desired heading between current position and desired node
        if not teleop_enable:
            (
                self.heading.desired_heading,
                self.heading.desired_steering_angle,
            ) = self.path_planning.get_desired_heading_steering_between_nodes(
                self.desired_node,
                self.current_position_node,
                self.heading.current_heading,
            )
        # else desired input is called via teleop
        else:
            self.read_arrow_keys()

        # once you get desired steering angle, verify its within range

        # steering overflow
        if abs(self.heading.desired_steering_angle) > np.pi:
            self.heading.desired_steering_angle -= (
                np.sign(self.heading.desired_steering_angle) * 2 * np.pi
            )

        # max steering lock
        if abs(self.heading.desired_steering_angle) > self.steering_lock_angle_rad:
            self.heading.desired_steering_angle = (
                np.sign(self.heading.desired_steering_angle)
                * self.steering_lock_angle_rad
            )

        ## zz perhaps add if steering limit switch is hit, correct steering here

    def speed_robot(self, teleop_enable=False):
        """Determines how fast the robot should be driving, either from path planning or teleop"""
        if not teleop_enable:
            self.desired_drive_velocity = 1
        else:
            # if teleop, use arrow keys to control velocity
            self.read_arrow_keys()  # zz maybe split reading into fwd/back and left/right

        # max speed
        if abs(self.desired_drive_velocity) > self.max_speed_mps:
            self.desired_drive_velocity = (
                np.sign(self.desired_drive_velocity) * self.max_speed_mps
            )

    # TODO determine how to get water level. Sensor, or integral of time and flow rate?
    def get_water_level(self):
        self.current_water_level = 0
        return self.current_water_level

    # TODO improve sequence/set_speed fxn if desired = current
    # zz modify the speeds to bump, make a bump function?
    # TODO make a quick home function if a limit switch is bumped while driving? Only modifies the edge hit
    def home_steering(self):

        homing_speed = 30  # TODO Update / use PID for slow homing

        # zz check first that all necessary hardware is active:
        if (
            self.steering_motor.simulate
            or self.steering_motor_encoder.simulate
            or self.left_limit_switch.simulate
            or self.right_limit_switch.simulate
        ):
            print("Cannot home steering, not all hardware is active")
            return

        # if steering is at limit, bump it left
        while self.right_limit_switch.is_pressed():
            self.steering_motor.set_speed(-homing_speed)

        # stop motor
        self.steering_motor.set_speed(0)

        # zz should wait a few second, until we have better PID/motor inertia handling for motor firmware
        self.timer.wait_seconds(2)

        self.steering_motor.set_speed(homing_speed)

        # bump steering right till limit switch is pressed
        while not self.right_limit_switch():
            # wait...
            pass

        # stop motor
        self.steering_motor.set_speed(0)

        # set steering encoder right home here
        self.steering_motor_encoder.home_right()

        # zz should wait a few second, until we have better PID/motor inertia handling for motor firmware
        self.timer.wait_seconds(2)

        self.steering_motor.set_speed(-homing_speed)

        # bump steering left till limit switch is pressed
        while not self.left_limit_switch():
            # wait...
            pass

        self.steering_motor.set_speed(0)

        # set steering encoder left home here
        self.steering_motor_encoder.home_left()

        # update steering encoder with angles
        self.steering_motor_encoder.update_steering_angle_per_step(
            self.steering_lock_angle_rad
        )

        # zz should wait a few second, then center the steering motor
        # zz do we even care about this? or is it more efficient to just start path planning from here?
        self.steering_motor.set_speed(homing_speed)
        while self.steering_motor_encoder.get_steering_angle_rad() > 0:
            # wait...
            pass
        self.steering_motor.set_speed(0)
        self.timer.wait_seconds(2)

    # TODO all low level drive commands, when function receives relative coords between two nodes

    # TODO get velocity from merge sensor data, primarily use encoders and pose?
    # zz remove simulate in param
    def execute_drive(self):

        speed_step = self.max_speed_mps / 10

        # TODO verify reverse
        # zz temp velocity is slow stepping
        if self.desired_drive_velocity > self.current_drive_velocity:
            self.current_drive_velocity += speed_step
        elif self.desired_drive_velocity < self.current_drive_velocity:
            self.current_drive_velocity -= speed_step

        # TODO Modify simulate/real so that only simulate has distance = velocity
        # get simulate from motor directly

        # TODO replace with PID
        """
        Velocity ~= PWM input (temp zz)
        map 0 - max_speed as PWM: 0-100, same for negatives's
        send that input
        """

        # speed to send to motors:
        pwm_value = (self.current_drive_velocity / self.max_speed_mps) * 100

        # TODO enable drive motors when connected
        self.left_motor.set_speed(pwm_value)
        # self.right_motor.set_speed(pwm_value)

        # self.left_motor.set_speed(100)

    # TODO execute steering angle based on desired
    def execute_steering(self):

        # TODO Modify simulate/real so that only simulate has distance = velocity
        # if simulate:
        #     pass

        # else:  # real

        # TODO replace with PID (No PID, very aggressive turning)
        """
        Steering angle input

        Simulate steering encoder:

        current_heading += steering_ROC * time (assumed to be 1 for now)
        """

        # Have desired steering angle and current steering angle
        # self.heading.current_steering_angle = (
        #     self.steering_motor_encoder.get_steering_angle_rad()
        # )

        self.update_current_steering_angle()


        self.steer_PID_rad(
            self.heading.desired_steering_angle, self.heading.current_steering_angle
        )

        # TODO have better steering corrections (steer back on path)
        # Preventing Oversteer
        if self.left_limit_switch.is_pressed():
            # self.steering_motor.set_speed(10)
            self.steering_motor.set_speed(0)
        elif self.right_limit_switch.is_pressed():
            # self.steering_motor.set_speed(-10)
            self.steering_motor.set_speed(0)

    def update_current_steering_angle(self):

        if self.steering_motor_encoder.simulate:
            # zz slightly redundant
            # zz awk, kinda simulates the delta that goes into PID, improve
            if (
                self.heading.desired_steering_angle
                > self.heading.current_steering_angle
            ):
                self.heading.current_steering_angle += self.steering_lock_angle_rad / 3
            elif (
                self.heading.desired_steering_angle
                < self.heading.current_steering_angle
            ):
                self.heading.current_steering_angle -= self.steering_lock_angle_rad / 3
            else:
                self.heading.current_steering_angle = 0

            # return self.heading.current_steering_angle
        else:
            self.heading.current_steering_angle = (
                self.steering_motor_encoder.get_steering_angle_rad()
            )
            # return self.steering_motor_encoder.get_steering_angle_rad()

    def steer_PID_rad(self, desired_angle, current_angle):
        delta_angle = desired_angle - current_angle
        # print(delta_angle)
        self.steering_motor.set_speed(delta_angle * 30)  # replace with PID

    # def steer_angle_rad(self, angle):
    #     self.heading.desired_steering_angle = angle

    # pass
    def steer_pwm(self, desired=None, current=None):

        # Use the provided values or use defaults if they are None
        desired = (
            desired if desired is not None else self.heading.desired_steering_angle
        )
        current = (
            current if current is not None else self.heading.current_steering_angle
        )

        # if desired is not None or current is not None:

        #     steering_input = self._steering_pid_controller(
        #         self.heading.current_steering_angle, self.heading.desired_steering_angle
        #     )
        # else:
        #     steering_input = self._steering_pid_controller(current, desired)
        steering_input = self._steering_pid_controller(current, desired)

        self.steering_motor.set_speed(steering_input)

    # zz check desired velocity
    def drive_pwm(self):
        pass

    # # TODO confirm conversion wraparound ok
    # def steer_angle_deg(self, angle):
    #     self.steer_angle_rad(math.radians(angle))


    # TODO get encoder values
    def get_encoder_values(self):
        pass

    def read_arrow_keys(self):
        try:
            # Check for each arrow key independently
            up_pressed = keyboard.is_pressed("up")
            down_pressed = keyboard.is_pressed("down")
            left_pressed = keyboard.is_pressed("left")
            right_pressed = keyboard.is_pressed("right")

            if up_pressed:
                print("Up arrow key pressed")
                # give forward velocity, L R + 50%
                # self.left_motor.speed = 50
                # self.right_motor.speed = 50
                self.desired_drive_velocity = 1

                # self.left_motor.set_speed(50)
                # self.right_motor.set_speed(50)

            elif down_pressed:
                print("Down arrow key pressed")
                # give backward velocity, L R - 50%
                # self.left_motor.speed = -50
                # self.right_motor.speed = -50
                self.desired_drive_velocity = -1
            else:
                self.desired_drive_velocity = 0

            if left_pressed:
                print("Left arrow key pressed")
                # give left velocity, L - 25%, R + 25%
                # self.left_motor.speed = 25
                # self.right_motor.speed = 75
                # self.heading.current_steering_angle = +self.steering_lock_angle_rad / 5
                self.heading.desired_steering_angle = +self.steering_lock_angle_rad / 5

            elif right_pressed:
                print("Right arrow key pressed")
                # give right velocity, L + 25%, R - 25%
                # self.left_motor.speed = 75
                # self.right_motor.speed = 25
                # self.heading.current_steering_angle = -self.steering_lock_angle_rad / 5
                self.heading.desired_steering_angle = -self.steering_lock_angle_rad / 5
            else:
                # self.heading.current_steering_angle = 0
                self.heading.desired_steering_angle = 0

        except Exception as e:
            print(f"An error occurred: {e}")
