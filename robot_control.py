import math
from time import sleep
import time
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

        self.max_speed_mps = 1  # rads # zz make hyperparameter # zz depreciated
        self.desired_drive_velocity = (
            0  # meters per second, zz change to PWM? # zz depreciated
        )
        self.current_drive_velocity = 0  # meters per second # zz depreciated
        self.max_drive_pwm = 90  # zz make hyperparameter
        self.desired_drive_pwm = 0
        self.current_drive_pwm = 0

        self.time_at_last_update = 0
        self.timer = helper.timer()
        self.heading = helper.heading()
        self.steering_motor = None
        self.left_motor = None
        self.right_motor = None
        self._internal_drive_pwm = 40
        self._encoder_steering = False
        self.last_if_execution_time = 0

    # zz depreciated
    def initialize_modules_pass_objs(self, image_processing, path_planning):
        self.image_processing = image_processing
        self.path_planning = path_planning

    def initialize_modules(self, simulate_all=False):
        self.initialize_image_processing()
        self.initialize_path_planning()
        self.initialize_hardware(simulate_all)

        # # zz speedup path for testing
        # self.path_planning.path.nodes = self.path_planning.path.nodes[10:]
        # print(
        #     f" X: {self.path_planning.path.nodes[0].x_coord}, Y: {self.path_planning.path.nodes[0].y_coord}"
        # )
        # # self.current_position_node.x_coord = self.path_planning.path.nodes[0].x_coord

    def initialize_image_processing(self):
        self.image_processing = image_processing.image_processing()
        self.pylon_processor = image_processing.pylon_processing()

    # make hyperparameter
    def initialize_path_planning(self):
        self.path_planning = path_planning.path_planning(rink_length=60, rink_width=40)

    def initialize_hardware(self, simulate_all=False):
        # zz temp config section
        # zz make hyperparameter, maybe make simulate object and clean this up
        # make below TRUE if disabled / simulated during regular run time
        simulate_left_limit_switch = True
        simulate_right_limit_switch = True
        simulate_steering_motor = False
        simulate_left_motor = False
        simulate_right_motor = False
        simulate_steering_motor_encoder = True
        simulate_left_motor_encoder = True
        simulate_right_motor_encoder = True
        simulate_valve = True

        LEFT_LIMIT_SWITCH_PIN = 22
        RIGHT_LIMIT_SWITCH_PIN = 27

        STEERING_MOTOR_PWM_PIN = 25  # goes to enable
        STEERING_MOTOR_IN1_PIN = 24
        STEERING_MOTOR_IN2_PIN = 23

        LEFT_MOTOR_PWM_PIN = 21  # goes to enable
        LEFT_MOTOR_IN1_PIN = 16
        LEFT_MOTOR_IN2_PIN = 20

        RIGHT_MOTOR_PWM_PIN = 8  # goes to enable
        RIGHT_MOTOR_IN1_PIN = 12
        RIGHT_MOTOR_IN2_PIN = 7

        STEERING_MOTOR_ENCODER_PIN_A = 19
        STEERING_MOTOR_ENCODER_PIN_B = 26
        LEFT_MOTOR_ENCODER_PIN_A = 10
        LEFT_MOTOR_ENCODER_PIN_B = 9
        RIGHT_MOTOR_ENCODER_PIN_A = 6
        RIGHT_MOTOR_ENCODER_PIN_B = 13

        VALVE_IN1_PIN = 23
        VALVE_IN2_PIN = 24

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
            simulate_valve = True

        # TODO make all hyperparameter

        # initialize limit switch

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

        # initialize motors

        self.steering_motor = motor_driver.Steering_Motor(
            pwm_pin=STEERING_MOTOR_PWM_PIN,
            in_1_pin=STEERING_MOTOR_IN1_PIN,
            in_2_pin=STEERING_MOTOR_IN2_PIN,
            name="steering_motor",
            simulate=simulate_steering_motor,
        )

        self.left_motor = motor_driver.Motor_Driver(
            pwm_pin=LEFT_MOTOR_PWM_PIN,
            in_1_pin=LEFT_MOTOR_IN1_PIN,
            in_2_pin=LEFT_MOTOR_IN2_PIN,
            name="left_motor",
            simulate=simulate_left_motor,
        )

        self.right_motor = motor_driver.Motor_Driver(
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
            simulate=True,
            center_angle_rad=self.steering_lock_angle_rad,
        )

        self.left_motor_encoder = encoder.encoder(
            LEFT_MOTOR_ENCODER_PIN_A,
            LEFT_MOTOR_ENCODER_PIN_B,
            name="left_motor_encoder",
            simulate=True,
        )

        self.right_motor_encoder = encoder.encoder(
            RIGHT_MOTOR_ENCODER_PIN_A,
            RIGHT_MOTOR_ENCODER_PIN_B,
            name="right_motor_encoder",
            simulate=True,
        )

        self.valve = motor_driver.Valve(
            in_1_pin=VALVE_IN1_PIN,
            in_2_pin=VALVE_IN2_PIN,
            name="valve",
            simulate=simulate_valve,
        )

        # TODO get Kp, Ki, Kd values from tuning
        self.init_steering_PID()

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
        self.valve.close()

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
    # zz call motor PID control here to improve polling?
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

    def plot_robot_position_init(self):
        self.path_planning.plot_robot_init(self.current_position_node, show_rink=True)

    def plot_robot_position(self, printout=False):
        self.path_planning.plot_robot(self.current_position_node)
        if printout:
            print(
                f"To Coord: ({self.desired_node.x_coord}, {self.desired_node.y_coord}), C.Coord: ({self.current_position_node.x_coord:.2f}, {self.current_position_node.y_coord:.2f}), D.Heading: {self.heading.desired_heading:.2f}, C.Heading: {self.heading.current_heading:.2f}, D.Steering Angle: {self.heading.desired_steering_angle:.2f}, C.Steering Angle: {self.heading.current_steering_angle:.2f}, D.Speed: {self.desired_drive_velocity:.2f}, C.Speed: {self.current_drive_velocity:.2f}"
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
        # self.execute_steering()

        # zz execute drive commands to motor hardware
        # self.execute_velocity()
        self.execute_drive()

        # get distance from encoder steps, avg L & R (ignore wheel-slip/turning)
        # TODO add a button for telop operation to begin turning
        left_distance = self.left_motor_encoder.get_distance_m_since_last_call()
        right_distance = self.right_motor_encoder.get_distance_m_since_last_call()
        if left_distance == False:
            # if left_distance is simulated, assume its a ratio of drive PWM
            left_distance = self.current_drive_pwm / 100
        if right_distance == False:
            # if right_distance is simulated, assume its a ratio of drive PWM
            right_distance = self.current_drive_pwm / 100
        distance = (left_distance + right_distance) / 2
        # print(f"Distance: {distance}")

        # heading orientation overflow
        zz_was_caught = False
        self.heading.current_heading += self.heading.current_steering_angle
        if abs(self.heading.current_heading) > np.pi:
            zz_was_caught = True
            self.heading.current_heading -= (
                np.sign(self.heading.current_heading) * 2 * np.pi
            )

            # self.heading.current_heading %= 2 * np.pi
            # self.heading.current_heading %= 2 * np.pi
            # if self.heading.current_heading > np.pi:
            #     self.heading.current_heading -= 2 * np.pi
            # elif self.heading.current_heading < -np.pi:
            #     self.heading.current_heading += 2 * np.pi
            zz_was_caught = True

        # if abs(self.heading.current_heading) > 20:
        #     print(
        #         f"ERROR: heading overflow: {self.heading.current_heading} --- IS LARGER DETECTED {abs(self.heading.current_heading) > np.pi}"
        #     )

        x_dist = self.heading.get_x_component(self.heading.current_heading) * distance
        y_dist = self.heading.get_y_component(self.heading.current_heading) * distance
        self.current_position_node.x_coord += x_dist
        self.current_position_node.y_coord += y_dist

    # TODO init PID
    def init_steering_PID(
        self, Kp=250, Ki=2, Kd=150, setpoint=0, output_limits=(-100, 100)
    ):
        self._steering_pid_controller = PID(
            Kp=Kp, Ki=Ki, Kd=Kd, setpoint=setpoint, output_limits=output_limits
        )

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
            # self.read_arrow_keys()
            self.full_teleop_keyboard()

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
            self.desired_drive_pwm = 80  # TODO update to get from the current node info
        else:
            # if teleop, use arrow keys to control velocity
            self.read_arrow_keys()  # zz maybe split reading into fwd/back and left/right

        # max speed PWM
        if abs(self.desired_drive_pwm) > self.max_drive_pwm:
            self.desired_drive_pwm = (
                np.sign(self.desired_drive_pwm) * self.max_drive_pwm
            )

    def speed_robot_old(self, teleop_enable=False):
        """OLD
        Determines how fast the robot should be driving, either from path planning or teleop
        """
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

    # zz unnecessary abstraction
    def execute_drive(self):
        "Execute drive commands to motor hardware, make current = desired"

        # calls a linear ramp on drive PWM
        self.drive_pwm(self.desired_drive_pwm)

    # TODO execute steering angle based on desired
    def execute_steering(self):
        """
        Steering angle input

        Simulate steering encoder:

        current_heading += steering_ROC * time (assumed to be 1 for now)
        """

        self.steer_PID_rad(self.heading.desired_steering_angle)
        # zz check steer if broken encoders

        # TODO have better steering corrections (steer back on path)
        # Preventing Oversteer
        if self.left_limit_switch.is_pressed():
            # self.steering_motor.set_speed(10)
            self.steering_motor.set_speed(0)
        elif self.right_limit_switch.is_pressed():
            # self.steering_motor.set_speed(-10)
            self.steering_motor.set_speed(0)

    # zz depreciated
    # def update_current_steering_angle(self):

    #     if self.steering_motor_encoder.simulate:
    #         # zz slightly redundant
    #         # zz awk, kinda simulates the delta that goes into PID, improve
    #         if (
    #             self.heading.desired_steering_angle
    #             > self.heading.current_steering_angle
    #         ):
    #             self.heading.current_steering_angle += self.steering_lock_angle_rad / 3
    #         elif (
    #             self.heading.desired_steering_angle
    #             < self.heading.current_steering_angle
    #         ):
    #             self.heading.current_steering_angle -= self.steering_lock_angle_rad / 3
    #         else:
    #             self.heading.current_steering_angle = 0

    #         # return self.heading.current_steering_angle
    #     else:
    #         self.heading.current_steering_angle = (
    #             self.steering_motor_encoder.get_steering_angle_rad()
    #         )
    #         # return self.steering_motor_encoder.get_steering_angle_rad()

    # TODO check if this function works in polling, or if it needs to be threaded
    def steer_PID_deg(self, desired_angle):
        print(f"deg: {desired_angle}, rad: {math.radians(desired_angle)}")
        self.steer_PID_rad(math.radians(desired_angle))

    # TODO check if this function works in polling, or if it needs to be threaded
    def steer_PID_rad(self, desired_angle):
        """Receive: desired angle of steering rack in radians
        if real: send motors corresponding PID PWM value, receive encoder values to update current heading
        if simulate, receive current steering += PID_output/<scaling> to steer"""

        current_angle = self.heading.current_steering_angle
        self._steering_pid_controller.setpoint = desired_angle
        pid_out = self._steering_pid_controller(current_angle)

        self.steering_motor.set_speed(pid_out)
        # TODO tune PID_output speed PWM value to motor turning angle for sim
        # TODO check if this updates fast enough
        # print(f"current_angle: {current_angle}, pid_out: {pid_out}")

        if self.steering_motor_encoder.simulate:

            # if pid_out != 0:
            # print(f"PID: {pid_out}")

            # print(f"PID: {pid_out}")
            simulated_steering_angle_change = pid_out * (
                self.steering_lock_angle_rad / (20 * 100)
            )  # 2 * 100 is [-100 -> 100] scaling -lock/2 to lock/2, will need to reduce if not using full PWM range
            self.heading.current_steering_angle += simulated_steering_angle_change
            # print(f"Steering Angle Change: {simulated_steering_angle_change}")

            delta_angle = desired_angle - current_angle
            if delta_angle > 0:
                pass
                # self.heading.current_steering_angle += 0.2
            # elif delta_angle < 0:
            #     self.heading.current_steering_angle -= 0.2

        else:
            self.heading.current_steering_angle = (
                self.steering_motor_encoder.get_steering_angle_rad()
            )

    # zz check desired velocity
    # zz both motors should be going the same way bc rotated 180deg & opposite side of gear
    def drive_pwm(self, desired_pwm):
        left_pwm = self.left_motor.linear_ramp_speed(desired_pwm)
        right_pwm = self.right_motor.linear_ramp_speed(desired_pwm)

        #  if left_pwm == False:
        #     # if left_distance is simulated, assume its a ratio of drive PWM
        #     left_distance = self.current_drive_pwm / 100
        # if right_distance == False:
        #     # if right_distance is simulated, assume its a ratio of drive PWM
        #     right_distance = self.current_drive_pwm / 100

        # zz this may not work if we need to customize each motor PWM. May need maps
        self.current_drive_pwm = np.average([left_pwm, right_pwm])

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
                self.desired_drive_pwm = 80

            elif down_pressed:
                print("Down arrow key pressed")
                self.desired_drive_pwm = -80
            else:
                self.desired_drive_pwm = 0

            if left_pressed:
                print("Left arrow key pressed")
                self.heading.desired_steering_angle = +self.steering_lock_angle_rad / 5

            elif right_pressed:
                print("Right arrow key pressed")
                self.heading.desired_steering_angle = -self.steering_lock_angle_rad / 5
            else:
                self.heading.desired_steering_angle = 0

        except Exception as e:
            print(f"An error occurred: {e}")

    def _testing_read_arrow_keys(self):
        try:
            # Check for each arrow key independently
            up_pressed = keyboard.is_pressed("up")
            down_pressed = keyboard.is_pressed("down")
            left_pressed = keyboard.is_pressed("left")
            right_pressed = keyboard.is_pressed("right")

            # for steering
            if left_pressed:
                print("Left arrow key pressed")
                self.heading.desired_steering_angle = +self.steering_lock_angle_rad / 5

            elif right_pressed:
                print("Right arrow key pressed")
                self.heading.desired_steering_angle = -self.steering_lock_angle_rad / 5
            else:
                self.heading.desired_steering_angle = 0

            # for testing

            s_pressed = keyboard.is_pressed("s")
            if s_pressed:
                print("S key pressed")

            q_pressed = keyboard.is_pressed("q")
            if q_pressed:
                print("Q key pressed")

            return s_pressed, q_pressed

        except Exception as e:
            print(f"An error occurred: {e}")

    def full_teleop_keyboard(self):
        """
        - Arrow keys drive robot
        - Q is close graph/exit (not needed?)
        - F is speed up by 20% pwm, G is slow down by 20% pwm
        - V is valve open, B is valve close
        - E is set steering left position, R is set steering right position
        """
        try:
            # Check for each arrow key independently
            drive_fwd = keyboard.is_pressed("up")
            drive_bwd = keyboard.is_pressed("down")
            # steer_left = keyboard.is_pressed("left")
            # steer_right = keyboard.is_pressed("right")

            steer_left = keyboard.is_pressed("n")
            steer_right = keyboard.is_pressed("m")
            slow_steer_left = keyboard.is_pressed("left")
            slow_steer_right = keyboard.is_pressed("right")

            toggle_steering = keyboard.is_pressed("t")

            step_steer_left = keyboard.is_pressed("j")
            step_steer_right = keyboard.is_pressed("k")
            # slow_steer_left = keyboard.is_pressed("n")
            # slow_steer_right = keyboard.is_pressed("m")
            fast_steer_left = keyboard.is_pressed("u")
            fast_steer_right = keyboard.is_pressed("i")

            close_control = keyboard.is_pressed("q")
            speed_up = keyboard.is_pressed("f")
            slow_down = keyboard.is_pressed("g")
            open_valve = keyboard.is_pressed("v")
            close_valve = keyboard.is_pressed("b")
            set_steering_left = keyboard.is_pressed("e")
            set_steering_right = keyboard.is_pressed("r")

            # for drive and steering
            if drive_fwd:
                self.desired_drive_pwm = self._internal_drive_pwm
            elif drive_bwd:
                self.desired_drive_pwm = -self._internal_drive_pwm
            else:
                self.desired_drive_pwm = 0

            if speed_up:
                self._internal_drive_pwm += 20
                self._internal_drive_pwm = min(self._internal_drive_pwm, 100)
                print(f"Increased speed, currently: {self._internal_drive_pwm}")
                sleep(0.5)
            elif slow_down:
                self._internal_drive_pwm -= 20
                self._internal_drive_pwm = max(self._internal_drive_pwm, 0)
                print(f"Decreased speed, currently: {self._internal_drive_pwm}")
                sleep(0.5)

            # zz check + vs - for steering
            if toggle_steering:
                self._encoder_steering = not self._encoder_steering
                print(f"-------Encoder steering: {self._encoder_steering} -------")

            if self._encoder_steering:
                if steer_left:
                    self.heading.desired_steering_angle = (
                        +self.steering_lock_angle_rad / 50
                    )
                    # self.heading.desired_steering_angle = self.steering_lock_angle_rad / 5
                elif steer_right:
                    self.heading.desired_steering_angle = (
                        -self.steering_lock_angle_rad / 50
                    )
                    # self.heading.desired_steering_angle = -self.steering_lock_angle_rad / 5
                else:
                    self.heading.desired_steering_angle = 0
            else:
                if step_steer_left:
                    self.steering_motor.set_speed(80)
                    sleep(0.3)
                elif step_steer_right:
                    self.steering_motor.set_speed(-70)
                    sleep(0.3)
                elif slow_steer_left:
                    self.steering_motor.set_speed(70)
                elif slow_steer_right:
                    self.steering_motor.set_speed(-60)
                elif fast_steer_left:
                    self.steering_motor.set_speed(60)
                elif fast_steer_right:
                    self.steering_motor.set_speed(-60)
                else:
                    self.steering_motor.set_speed(0)

            # homing steering (right is 0, left is max)
            # zz if calling redo right, might need to offset the left motor max
            if set_steering_right:
                self.steering_motor_encoder.home_right()
                print(f"Right Homed at {self.steering_motor_encoder.get_steps()} steps")
            elif set_steering_left:
                self.steering_motor_encoder.home_left()
                print(f"Left Homed at {self.steering_motor_encoder.get_steps()} steps")

            # for valve, zz will this keep valve open if not held
            if open_valve:
                self.valve.open_valve()
                print("Valve Opened")
            elif close_valve:
                self.valve.close_valve()
                print("Valve Closed")

            if close_control:
                # return False
                self.path_planning.stop_interactive_plot()
                self.close_modules()
                return False

            return True

        except Exception as e:
            print(f"An error occurred: {e}")
            return False

    def steer_to_pylon(self, show_frame=False):

        distance_from_center = self.pylon_processor.process_pylon(show_frame=show_frame)
        # distance_from_center = pylon_processor.process_pylon("image", "pylon_right.jpg")
        # distance_from_center = pylon_processor.process_pylon(
        #     "image", "pylon_center.jpg"
        # )
        current_time = time.time()

        if current_time - self.last_if_execution_time >= 0: # zz disabled
            self.last_if_execution_time = current_time
            # Left positive, right negative
            if distance_from_center < 0:
                print("Steer right")
                self.steering_motor.set_speed(-20)
                self.heading.desired_steering_angle = self.steering_lock_angle_rad / 5
            elif distance_from_center > 0:
                print("Steer left")
                self.heading.desired_steering_angle = self.steering_lock_angle_rad / 5
                self.steering_motor.set_speed(20)
            elif distance_from_center == 0:
                print("Center")
                self.heading.desired_steering_angle = 0.0
                self.steering_motor.set_speed(0)
            else:
                print("Error")

        # pylon_processor.process_pylon("video", "")
