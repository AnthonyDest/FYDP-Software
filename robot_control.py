from simple_pid import PID
import helper
import image_processing
import path_planning
import motor_driver


class robot_control:
    def __init__(self):
        # Initialize any variables or resources here
        self.current_position_node = helper.Node(0, 0, 0, 0, 0)
        self.desired_node = helper.Node(0, 0, 0, 0, 0)
        self.current_step_number = 0

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

    # TODO check x and y coords of current node and desired node, if within tolerance, return true
    def is_robot_near_desired_node(self):
        if (
            self.current_position_node.x_coord in self.desired_node.x_range
            and self.current_position_node.y_coord in self.desired_node.y_range
        ):
            return True
        return False

    def update_next_node(self):
        self.current_step_number += 1
        self.desired_node = self.path_planning.path.nodes[self.current_step_number]

    def plot_robot_position(self):
        self.path_planning.plot_robot(self.current_position_node, show_rink=True)

    # TODO init PID
    def init_PID(self):
        self.steering_pid = PID(1, 0.1, 0.05, setpoint=1)

    # TODO steer robot to desired heading
    def steer_robot(self):
        self.get_steering_velocity()
        pass

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
