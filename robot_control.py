import helper
import path_planning

class robot_control:
    def __init__(self):
        # Initialize any variables or resources here
        self.robot_node = helper.Node(0, 0, 0, 0)

    def initalize_modules(self, image_processing, path_planning):
        self.image_processing = image_processing
        self.path_planning = path_planning

    # TODO drive to a node, calculate relative coords
    def drive_to_node(self, node: helper.Node):
        # Code to drive the robot to a node
        pass

    # TODO read in all sensor data
    def read_in_all_sensor_data(self):
        pass

    # TODO merge all sensor data into robot node / more useable values
    def merge_sensor_data(self):
        pass

    # TODO steer robot to desired heading
    def steer_robot(self):
        pass
    
    # TODO all low level drive commands, when function recieves relative coords
