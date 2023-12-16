import helper

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
    # TODO generate a path of nodes to follow
    def generate_path(self):
        self.path = []
        
        # TODO create a mao of the rink, with the path overlaid
        self.map = None

        return self.path

    # TODO use matplotlib, plot the path
    def plot_path(self):
        pass

    # TODO use matplotlib, plot robot trajectory on path. Use map and path (truncate passed nodes?)
    def plot_robot(self):
        pass

