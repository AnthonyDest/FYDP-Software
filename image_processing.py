import helper


class image_processing:
    # initalize all default values
    def __init__(self):
        self.pose_location = None
        self.vision_mask_location = None

    # TODO get pose from april tag
    def update_pose_location(self):
        self.pose_location = helper.Node(0, 0, 0, 0)
        pass

    # TODO determines how L/R the robot is from the desired overlap of the water
    # TODO gives location x +=/-=, and y +=/-= of actual position to correct steering
    # zz assumption is that the previous path is always more correct than current sensor input
    def update_vision_mask_location(self):
        self.vision_mask_location = helper.Node(0, 0, 0, 0)
        pass
