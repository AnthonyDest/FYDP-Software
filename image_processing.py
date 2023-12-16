class image_processing:
    def __init__(self):
        self.pose_location = None
        self.vision_mask_location = None

    def update_pose_location(self, new_location):
        self.pose_location = new_location

    def update_vision_mask_location(self, new_location):
        self.vision_mask_location = new_location
