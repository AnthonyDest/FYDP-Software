import image_processing
import path_planning
import robot_control
import states

# Code is split into 4 modules:
# image_processing.py   - used for machine vision and localization
# path_planning.py      - used to determine path to take
# robot_control.py      - used to control robot hardware
# main.py               - used to merge all modules together


class Robot:
    # split up to test modules separately
    # zz replace passing in objects with a parent class that has all the objects
    def __init__(self):
        self.robot_control = robot_control.robot_control()
        self.robot_control.initialize_modules()

    def clear_rink(self):
        pass

    # TODO state machine during runtime, facilitates what the robot should do
    def state_machine(self):
        self.robot_control.path_planning.create_rink_border()
        self.robot_control.path_planning.generate_path()
        self.robot_control.path_planning.plot_rink_border()
        self.robot_control.path_planning.plot_path(show_rink=True)
        self.robot_control.plot_robot_position()
        # self.robot_control.steer_robot()

        state = states.state

        current_state = state.follow_path
        self.robot_control.reset_timer()

        zzEscape = 0
        # TODO enter all states and how to handle here
        while True:
            # zz update all sensor data
            # encoder, steering velocity, motor velocity, IMU positioning...
            self.robot_control.read_in_all_sensor_data()

            match current_state:
                case state.initialization:
                    pass

                case state.follow_path:
                    # iterate through each node in list

                    # calculate relative change between current status node and desired node
                    # call steer and drive parameters to move to next node

                    # check if at next node +- tolerance, if so update next node
                    near_node = self.robot_control.is_robot_near_desired_node()
                    if near_node:
                        more_stops_after_next = self.robot_control.update_next_node()

                        # if end case, stop
                        if not more_stops_after_next:
                            current_state = state.end
                            continue

                    # # zz manual moving for simulation
                    # self.robot_control.current_position_node = (
                    #     self.robot_control.desired_node
                    # )

                    # steer and drive will act as an if statement for control, part of parent loop of state machine
                    self.robot_control.drive_path(simulate_feedback=True)

                    # plot robot current position
                    self.robot_control.plot_robot_position()

                    # if node.type = travel, switch to travel_to_refill

                    pass
                case state.wait_for_refill:
                    pass
                case state.travel_to_refill:
                    pass
                case state.travel_to_path:
                    pass

                case state.end:
                    print("Travel done")

                    # wait for user to finish analyzing plots
                    self.robot_control.path_planning.stop_interactive_plot()
                    # plt.ioff
                    return None

                case _:
                    pass

            # print(f"Current State: {current_state}")
            zzEscape += 1
            if zzEscape > 2000:
                print("zzEscape")
                return None


# accommodate robot turning on, how to enter script, necessary hardware... (https://raspberrypi-guide.github.io/programming/run-script-on-boot)
def turn_on_robot():
    # handle turn on state
    pass


# initialize all
if __name__ == "__main__":
    robot = Robot()
    robot.state_machine()
