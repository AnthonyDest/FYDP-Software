import sys
import argparse
from time import sleep

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
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
        self.robot_control.initialize_modules(simulate_all_arg)

    def clear_rink(self):
        pass

    # TODO state machine during runtime, facilitates what the robot should do
    def state_machine(self):
        try:
            self.robot_control.path_planning.create_rink_border()
            # TODO need to figure out current position on map if not at start node, and travel to start node
            self.robot_control.path_planning.generate_path()

            # self.robot_control.path_planning.plot_rink_border()
            # self.robot_control.path_planning.plot_path(show_rink=True)
            # self.robot_control.plot_robot_position(printout=printout_arg)
            # self.robot_control.steer_robot()

            if speedup_arg:
                self.speedup_start()

            state = states.state

            if teleop_enable_arg:
                current_state = state.teleop
                self.robot_control.plot_robot_position_init()
            elif tune_steering_arg:
                current_state = state.tune_steering_pid
            elif test_arg:
                current_state = state.test
            else:
                # self.robot_control.plot_robot_position(printout=printout_arg)

                current_state = state.initialization

            self.robot_control.reset_timer()

            zzEscape = 0
            # TODO enter all states and how to handle here
            while True:
                # zz update all sensor data
                # encoder, steering velocity, motor velocity, IMU positioning...
                self.robot_control.read_in_all_sensor_data()

                if current_state == state.initialization:
                    self.robot_control.home_steering()
                    current_state = state.follow_path

                elif current_state == state.follow_path:
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

                    # steer and drive will act as an if statement for control, part of parent loop of state machine
                    self.robot_control.drive_path()

                    # plot robot current position
                    self.robot_control.plot_robot_position(printout=printout_arg)

                    # if node.type = travel, switch to travel_to_refill
                    # pass

                elif current_state == state.wait_for_refill:
                    pass
                elif current_state == state.travel_to_refill:
                    pass
                elif current_state == state.travel_to_path:
                    pass
                elif current_state == state.manual:
                    # near_node = self.robot_control.is_robot_near_desired_node()

                    # self.robot_control.read_arrow_keys()
                    self.robot_control.steer_robot(teleop_enable=teleop_enable_arg)

                    self.robot_control.execute_desired()

                    self.robot_control.plot_robot_position(printout=printout_arg)

                elif current_state == state.tune_steering_pid:
                    """This is a temporary state to tune the steering PID controller."""
                    # self.robot_control.steer_robot(teleop_enable=True)

                    if zzEscape == 1:
                        self.robot_control.init_steering_PID(Kp=500, Ki=2, Kd=150)
                        self.robot_control.path_planning.stop_interactive_plot()
                        print("Tuning steering PID")
                        print(
                            f"Current steering PID: P: {self.robot_control._steering_pid_controller.Kp} I: {self.robot_control._steering_pid_controller.Ki} D: {self.robot_control._steering_pid_controller.Kd}"
                        )

                    s_pressed, q_pressed = self.robot_control.read_teleop_keyboard()
                    self.robot_control.execute_desired()

                    # self._steering_pid_controller = PID(Kp=10, Ki=0, Kd=0, setpoint=0, output_limits=(-100, 100))

                    # manually steer using arrowkeys, when ready, click s to start tuning
                    # if s is pressed, we want to tune
                    if s_pressed:
                        # user_input = input(
                        #     "Enter angle to steer to (+Left, -Right) or DISABLED: press Enter for default "
                        # )
                        user_input = 50
                        self.prev_angle = []

                        self.fig, self.ax = plt.subplots()
                        (self.desired_line,) = self.ax.plot(
                            [], [], label="Desired angle"
                        )
                        (self.current_line,) = self.ax.plot(
                            [], [], label="Current Angle"
                        )

                        # Set labels and title
                        self.ax.set_title("Tune Steering PID")
                        self.ax.set_xlabel("Time")
                        self.ax.set_ylabel("Steering angle [deg]")
                        self.ax.set_xlim(0, 10)
                        self.ax.set_ylim(-100, 100)

                        # Show a legend
                        self.ax.legend()

                        # Show the grid
                        self.ax.grid(True)

                        # Create an animation that updates the plot using the update function
                        animation = FuncAnimation(
                            self.fig, self.update_plot, frames=100, interval=100
                        )
                        # plt.ion()

                        if user_input:
                            input_rad = robot_control.math.radians(float(user_input))
                            self.desired_deg = float(user_input)

                            plt.show()

                        #     while (
                        #         self.robot_control.heading.current_steering_angle
                        #         != input_rad
                        #     ):
                        #         self.robot_control.steer_PID_deg(float(user_input))
                        #         print(
                        #             f"Current steering angle: {robot_control.math.degrees(self.robot_control.heading.current_steering_angle)}"
                        #         )

                        #         animation.event_source.start()

                        #     print(
                        #         f"ENDED: Current steering angle: {robot_control.math.degrees(self.robot_control.heading.current_steering_angle)}"
                        #     )
                        #     animation.event_source.stop()
                        #     print("waiting_start")
                        #     self.robot_control.timer.wait_seconds(2)
                        #     print("waiting_end")
                        # else:
                        #     # default turn of left 2 degrees, left 5 degrees
                        #     # print("Default turns of left 2 degrees")
                        #     pass
                        self.robot_control.steering_motor.set_speed(0)
                        print(
                            f"ENDED: Current steering angle: {robot_control.math.degrees(self.robot_control.heading.current_steering_angle)}"
                        )
                        self.robot_control.timer.wait_seconds(0.5)
                    self.robot_control.timer.wait_seconds(0.01)

                elif current_state == state.end:
                    print("Travel done")

                    # wait for user to finish analyzing plots
                    self.robot_control.path_planning.stop_interactive_plot()

                    # self.robot_control.plot_robot_position(printout=printout_arg)

                    # plt.ioff
                    self.robot_control.close_modules()
                    break

                elif current_state == state.teleop:
                    # self.robot_control.steer_robot(teleop_enable=True)
            
                    self.robot_control.handle_obstacle_in_path()

                    self.robot_control.steer_robot(teleop_enable=teleop_enable_arg)

                    self.robot_control.execute_desired()

                    # zz print encoder values (steering)
                    self.robot_control.steering_motor_encoder.get_steps()

                    # self.robot_control.drive_pwm(80)
                    # self.robot_control.close_modules()
                    # current_state = state.end

                    # self.robot_control.plot_robot_position(printout=printout_arg)

                elif current_state == state.test:

                    # self.robot_control.right_motor.set_speed(50)
                    # self.robot_control.left_motor.set_speed(50)
                    # self.robot_control.steering_motor.set_speed(50)

                    self.robot_control.left_limit_switch.print_contact()

                    # while(True):
                    #     print("simulated driving...")
                    #     self.robot_control.handle_obstacle_in_path()

                    # print("Start")
                    # sleep(10)
                    # print("end")
                    # current_state = state.end

                else:
                    pass

                # zz move?
                # _, q_pressed = self.robot_control._testing_read_arrow_keys()
                # if q_pressed:
                #     current_state = state.end

                # print(f"Current State: {current_state}")
                # self.robot_control.timer.wait_seconds(0.1)
                # zzEscape += 1
                # if zzEscape % 10 == 0:
                #     # self.robot_control.timer.wait_seconds(0.5)
                #     print(
                #         f"zzEscape: {zzEscape}, time: {self.robot_control.timer.get_delta_time()}"
                #     )
                # self.robot_control.plot_robot_position(printout=printout_arg)
                if zzEscape > 120000:
                    print("zzEscape")
                    current_state = state.end

        except KeyboardInterrupt:
            print("Exit handled")
            # self.robot_control.path_planning.stop_interactive_plot()  # zz check
            self.robot_control.close_modules()

    def speedup_start(self):
        # zz speedup path for testing
        self.robot_control.path_planning.path.nodes = (
            self.robot_control.path_planning.path.nodes[10:]
        )

        self.robot_control.current_position_node.x_coord = (
            self.robot_control.path_planning.path.nodes[0].x_coord
        )

        self.robot_control.current_position_node.y_coord = (
            self.robot_control.path_planning.path.nodes[0].y_coord
        )

        self.robot_control.update_next_node()
        # TODO call update_next_node() at start - if we never call it in state_machine, its assumed to be at (0,0)

    def update_plot(self, frame):
        # Generate time values up to the current frame

        # self.ax.set_xlim(0 + frame / 10, 10 + frame / 10)
        # self.ax.set_ylim(-100, 100)
        time_values = np.linspace(0, frame / 10, frame)

        # Create a constant y value (e.g., 5) for each time point
        # constant_y_value = np.full_like(time_values, 5)
        desired_angle = int(self.desired_deg)

        current_angle = np.rad2deg(self.robot_control.heading.current_steering_angle)

        self.prev_angle.append(current_angle)

        self.robot_control.steer_PID_deg(float(desired_angle))

        num_points = min(len(time_values), len(self.prev_angle))
        time_values = time_values[:num_points]
        prev_angle_slice = self.prev_angle[-num_points:]

        # Update the plot data
        self.desired_line.set_data(time_values, desired_angle)
        self.current_line.set_data(time_values, prev_angle_slice)
        self.ax.set_xlim(-10 + frame / 10, 10 + frame / 10)


# accommodate robot turning on, how to enter script, necessary hardware... (https://raspberrypi-guide.github.io/programming/run-script-on-boot)
def turn_on_robot():
    # handle turn on state
    pass


# initialize all
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IndexEngine")
    parser.add_argument("--teleop", action="store_true", help="Enable teleop")
    parser.add_argument("--simulate", action="store_true", help="Enable teleop")
    parser.add_argument(
        "--printout", action="store_true", help="Enable printout of position data"
    )
    parser.add_argument(
        "--tune-steer", action="store_true", help="Tune the steering PID"
    )
    parser.add_argument("--fstart", action="store_true", help="Skip start of path")
    parser.add_argument("--test", action="store_true", help="Enable testing state")
try:
    cli = parser.parse_args()

    # Set teleop_enable to True if --teleop is specified
    teleop_enable_arg = cli.teleop
    simulate_all_arg = cli.simulate
    printout_arg = cli.printout
    tune_steering_arg = cli.tune_steer
    speedup_arg = cli.fstart
    test_arg = cli.test

    # zz temp auto enable simulate for testing convenience
    # TODO perhaps if gpio is not available, auto implement then auto enable simulate
    # simulate_enable_arg = True

except Exception as e:
    print("Error:", str(e))
    sys.exit(1)
robot = Robot()
robot.state_machine()
