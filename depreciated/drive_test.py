import importlib

try:
    # simulating_on_windows = False
    object_name = "left_motor"

    module = importlib.import_module("motor_driver")
    left_motor = getattr(module, object_name)
except (ImportError, AttributeError):
    print("SIMULATING MOTORS")
    simulating_on_windows = True

# from motor_driver import left_motor
import time

left_motor.set_speed(100)
timeValue = 2

try:
    while True:
        print("running")

        print("20")
        left_motor.set_speed(20)
        time.sleep(timeValue)

        print("40")
        left_motor.set_speed(40)
        time.sleep(timeValue)

        print("60")
        left_motor.set_speed(60)
        time.sleep(timeValue)

        print("80")
        left_motor.set_speed(80)
        time.sleep(timeValue)

        print("100")
        left_motor.set_speed(100)
        time.sleep(timeValue)


except KeyboardInterrupt:
    left_motor.close()

print("END")
