from motor_driver import Motor
import time

left_motor = Motor(8, 7, 12)

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
