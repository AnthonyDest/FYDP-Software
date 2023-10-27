import RPi.GPIO as gpio
from motor_driver import left_motor
from encoder import left_encoder

left_motor.move_forward
left_motor.set_speed(100)

try:
    while True:
        left_value = left_encoder.value

        print(f"Left Encoder Value: {left_value}")

except KeyboardInterrupt:
    gpio.cleanup()

