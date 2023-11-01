import RPi.GPIO as GPIO
from time import sleep

counter = 10

Enc_A = 23  
Enc_B = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(Enc_A, GPIO.IN)
GPIO.setup(Enc_B, GPIO.IN)


def init():
    print ("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A, GPIO.IN)
    GPIO.setup(Enc_B, GPIO.IN)
    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode, bouncetime=10)
    return


def rotation_decode(Enc_A):
    global counter
    sleep(0.002)
    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)

    if (Switch_A == 1) and (Switch_B == 0):
        counter += 1
        print ("direction -> ", counter)
        while Switch_B == 0:
            Switch_B = GPIO.input(Enc_B)
        while Switch_B == 1:
            Switch_B = GPIO.input(Enc_B)
        return

    elif (Switch_A == 1) and (Switch_B == 1):
        counter -= 1
        print ("direction <- ", counter)
        while Switch_A == 1:
            Switch_A = GPIO.input(Enc_A)
        return
    else:
        return

def main():
    try:
        init()
        while True :
            sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()

# from gpiozero import RotaryEncoder

# # Define constants for the GPIO pins
# LEFT_ENCODER_A_PIN = 23
# LEFT_ENCODER_B_PIN = 24
# # RIGHT_ENCODER_A_PIN = 22
# # RIGHT_ENCODER_B_PIN = 23

# # Initialize the left and right encoders
# left_encoder = RotaryEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN, max)
# # right_encoder = RotaryEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
# # right_encoder = RotaryEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, wrap=True, max_steps=180)
# # max steps


# try:
#     while True:
#         # Read values from the left and right encoders
#         # left_value = left_encoder.value
#         # right_value = right_encoder.value

#         left_value = left_encoder.steps

#         print(f"Left Encoder Value: {left_value}")
#         # print(f"Right Encoder Value: {right_value}")

# except KeyboardInterrupt:
#     # Close the left and right encoders on Ctrl+C
#     left_encoder.close()
#     # right_encoder.close()



