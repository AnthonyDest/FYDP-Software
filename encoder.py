from gpiozero import RotaryEncoder

# Define constants for the GPIO pins
LEFT_ENCODER_A_PIN = 23
LEFT_ENCODER_B_PIN = 24
# RIGHT_ENCODER_A_PIN = 22
# RIGHT_ENCODER_B_PIN = 23

# Initialize the left and right encoders
left_encoder = RotaryEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN)
# right_encoder = RotaryEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
# right_encoder = RotaryEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN, wrap=True, max_steps=180)
# max steps


# try:
#     while True:
#         # Read values from the left and right encoders
#         left_value = left_encoder.value
#         # right_value = right_encoder.value

#         print(f"Left Encoder Value: {left_value}")
#         # print(f"Right Encoder Value: {right_value}")

# except KeyboardInterrupt:
#     # Close the left and right encoders on Ctrl+C
#     left_encoder.close()
#     # right_encoder.close()
