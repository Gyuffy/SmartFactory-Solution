# -*- coding: utf-8 -*-
import gpiod
import time

# GPIO pin numbers
SERVO_PIN = 18
btn1 = 23
btn2 = 24

# Servo angle constatns
CENTER_ANGLE = 135
RIGHT_ANGLE = 175
LEFT_ANGLE = 95

# Open GPIO chip
chip = gpiod.Chip("gpiochip0")

# Get GPIO lines
servo_line = chip.get_line(SERVO_PIN)
button1_line = chip.get_line(btn1)
button2_line = chip.get_line(btn2)

# Set up GPIO lines for buttons
servo_line.request(consumer="servo", type=gpiod.LINE_REQ_DIR_OUT)
button1_line.request(
    consumer="button1",
    type=gpiod.LINE_REQ_DIR_IN,
    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
)
button2_line.request(
    consumer="button2",
    type=gpiod.LINE_REQ_DIR_IN,
    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
)

last_button1_state = 1
last_button2_state = 1
current_button = None


def set_servo(angle):
    pulse_width = (angle / 270) * (0.0025 - 0.0005) + 0.0005
    for _ in range(10):
        servo_line.set_value(1)
        time.sleep(pulse_width)
        servo_line.set_value(0)
        time.sleep(0.02 - pulse_width)


try:
    while True:
        # Read button
        button1_state = button1_line.get_value()
        button2_state = button2_line.get_value()

        # Print button States
        if button1_state != last_button1_state:
            print(f"Button 1 : {'Pressed' if button1_state == 0 else 'Released'}")

            if button1_state == 0:
                if current_button == 1:
                    print(f"Moving servo to {CENTER_ANGLE} degrees")
                    set_servo(CENTER_ANGLE)
                    current_button = None
                else:
                    print(f"Moving servo to {RIGHT_ANGLE} degrees")
                    set_servo(RIGHT_ANGLE)
                    current_button = 1

        if button2_state != last_button2_state:
            print(f"Button 2 : {'Pressed' if button2_state == 0 else 'Released'}")

            if button2_state == 0:
                if current_button == 2:
                    print(f"Moving servo to {CENTER_ANGLE} degrees")
                    set_servo(CENTER_ANGLE)
                    current_button = None
                else:
                    print(f"Moving servo to {LEFT_ANGLE} degrees")
                    set_servo(LEFT_ANGLE)
                    current_button = 2

        # Update last button States
        last_button1_state = button1_state
        last_button2_state = button2_state

        # Reduce CPU Resource
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    # Relase GPIO lines
    servo_line.release()
    button1_line.release()
    button2_line.release()
