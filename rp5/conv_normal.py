# -*- coding: utf-8 -*-
import gpiod
import time
import threading

# GPIO pin numbers
dir_pin = 17
step_pin = 27
enable_pin = 22
btn1 = 23
btn2 = 24

# Open GPIO chip
chip = gpiod.Chip("gpiochip0")

# Get GPIO lines
dir_line = chip.get_line(dir_pin)
step_line = chip.get_line(step_pin)
enable_line = chip.get_line(enable_pin)
button1_line = chip.get_line(btn1)
button2_line = chip.get_line(btn2)

# Set up GPIO lines for buttons
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
step_line.request(consumer="step", type=gpiod.LINE_REQ_DIR_OUT)
enable_line.request(consumer="enable", type=gpiod.LINE_REQ_DIR_OUT)
button1_line.request(
    consumer="button1",
    type=gpiod.LINE_REQ_DIR_IN,
    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
)
button2_line.request(
    consumer="button1",
    type=gpiod.LINE_REQ_DIR_IN,
    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP,
)

# Motor control variables
motor_running = False
motor_direction = 1  # 1 for CW, 0 for CCW
current_button = None


def step_motor():
    global motor_running
    while motor_running:
        step_line.set_value(1)
        time.sleep(0.0001)
        step_line.set_value(0)
        time.sleep(0.0001)


def print_motor_status():
    print(
        f"Motor Running: {motor_running}, Direction: {'CW' if motor_direction == 0 else 'CCW'}"
    )


last_button1_state = 1
last_button2_state = 1

try:
    while True:
        # read button
        button1_state = button1_line.get_value()
        button2_state = button2_line.get_value()

        # print button states
        if button1_state != last_button1_state:
            print(f"Button 1: {'Pressed' if button1_state == 0 else 'Released'}")

            if button1_state == 0:  # 눌렸을 때
                if current_button == 1:
                    print(f"Stopping Motor")
                    motor_running = False
                    enable_line.set_value(1)  # enable motor
                    current_button = None
                else:  # first press
                    print(f"Starting Motor CW")
                    motor_direction = 0
                    dir_line.set_value(motor_direction)
                    enable_line.set_value(0)  # enable motor
                    motor_running = True
                    current_button = 1
                    threading.Thread(target=step_motor).start()

                print_motor_status()

        if button2_state != last_button2_state:
            print(f"Button 2: {'Pressed' if button2_state == 0 else 'Released'}")
            if button2_state == 0:  # 눌렸을 때
                if current_button == 2:
                    print(f"Stopping Motor")
                    motor_running = False
                    enable_line.set_value(1)  # enable motor
                    current_button = None
                else:  # first press
                    print(f"Starting Motor CCW")
                    motor_direction = 1
                    dir_line.set_value(motor_direction)
                    enable_line.set_value(0)  # enable motor
                    motor_running = True
                    current_button = 2
                    threading.Thread(target=step_motor).start()

                print_motor_status()

        # update last button states
        last_button1_state = button1_state
        last_button2_state = button2_state

        # reduce cpu resource
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    # Release GPIO lines
    motor_running = False
    time.sleep(0.1)
    enable_line.set_value(1)
    dir_line.release()
    step_line.release()
    enable_line.release()
    button1_line.release()
    button2_line.release()
