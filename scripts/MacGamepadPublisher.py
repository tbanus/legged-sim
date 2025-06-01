#!/usr/bin/env python3
import lcm
import sys
sys.path.append('../')
import time
import threading
import pygame
from lcm_types.python.gamepad_lcmt import gamepad_lcmt

# Initialize LCM with your UDP configuration
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

# Global flag to control the periodic thread
PeriodicENABLE = True

# Initialize pygame and joystick module
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    sys.exit(1)
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Using joystick: {joystick.get_name()}")

# Create a gamepad_lcmt message instance (fields depend on your LCM type)
msg = gamepad_lcmt()

# Define the periodic LCM publisher thread; publishes on channel "asd" every 0.1 sec.
def PeriodicLCM():
    global PeriodicENABLE
    while PeriodicENABLE:
        time.sleep(0.1)
        lc.publish("asd", msg.encode())

# Start the periodic publisher thread
periodic_thread = threading.Thread(target=PeriodicLCM)
periodic_thread.start()

try:
    # Main loop: read joystick state and publish on channel "command"
    while True:
        # Pump pygame event loop to update joystick events
        pygame.event.pump()

        # Update analog sticks
        # (Assuming axes 0 and 1 are left stick and 3 and 4 are right stick)
        msg.leftStickAnalog[0] = int(joystick.get_axis(0) * 32767)
        msg.leftStickAnalog[1] = int(joystick.get_axis(1) * 32767)
        msg.rightStickAnalog[0] = int(joystick.get_axis(3) * 32767)
        msg.rightStickAnalog[1] = int(joystick.get_axis(4) * 32767)

        # Update stick buttons (example: adjust index as your controller maps them)
        msg.leftStickButton  = joystick.get_button(8)
        msg.rightStickButton = joystick.get_button(9)

        # Update start and back buttons
        msg.start = joystick.get_button(10)
        msg.back  = joystick.get_button(11)
        msg.start = 1
        # Update bumper buttons
        msg.leftBumper  = joystick.get_button(4)
        msg.rightBumper = joystick.get_button(5)

        # Update triggers (assuming they are provided as axes; adjust indices if needed)
        left_trigger  = joystick.get_axis(2)
        right_trigger = joystick.get_axis(5)
        msg.leftTriggerAnalog  = int(left_trigger * 32767)
        msg.rightTriggerAnalog = int(right_trigger * 32767)
        msg.leftTriggerButton  = 1 if left_trigger > 0.5 else 0
        msg.rightTriggerButton = 1 if right_trigger > 0.5 else 0

        # Update face buttons (A, B, X, Y)
        msg.x = joystick.get_button(0)
        msg.b = joystick.get_button(1)
        msg.y = joystick.get_button(2)
        msg.a = joystick.get_button(3)

        # Publish the updated message on channel "command"
        lc.publish("command", msg.encode())

        # Sleep briefly to control the polling rate (12.5 msec as in your original code)
        time.sleep(0.0125)

except KeyboardInterrupt:
    print("Exiting joystick loop...")

finally:
    PeriodicENABLE = False
    periodic_thread.join()
    joystick.quit()
    pygame.quit()
