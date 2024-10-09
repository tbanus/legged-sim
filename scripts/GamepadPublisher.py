
#from __future__ import print_function
import lcm
import sys
sys.path.append('../')
from lcm_types.python.gamepad_lcmt import gamepad_lcmt
import time
import threading
# Uses github-zeth/inputs
# pip3 install inputs
from inputs import get_gamepad 
from inputs import get_mouse


def getGameData():
    """Just print out some event infomation when the gamepad is used."""
   
    events = get_gamepad()
    
    for event in events:
        print(event.ev_type,event.code, event.state)
    return events    
      

def PeriodicLCM():
    while True:
        time.sleep(0.0125)
        #if msg.start==0:
            # print(msg.start)
        lc.publish("command", msg.encode())


def PeriodicLCM():
    
    while PeriodicENABLE:
        time.sleep(0.1)
       # print(msg.leftStickAnalog[1])
        lc.publish("asd", msg.encode())
lc=lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
PeriodicENABLE=True

x = threading.Thread(target=PeriodicLCM)
msg = gamepad_lcmt()
x = threading.Thread(target=PeriodicLCM)
i=1
x.start()
while True:
    
    
    ev= getGameData()
    for event in ev:
        
        # Analog Sticks
        if event.code == "ABS_X" :
            msg.leftStickAnalog[0] = event.state
        if event.code == "ABS_Y" :
            msg.leftStickAnalog[1] = event.state
        if event.code == "ABS_RX" :
            msg.rightStickAnalog[0] = event.state
        if event.code == "ABS_RY" :
            msg.rightStickAnalog[1] = event.state
        if event.code == "BTN_THUMBL" :
            msg.leftStickButton = event.state
        if event.code == "BTN_THUMBR" :
            msg.rightStickButton = event.state
            
        # Other Buttons
        if event.code == "BTN_START" :
            msg.start = event.state
        if event.code == "BTN_SELECT":
            msg.back = event.state
            
        # Trigger buttons
        if event.code == "BTN_TL" :
            msg.leftBumper = event.state
        if event.code == "BTN_TR" :
            msg.rightBumper = event.state
        if event.code == "ABS_Z" :
            msg.leftTriggerAnalog = event.state
            msg.leftTriggerButton = bool(event.state)
        if event.code == "ABS_RZ" :
            msg.rightTriggerAnalog = event.state
            msg.rightTriggerButton = bool(event.state)
            
        # A,B,X,Y right buttons
        if event.code == "BTN_NORTH" :
            msg.x = event.state
        if event.code == "BTN_WEST" :
            msg.y = event.state
        if event.code == "BTN_EAST" :
            msg.b = event.state 
        if event.code == "BTN_SOUTH" :
            msg.a = event.state 
        
        # if event.code == "BTN_WEST" :
        #     msg.y = event.state 
        # if event.code == "BTN_WEST" :
        #     msg.y = event.state 
        # if event.code == "BTN_WEST" :
        #     msg.y = event.state  
    lc.publish("command", msg.encode())
PeriodicENABLE=False
x.join()
x.stop()
