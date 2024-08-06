import lcm
import csv
import math
import os
import sys
sys.path.append('../')
# from lcm_types.debug_data_lcmt import debug_data_lcmt
from lcm_types.python.leg_control_command_lcmt import leg_control_command_lcmt
msg_type=leg_control_command_lcmt
global data_writer
global data_f
global attr
global filename
filename="leg_control_command_log"
lcm_channel="leg_control_command"
attr = []
def HeaderGenerator():
    Headers = []
    for i in range(len(msg_type.__slots__)):
        if msg_type.__dimensions__[i]:
            for j in range(msg_type.__dimensions__[i][0]):
                Headers = Headers + [ msg_type.__slots__[i] + "_" + str(j) ]
        else :
            Headers = Headers + [ msg_type.__slots__[i]]


    return Headers


def my_handler1(channel, data):
    # print(data)
    global attr
    LogList = []

    wbcdata =msg_type.decode(data)


    
    for at in attr:
        attrlist = getattr(wbcdata,at)
        if type(attrlist)==tuple or type(attrlist)==list:
            # attrlist=attrlist.strip()
            for j in range(len(attrlist)):
                LogList = LogList + [attrlist[j]]
        else:
                LogList = LogList + [attrlist]
    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
    data_writer.writerow(LogList) 



user_input = input('remove current file ? y or n')
if user_input == 'y': 
    file = '../opy_logs/'+filename
    try:
        os.remove (file)
        data_f = open('../logs/'+filename, 'a',newline='')
    except FileNotFoundError:
        data_f = open('../logs/'+filename, 'x',newline='')
    # data_f = open('../opy_logs/'+filename, 'a',newline='')
    data_writer = csv.writer(data_f)
    Headers = HeaderGenerator()
    print(Headers)
    data_writer.writerow(Headers) 
else:
    data_f = open('../logs/'+filename, 'a',newline='')
    data_writer = csv.writer(data_f)

for i in range(len(msg_type.__slots__)):
  attr = attr + [ msg_type.__slots__[i] ]    
print(attr)

 



lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription1 = lc.subscribe(lcm_channel, my_handler1)
#subscription2 = lc.subscribe("CAN Data", my_handler2)

try:
    while True:

        lc.handle()
except KeyboardInterrupt:
   pass
    

lc.unsubscribe(subscription1)
#lc.unsubscribe(subscription2)
