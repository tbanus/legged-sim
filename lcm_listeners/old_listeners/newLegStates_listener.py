import lcm
import csv
import math
import os
import sys
sys.path.append('../')
# from lcm_types.python.debug_data_lcmt import debug_data_lcmt
from lcm_types.python.leg_control_data_lcmt import leg_control_data_lcmt
from lcm_types.python.leg_control_command_lcmt import leg_control_command_lcmt
msg_type1=leg_control_command_lcmt
msg_type2=leg_control_data_lcmt
global data_writer
global data_f
global attr
global filename
filename="leg_data_log"
lcm_channel1="leg_control_command"
lcm_channel2="leg_control_data"
time =0.0
attr1 = []
attr2 = []
LogList = []
def HeaderGenerator():
    Headers = []
    for i in range(len(msg_type1.__slots__)):
        if msg_type1.__dimensions__[i]:
            for j in range(msg_type1.__dimensions__[i][0]):
                Headers = Headers + [ msg_type1.__slots__[i] + "_" + str(j) ]
        else :
            Headers = Headers + [ msg_type1.__slots__[i]]
    for i in range(len(msg_type2.__slots__)):
        if msg_type2.__dimensions__[i]:
            for j in range(msg_type2.__dimensions__[i][0]):
                Headers = Headers + [ msg_type2.__slots__[i] + "_" + str(j) ]
        else :
            Headers = Headers + [ msg_type2.__slots__[i]]

    Headers = Headers + ["time"]
    return Headers


def my_handler1(channel, data):
    # print(data)
    global attr1
    global LogList

    wbcdata =msg_type1.decode(data)


    
    for at in attr1:
        attrlist = getattr(wbcdata,at)
        if type(attrlist)==tuple or type(attrlist)==list:
            # attrlist=attrlist.strip()
            for j in range(len(attrlist)):
                LogList = LogList + [attrlist[j]]
        else:
                LogList = LogList + [attrlist]
            
    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
    # data_writer.writerow(LogList) 

def my_handler2(channel, data):
    # print(data)
    global attr2
    global LogList


    wbcdata =msg_type2.decode(data)


    
    for at in attr2:
        attrlist = getattr(wbcdata,at)
        if type(attrlist)==tuple or type(attrlist)==list:
            # attrlist=attrlist.strip()
            for j in range(len(attrlist)):
                LogList = LogList + [attrlist[j]]
        else:
                LogList = LogList + [attrlist]
    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
     LogList = LogList + [time]
    data_writer.writerow(LogList) 
    time = time + 0.002
    LogList.clear()

user_input = input('remove current file ? y or n')
if user_input == 'y': 
    file = '../lcm_logs/'+filename
    try:
        os.remove (file)
        data_f = open('../lcm_logs/'+filename, 'a',newline='')
    except FileNotFoundError:
        data_f = open('../lcm_logs/'+filename, 'x',newline='')
    # data_f = open('../lcm_logs/'+filename, 'a',newline='')
    data_writer = csv.writer(data_f)
    Headers = HeaderGenerator()
    print(Headers)
    data_writer.writerow(Headers) 
else:
    data_f = open('../lcm_logs/'+filename, 'a',newline='')
    data_writer = csv.writer(data_f)
for i in range(len(msg_type1.__slots__)):
  attr1 = attr1 + [ msg_type1.__slots__[i] ]    
for i in range(len(msg_type2.__slots__)):
  attr2 = attr2 + [ msg_type2.__slots__[i] ]    
print(attr1, attr2)

 



lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription1 = lc.subscribe(lcm_channel1, my_handler1)
subscription2 = lc.subscribe(lcm_channel2, my_handler2)

try:
    while True:

        lc.handle()
except KeyboardInterrupt:
   pass
    

lc.unsubscribe(subscription1)
#lc.unsubscribe(subscription2)
