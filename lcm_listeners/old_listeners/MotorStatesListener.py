import lcm
import csv
import math
import os
from LCM_CANcommand import LCM_CANcommand
from LCM_CANData import LCM_CANData

global MotorStates_writer
global MotorStates_f
global attr1
global attr2
attr1= []
attr2= []
global Loglist
def HeaderGenerator(obj):
    Headers = []
    for i in range(len(obj.__slots__)):
        if (obj.__dimensions__[i]):
            for j in range(obj.__dimensions__[i][0]):
                Headers = Headers + [ obj.__slots__[i] + "_" + str(j) ]
        else:
            Headers = Headers +  [obj.__slots__[i]]
    return Headers


def my_handler1(channel, data):
    global attr1
    global attr2
    global LogList
    

    if(channel=="CAN Commands"):
        DATA =LCM_CANcommand.decode(data)
        attr = attr1
    elif(channel=="CAN Data"):
        DATA =LCM_CANData.decode(data)
        attr = attr2


    
    for at in attr:
        attrlist = getattr(DATA,at)
        try:
            for j in range(len(attrlist)):
                LogList = LogList + [attrlist[j]]
        except:
            LogList = LogList + [attrlist]     
      
     

    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
    



user_input = input('remove current file ? y or n')
if user_input == 'y': 
    file = '../lcm_logs/MotorStates'
    os.remove (file)
    MotorStates_f = open('../lcm_logs/MotorStates', 'a',newline='')
    MotorStates_writer = csv.writer(MotorStates_f)
    Headers1 = HeaderGenerator(LCM_CANcommand)
    print(Headers1)
   
    Headers2 = HeaderGenerator(LCM_CANData)
    print(Headers2)
    MotorStates_writer.writerow(Headers1+Headers2)


else:
    MotorStates_f = open('../lcm_logs/MotorStates', 'a',newline='')
    MotorStates_writer = csv.writer(MotorStates_f)

for i in range(len(LCM_CANcommand.__slots__)):
  attr1 = attr1 + [ LCM_CANcommand.__slots__[i] ]
for i in range(len(LCM_CANData.__slots__)):
  attr2 = attr2 + [ LCM_CANData.__slots__[i] ]


print(attr1)
print(attr2)

 



lc1 = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

lc2 = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription1 = lc1.subscribe("CAN Commands", my_handler1)
subscription2 = lc2.subscribe("CAN Data", my_handler1)

try:
    while True:
        LogList = []
        lc1.handle()
        lc2.handle()
        MotorStates_writer.writerow(LogList) 
except KeyboardInterrupt:
   pass
    

lc.unsubscribe(subscription1)
#lc.unsubscribe(subscription2)
