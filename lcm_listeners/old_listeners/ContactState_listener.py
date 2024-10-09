import lcm
import csv
import math
import os
from contact_kalman_lcmt import contact_kalman_lcmt
msg_type=contact_kalman_lcmt
global data_writer
global data_f
global attr
global filename
filename="contact_kalman_log"
lcm_channel="contact_kalman"
attr = []
def HeaderGenerator():
    Headers = []
    for i in range(len(msg_type.__slots__)):
      
        for j in range(msg_type.__dimensions__[i][0]):
            Headers = Headers + [ msg_type.__slots__[i] + "_" + str(j) ]

    return Headers


def my_handler1(channel, data):
    global attr
    LogList = []

    wbcdata =msg_type.decode(data)


    
    for at in attr:
        attrlist = getattr(wbcdata,at)
        for j in range(len(attrlist)):

            LogList = LogList + [attrlist[j]]
    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
    data_writer.writerow(LogList) 



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
