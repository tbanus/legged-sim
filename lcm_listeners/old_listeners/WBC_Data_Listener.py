import lcm
import csv
import math
import os
from wbc_test_data_t import wbc_test_data_t

global WBC_Data_writer
global WBC_data_f
global attr
attr = []
def HeaderGenerator():
    Headers = []
    for i in range(len(wbc_test_data_t.__slots__)):
      
        for j in range(wbc_test_data_t.__dimensions__[i][0]):
            Headers = Headers + [ wbc_test_data_t.__slots__[i] + "_" + str(j) ]

    return Headers


def my_handler1(channel, data):
    global attr
    LogList = []

    wbcdata =wbc_test_data_t.decode(data)


    
    for at in attr:
        attrlist = getattr(wbcdata,at)
        for j in range(len(attrlist)):

            LogList = LogList + [attrlist[j]]
    print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
    WBC_Data_writer.writerow(LogList) 



user_input = input('remove current file ? y or n')
if user_input == 'y': 
    file = '../lcm_logs/Wbc_data_file'
    os.remove (file)
    WBC_data_f = open('../lcm_logs/Wbc_data_file', 'a',newline='')
    WBC_Data_writer = csv.writer(WBC_data_f)
    Headers = HeaderGenerator()
    print(Headers)
    WBC_Data_writer.writerow(Headers) 
else:
    WBC_data_f = open('../lcm_logs/Wbc_data_file', 'a',newline='')
    WBC_Data_writer = csv.writer(WBC_data_f)

for i in range(len(wbc_test_data_t.__slots__)):
  attr = attr + [ wbc_test_data_t.__slots__[i] ]    
print(attr)

 



lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription1 = lc.subscribe("wbc_lcm_data", my_handler1)
#subscription2 = lc.subscribe("CAN Data", my_handler2)

try:
    while True:

        lc.handle()
except KeyboardInterrupt:
   pass
    

lc.unsubscribe(subscription1)
#lc.unsubscribe(subscription2)
