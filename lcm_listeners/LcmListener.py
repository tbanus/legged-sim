import lcm
import csv
import math
import os


class LcmListener():

    def __init__(self, filename, lcm_channel, msg_type, lcm_channel2=None, msg_type2=None, enable_prints=False) -> None:
        if lcm_channel2 is not None:
            print("setting up the lcm listener with two channels\n")
            self.setup_double(filename, lcm_channel, msg_type, lcm_channel2, msg_type2)
            
        else:
            print("setting up the lcm listener with single channel\n")
            self.setup_single(filename,lcm_channel,msg_type)



    def setup_single(self, filename, lcm_channel, msg_type ) -> None:

     
        self.LogList=[]
        self.msg_type=msg_type
        self.attr=[]
        self.enable=1
        user_input = input('remove current file ? y or n')
        if user_input == 'y': 
            file = '../lcm_logs/'+filename+".log"
            try:
                os.remove (file)
                self.data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            except FileNotFoundError:
                self.data_f = open('../lcm_logs/'+filename+".log", 'x',newline='')
            # data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            self.data_writer = csv.writer(self.data_f)
            Headers = self.generate_header(self.msg_type)
            Headers = Headers + ["time"]
            print(Headers)
            self.data_writer.writerow(Headers) 
        else:
            self.data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            self.data_writer = csv.writer(self.data_f)

        for i in range(len(self.msg_type.__slots__)):
            self.attr = self.attr + [ self.msg_type.__slots__[i] ]    
        print(self.attr)
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.subscription1 = self.lc.subscribe(lcm_channel, self.handler)
        self.time=0.0
   
   
    def setup_double(self, filename, lcm_channel, msg_type, lcm_channel2, msg_type2) -> None:
     
        self.LogList=[]
        self.msg_type=msg_type
        self.msg_type2=msg_type2
        self.attr=[]
        self.attr2=[]
        self.enable=1
        user_input = input('remove current file ? y or n')
        if user_input == 'y': 
            file = '../lcm_logs/'+filename+".log"
            try:
                os.remove (file)
                self.data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            except FileNotFoundError:
                self.data_f = open('../lcm_logs/'+filename+".log", 'x',newline='')
            # data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            self.data_writer = csv.writer(self.data_f)
            Headers = self.generate_header(self.msg_type2)
            # print("headers1", Headers)
            Headers =Headers+self.generate_header(self.msg_type)
            Headers = Headers + ["time"]
            print(Headers)
           
            self.data_writer.writerow(Headers) 
        else:
            self.data_f = open('../lcm_logs/'+filename+".log", 'a',newline='')
            self.data_writer = csv.writer(self.data_f)

        for i in range(len(self.msg_type2.__slots__)):
            self.attr2 = self.attr2 + [ self.msg_type2.__slots__[i] ]   
        for i in range(len(self.msg_type.__slots__)):
            self.attr = self.attr + [ self.msg_type.__slots__[i] ]     
        print(self.attr, self.attr2)
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.subscription2 = self.lc.subscribe(lcm_channel2, self.handler2)
        self.subscription1 = self.lc.subscribe(lcm_channel, self.handler)
     
        self.time=0.0
    
    def run(self):
        try:
            while self.enable:

                self.lc.handle()
        except KeyboardInterrupt:
            pass
            

        self.lc.unsubscribe(self.subscription1)



    def generate_header(self, msg_type):
        Headers = []
        for i in range(len(msg_type.__slots__)):
            if msg_type.__dimensions__[i]:
                for j in range(msg_type.__dimensions__[i][0]):
                    Headers = Headers + [ msg_type.__slots__[i] + "_" + str(j) ]
            else :
                Headers = Headers + [ msg_type.__slots__[i]]

        
        return Headers


    def handler(self, channel, data):
        # print(data)


        msg =self.msg_type.decode(data)


        
        for at in self.attr:
            attrlist = getattr(msg,at)
            if type(attrlist)==tuple or type(attrlist)==list:
                # attrlist=attrlist.strip()
                for j in range(len(attrlist)):
                    self.LogList = self.LogList + [attrlist[j]]
            else:
                    self.LogList = self.LogList + [attrlist]
        # print("Received message on channel \"%s\"" % channel)

        self.LogList = self.LogList + [self.time]
        self.data_writer.writerow(self.LogList) 
        self.LogList.clear()
        self.time = self.time + 0.002



    def handler2(self, channel, data):
        # print(data)


        msg =self.msg_type2.decode(data)


        
        for at in self.attr2:
            attrlist = getattr(msg,at)
     
            if type(attrlist)==tuple or type(attrlist)==list:
                # attrlist=attrlist.strip()
                for j in range(len(attrlist)):
                    self.LogList = self.LogList + [attrlist[j]]
            else:
                    self.LogList = self.LogList + [attrlist]
        # print("Received message on channel \"%s\"" % channel)
        





#subscription2 = lc.subscribe("CAN Data", my_handler2)


#lc.unsubscribe(subscription2)
