import lcm
import csv
import math
import os
import sys
sys.path.append('../../')
from lcm_types.python.leg_control_data_lcmt import leg_control_data_lcmt
from lcm_types.python.leg_control_command_lcmt import leg_control_command_lcmt


global writerLegSates
global fileLegStates
user_input = input('remove current file ? y or n')
if user_input == 'y': 
    file = '../lcm_logs/LegStatesLog'
    if(os.path.exists(file) and os.path.isfile(file)):
        os.remove(file)
    
    fileLegStates = open('../../lcm_logs/LegStatesLog', 'a',newline='')
    writerLegStates = csv.writer(fileLegStates)
    Headers = ["Leg-1 Ab/Ad Angle" ,"Leg-1 Ab/Ad Velocity "," Leg-1 Ab/Ad Tau Est.","Leg-1 Foot Pos x","Leg-1 Foot Force X", "Leg-1 Foot Force EST X", \
               "Leg-1 Hip Angle" ,"Leg-1 Hip Velocity "," Leg-1 Hip Tau Est.","Leg-1 Foot Pos y ","Leg-1 Foot Force Y","Leg-1 Foot Force EST Y", \
               "Leg-1 Knee Angle" ,"Leg-1 Knee Velocity "," Leg-1 Knee Tau Est."," Leg-1 Foot Pos z","Leg-1 Foot Force Z","Leg-1 Foot Force EST Z", \
               "Leg-2 Ab/Ad Angle" ,"Leg-2 Ab/Ad Velocity "," Leg-2 Ab/Ad Tau Est.", "Leg-2 Foot Pos x", "Leg-2 Foot Force X","Leg-2 Foot Force EST X",\
               "Leg-2 Hip Angle" ,"Leg-2 Hip Velocity "," Leg-2 Hip Tau Est.", "Leg-2 Foot Pos y ", "Leg-2 Foot Force Y","Leg-2 Foot Force EST Y",\
               "Leg-2 Knee Angle" ,"Leg-2 Knee Velocity "," Leg-2 Knee Tau Est.", " Leg-2 Foot Pos z.", "Leg-2 Foot Force Z","Leg-2 Foot Force EST Z",\
               "Leg-3 Ab/Ad Angle" ,"Leg-3 Ab/Ad Velocity "," Leg-3 Ab/Ad Tau Est.", "Leg-3 Foot Pos x",  "Leg-3 Foot Force X","Leg-3 Foot Force EST X",\
               "Leg-3 Hip Angle" ,"Leg-3 Hip Velocity "," Leg-3 Hip Tau Est.", "Leg-3 Foot Pos y ", "Leg-3 Foot Force Y","Leg-3 Foot Force EST Y",\
               "Leg-3 Knee Angle" ,"Leg-3 Knee Velocity "," Leg-3 Knee Tau Est.", " Leg-3 Foot Pos z.","Leg-3 Foot Force Z","Leg-3 Foot Force EST Z",\
               "Leg-4 Ab/Ad Angle" ,"Leg-4 Ab/Ad Velocity "," Leg-4 Ab/Ad Tau Est.", "Leg-4 Foot Pos x" , "Leg-4 Foot Force X","Leg-4 Foot Force EST X",\
               "Leg-4 Hip Angle" ,"Leg-4 Hip Velocity "," Leg-4 Hip Tau Est.", "Leg-4 Foot Pos y ","Leg-4 Foot Force Y", "Leg-4 Foot Force EST Y",\
               "Leg-4 Knee Angle" ,"Leg-4 Knee Velocity "," Leg-4 Knee Tau Est.", " Leg-4 Foot Pos z.","Leg-4 Foot Force Z","Leg-4 Foot Force EST Z", \
               "Leg-1 Ab/Ad Angle Command" ,"Leg-1 Ab/Ad Velocity Command "," Leg-1 Ab/Ad Tau FF Command", \
               "Leg-1 Hip Angle Command" ,"Leg-1 Hip Velocity Command"," Leg-1 Hip Tau FF Command", \
               "Leg-1 Knee Angle Command" ,"Leg-1 Knee Velocity Command"," Leg-1 Knee Tau FF Command", \
               "Leg-2 Ab/Ad Angle Command" ,"Leg-2 Ab/Ad Velocity Command"," Leg-2 Ab/Ad Tau FF Command", \
               "Leg-2 Hip Angle Command" ,"Leg-2 Hip Velocity Command"," Leg-2 Hip Tau FF Command", \
               "Leg-2 Knee Angle Command" ,"Leg-2 Knee Velocity Command"," Leg-2 Knee Tau FF Command", \
               "Leg-3 Ab/Ad Angle Command" ,"Leg-3 Ab/Ad Velocity Command"," Leg-3 Ab/Ad Tau FF Command", \
               "Leg-3 Hip Angle Command" ,"Leg-3 Hip Velocity Command"," Leg-3 Hip Tau FF Command", \
               "Leg-3 Knee Angle Command" ,"Leg-3 Knee Velocity Command "," Leg-3 Knee Tau FF Command", \
               "Leg-4 Ab/Ad Angle Command" ,"Leg-4 Ab/Ad Velocity Command "," Leg-4 Ab/Ad Tau FF Command", \
               "Leg-4 Hip Angle Command" ,"Leg-4 Hip Velocity Command "," Leg-4 Hip Tau FF Command", \
               "Leg-4 Knee Angle Command" ,"Leg-4 Knee Velocity Command"," Leg-4 Knee Tau FF Command"]   

    
    writerLegStates.writerow(Headers) 
else:
    fileLegStates  = open('../lcm_logs/LegStatesLog', 'a',newline='')
    writerLegStates = csv.writer(fileLegStates)








def my_handler1(channel, data):
    global LogListLegStates
    LogListLegStates = []
    LegStates=leg_control_data_lcmt.decode(data)
    

    # print("Received message on channel \"%s\"" % channel)
    print ("States: \n")
 
    for leg in range(4):
        for axis in range(3):
            idx = (leg)*3+(axis)
        # print("   Commands.tau_abad_ff- " + str(i+1) + " = %s" % str(Commands.tau_abad_ff[i]))
        # print("   Commands.tau_hip_ff - " + str(i+1) + " = %s" % str(Commands.tau_hip_ff[i] ))
        # print("   Commands.tau_knee_ff- " + str(i+1) + " = %s" % str(Commands.tau_knee_ff[i]))
        # print("   qd_des_abad of LEG - " + str(i+1) + " = %s" % str(Commands.qd_des_abad[i]))
        # print("   qd_des_hip of LEG - " + str(i+1) + " = %s" % str(Commands.qd_des_hip[i]))
        # print("   qd_des_knee of LEG - " + str(i+1) + " = %s" % str(Commands.qd_des_knee[i]))

            # print(" Foot pos " + str(idx) + " = %s" % str(LegStates.foot_pos[idx]))


            LogListLegStates = LogListLegStates + [LegStates.q[idx]*180/math.pi,LegStates.qd[idx],LegStates.tau_est[idx],LegStates.p[idx],LegStates.grf[idx],LegStates.ForceEstimate[idx]]
      
    
   
      
def my_handler2(channel, data):
    global LogListLegStates
    LegStates = leg_control_command_lcmt.decode(data)
    # print("Received message on channel \"%s\"" % channel)
    print ("FeedBacks: \n")
    
    for leg in range(4):
        for axis in range(3):
            idx = (leg)*3+(axis)
        # print("   q_abad of LEG - " + str(i+1) + " = %s" % str(Data.q_abad[i]))
        # print("   q_hip of LEG - " + str(i+1) + " = %s" % str(Data.q_hip[i]))
        # print("   q_knee of LEG - " + str(i+1) + " = %s" % str(Data.q_knee[i]))
        # print("   qd_abad of LEG - " + str(i+1) + " = %s" % str(Data.qd_abad[i]))
        # print("   qd_hip of LEG - " + str(i+1) + " = %s" % str(Data.qd_hip[i]))
        # print("   qd_knee of LEG - " + str(i+1) + " = %s" % str(Data.qd_knee[i]))
            
        if LogListLegStates==[]:
            print("ERROR LISTENING")
            pass
        else:
            LogListLegStates = LogListLegStates + [180/math.pi*LegStates.q_des[idx],180/math.pi*LegStates.qd_des[idx],LegStates.tau_ff[idx]]
      
    
    writerLegStates.writerow(LogListLegStates)
    LogListLegStates.clear()
    
  

global LogListLegStates
LogListLegStates = []

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
subscription1 = lc.subscribe("leg_control_data", my_handler1)
subscription2 = lc.subscribe("leg_control_command", my_handler2)

try:
    while True:

        lc.handle()
except KeyboardInterrupt:
   pass
    

lc.unsubscribe(subscription1)
#lc.unsubscribe(subscription2)
