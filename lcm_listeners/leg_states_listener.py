
import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.leg_control_data_lcmt import leg_control_data_lcmt
from lcm_types.python.leg_control_command_lcmt import leg_control_command_lcmt

if __name__=="__main__":
   
    msg_type=leg_control_data_lcmt
    filename="leg_control_states"+"_log"
    lcm_channel="leg_control_data"
    lcm_channel2="leg_control_command"
    msg_type2=leg_control_command_lcmt  
    listener= LcmListener(filename, lcm_channel, msg_type, lcm_channel2, msg_type2)
    listener.run()