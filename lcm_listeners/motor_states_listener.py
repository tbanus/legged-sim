
import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.can_command_lcmt import can_command_lcmt
from lcm_types.python.can_data_lcmt import can_data_lcmt
if __name__=="__main__":
    msg_type=can_command_lcmt
    filename="motor_states"+"_log"
    lcm_channel="can_command"
    msg_type2=can_data_lcmt
    lcm_channel2="can_data"
    listener= LcmListener(filename, lcm_channel, msg_type, lcm_channel2, msg_type2)
    listener.run()