
import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.debug_data_lcmt import debug_data_lcmt

if __name__=="__main__":
    msg_type=debug_data_lcmt
    filename="debug_data"+"_log"
    lcm_channel="debug_data"
    listener= LcmListener(filename, lcm_channel, msg_type)
    listener.run()