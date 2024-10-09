
import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.wbc_data_lcmt import wbc_data_lcmt

if __name__=="__main__":
    msg_type=wbc_data_lcmt
    filename="wbc_data"+"_log"
    lcm_channel="wbc_lcm_data"
    listener= LcmListener(filename, lcm_channel, msg_type)
    listener.run()