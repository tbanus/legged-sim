import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.state_est_debug_lcmt import state_est_debug_lcmt

if __name__=="__main__":
    msg_type=state_est_debug_lcmt
    filename="state_est_debug"+"_log"
    lcm_channel="state_est_debug"
    listener= LcmListener(filename, lcm_channel, msg_type)
    listener.run()