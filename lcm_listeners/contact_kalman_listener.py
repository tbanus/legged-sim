
import sys
sys.path.append('../')
from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.contact_kalman_lcmt import contact_kalman_lcmt

if __name__=="__main__":
    msg_type=contact_kalman_lcmt
    filename="contact_kalman"+"_log"
    lcm_channel="contact_kalman"
    listener= LcmListener(filename, lcm_channel, msg_type)
    listener.run()