
import sys
sys.path.append('../')

from lcm_listeners.LcmListener import LcmListener
from lcm_types.python.state_estimator_lcmt import state_estimator_lcmt

if __name__=="__main__":
    msg_type=state_estimator_lcmt
    filename="state_estimator_log"
    lcm_channel="state_estimator"
    listener= LcmListener(filename, lcm_channel, msg_type)
    listener.run()