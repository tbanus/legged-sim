
#ifndef DEBUGGER_H
#define DEBUGGER_H


#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define DEBUG_MSG(msg) \
    std::cout << "[DEBUG] [" << __FILENAME__ << ":" << __LINE__ << "] ("<<__PRETTY_FUNCTION__<< ") - " << msg << std::endl;


#endif