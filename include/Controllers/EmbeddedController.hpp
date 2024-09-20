#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

#include <RobotController.h>
#include <Utilities/RobotCommands.h>

#include <Controllers/GaitScheduler.h>
// #include "Controllers/ContactEstimator.h"
#include <FSM/ControlFSM.h>
#include "ControlParameters/ControlParameters.h"
#include "ControlParameters/RobotParameters.h"
#include <ControlParameters/MIT_UserParameters.h>

//#include <gui_main_control_settings_t.hpp>

class EmbeddedController:public RobotController{
public:
  EmbeddedController();
  virtual ~EmbeddedController(){}

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}

  SpiData* _Feedback;
  VectorNavData* _ImuData;
  

  SpiCommand* _Command;
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }
  // virtual void Estop(){ _controlFSM->initialize(); }

  ControlFSM<float>* _controlFSM;

protected:
  // // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;
  MIT_UserParameters userParameters;
  float desiredBodyHeight=0;

};


#endif
