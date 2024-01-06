#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

#include <RobotController.h>
#include <Utilities/RobotCommands.h>

// #include "Controllers/GaitScheduler.h"
// #include "Controllers/ContactEstimator.h"
// #include "FSM_States/ControlFSM.h"
// #include "MIT_UserParameters.h"
//#include <gui_main_control_settings_t.hpp>

class MIT_Controller: public RobotController{
public:
  MIT_Controller();
  virtual ~MIT_Controller(){}

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  Vec3<float> OrientationControl();
  // SpiData* _Feedback;
  // VectorNavData* _ImuData;
  Vec3<float> OriEuler;
  Vec3<float> OriEulerDes;
  Vec3<float> OriCmd;
  Vec3<float> OriKp;

  Eigen::Matrix<float,4,1> OriQuat;
  Vec4<float> leg_cmd;
      int kDirSign_[4]={1,1,-1,-1};
    int kSideSign_[4]={-1,1,-1,1};

  // SpiCommand* _Command;
  // virtual ControlParameters* getUserControlParameters() {
  //   return &userParameters;
  // }
  // virtual void Estop(){ _controlFSM->initialize(); }


protected:
  // ControlFSM<float>* _controlFSM;
  // // Gait Scheduler controls the nominal contact schedule for the feet
  // GaitScheduler<float>* _gaitScheduler;
  // MIT_UserParameters userParameters;
  float desiredBodyHeight=0;

};


#endif
