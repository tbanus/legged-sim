#include <sys/mman.h>
#include <glog/logging.h>
#include <RobotRunner.h>
// #include <Controllers/SingleLegController.h>
// #include <send_message.h>
// #include <Dynamics/OPYModel.h>
// #include "FSM/Stand_FSM.cpp"
// #include "FSM/FSM_State_Passive.cpp"
// #include "FSM/ControlFSM.cpp"
// #include "GamepadCommand.h"
// #include "LCM/RecieveLCMMessage.h"
// #include "Controllers/DesiredStateCommand.h"
#include <RobotController.h>
#include "EmbeddedController.hpp"
#include "Utilities/PeriodicTask.h"
#include <Utilities/RobotCommands.h>
#include <RobotRunner.h>
// #include <IMUreader_vn100.h>
// #include "Controllers/ControlParameters/ControlParameters.h"
// #include "Controllers/ControlParameters/RobotParameters.h"
// #include <Controllers/MIT_UserParameters.h>
// #include <Controllers/DesiredStateCommand.h>
// #include <Bridge.h>
// #include "Simulation/SimPeriodicTask.h"

//#include "Utilities/utilities.h"
// #include "dma_latency_trick.h"

#define MAX_STACK_SIZE 24576 // 16KB  of stack
#define TASK_PRIORITY 99 // linux priority, this is not the nice value



int main(int argc, char* argv[]){
  // google::SetStderrLogging(0);
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // google::SetLogDestination(0,"~/LOG_TRY");
  LOG(INFO) << "[Init] "<<"Prefault stack.......";

  // volatile char stack[MAX_STACK_SIZE];
  // memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  // if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
  //   LOG(ERROR)<< "[ERROR]! " << "mlockall failed.";
  // }
  // /* use the /dev/cpu_dma_latency trick if it's there */
  //   set_latency_target();
  // LOG(INFO)<<"[Init] "<< "Setup RT Scheduler...";
  


  // struct sched_param params;

  // params.sched_priority = TASK_PRIORITY;

  // if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
  //   LOG(ERROR) << "[ERROR]! " <<"sched_setscheduler failed.";
  // }



  PeriodicTaskManager taskManager;
  if (! &taskManager){LOG(FATAL)<<"Cannot initalize taskManager";}

  SpiCommand _Command;
  if (! &_Command){LOG(FATAL)<<"Cannot initalize _Command";}

  SpiData _Feedback;
  if (! &_Feedback){LOG(FATAL)<<"Cannot initalize _Feedback";}

  GamepadCommand _GamepadCommand;
  if (! &_GamepadCommand){LOG(FATAL)<<"Cannot initalize _GamepadCommand";}

  VectorNavData _ImuData;
  if (! &_ImuData){LOG(FATAL)<<"Cannot initalize ImuData";}

  RobotControlParameters _robotParams;
  if (! &_robotParams){LOG(FATAL)<<"Cannot initalize _robotParams";}


  // usleep(1e6);
  // #ifdef HW_ENABLE
  //   LOG(INFO)<<"HW MODE, Initializing SLCs"; 
  //   InitializeSLCs(&_Feedback,&_Command,&taskManager);
  // #endif

  // // Create Ori_Controller instance
  RobotController* _ctrl = new EmbeddedController();
  LOG(INFO)<<"Lez GOOO!!!!";
  if (!_ctrl){LOG(FATAL)<<"Cannot initalize RobotController";}
  // _ctrl->_Command=_Command;
  // #ifdef HW_ENABLE
   
  //   IMUReaderVN100 _IMUReader(&_vectorNavData,&taskManager, .00125f, "IMU Read");
  //   LOG(INFO)<<"HW MODE, Initializing IMU"; 

  // #else
  //   LOG(INFO)<<"[Init] Initializing in Sim Mode"; 
  // #endif

  RobotRunner* _robotRunner = new RobotRunner(_ctrl, &taskManager, 0.002, "robot-control");
  if (!_robotRunner){
    LOG(FATAL)<<"Cannot initalize RobotRunner";
    }
  PeriodicMemberFunction<RobotRunner> RR_Gamepad(&taskManager,0.00125,"RR-GamePad",&RobotRunner::ReceiveLCM,_robotRunner);

  // // Assign the parameters of the robotrunner object. 
  // // These are the real physical variables such as CAN addresses.

  _robotRunner->driverCommand = &_GamepadCommand;
  _robotRunner-> _ImuData= &_ImuData;
  _robotRunner->_Feedback = &_Feedback;
  _robotRunner->_Command = &_Command;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->initializeParameters();
  
  // // auto* param = _ctrl->getUserControlParameters();

  // Bridge* _bridge = new Bridge(_ctrl, &_robotParams);
  // _robotRunner->_bridge=_bridge;
  
  // std::thread bridgeLcmThread= std::thread(&Bridge::LcmThread,_bridge);

  LOG(INFO)<<"[Init] "<<"Start Periodic tasks.";
  // //GamePad Command periodic task to listen related LCM channel
  // PeriodicMemberFunction<RobotRunner> RR_Gamepad(&taskManager,0.0125,"RR-GamePad",&RobotRunner::ReceiveLCM,_robotRunner);
  // //Check Connection Periodic tast to ping upper computer periodicaly
  // //PeriodicMemberFunction<RobotRunner> CheckConnection(&taskManager,0.1,"Check Connection",&RobotRunner::CheckConnection,_robotRunner);
  // //PeriodicMemberFunction<RobotRunner> KeyboardCommand(&taskManager,0.1,"Keyboard Command",&RobotRunner::KeyboardCallBack,_robotRunner);

  // usleep(1e6);

  // //IMUReaderVN100 _IMUReader(&_vectorNavData,&taskManager, .00125f, "IMU Read");
  
   
 
  // #ifdef HW_ENABLE
   
  //   if (_IMUReader.ez->sensor()==NULL){
  //   LOG(ERROR)<<"IMU Cannot connect";
  //   }else {_IMUReader.start();} 
  //   //CheckConnection.start();
  // #endif
  
  _robotRunner->start();
  RR_Gamepad.start();
  // // KeyboardCommand.start();
  // usleep(1e6);

  float i = 1 ;
  while (i) { 
    // #ifndef HW_ENABLE
      _robotRunner->_Sim->UpdateScene();
      usleep(1e6/60);
  //   #else
  //      usleep(1e6);
  //   #endif

    
    // taskManager.printStatus();
  };

  // LOG(WARNING)<<"[EXIT]"<<"Exited main loop. Stopping all tasks";
  // taskManager.stopAll();
  // LOG(WARNING)<<"[EXIT]"<<"Tasks stopped. Close program.";

}
