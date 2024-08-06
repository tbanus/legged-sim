#include <Controllers/Ori_Controller.hpp>
// #include <FSM/ControlFSM.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
Ori_Controller::Ori_Controller():RobotController(){ initializeController(); }

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void Ori_Controller::initializeController() {
  OriKp << 20,20,20;
  // Initialize a new GaitScheduler object
  // _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // // Initialize a new ContactEstimator object
  // //_contactEstimator = new ContactEstimator<double>();
  // ////_contactEstimator->initialize();

  // // Initializes the Control FSM with all the required data
  // _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
  //                                     _legController, 
  //                                     _desiredStateCommand);
  
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void Ori_Controller::runController() {
  // // Find the current gait schedule
  // _gaitScheduler->step();

  // // Find the desired state trajectory
  // _desiredStateCommand->convertToStateCommands();

  // // Run the Control FSM code
  // _controlFSM->runFSM();
  if(desiredBodyHeight<0.2){
    desiredBodyHeight+=0.0005;
  }
  OrientationControl();
  OriQuat<< _ImuData->quat[0],_ImuData->quat[1],_ImuData->quat[2],_ImuData->quat[3];
  
  // std::cout<<"OriEuler"<<OriEuler<<std::endl;
  OriEuler=ori::quatToRPY(OriQuat);

  
  // OriKPY =
  // std::cout<<"controller "<< OrientationControl()<<std::endl;
  // std::cout<<"end "<< std::endl;
  for(int leg=0; leg<4; leg++){
    
  _legController->commands[leg].qDes(0)=0;
  _legController->commands[leg].qDes(1)=-M_PI_4;
  _legController->commands[leg].qDes(2)=M_PI_2;
  
  _legController->commands[leg].qdDes(0)=0;
  _legController->commands[leg].qdDes(1)=0;
  _legController->commands[leg].qdDes(2)=0;
  
  _legController->commands[leg].kpJoint(0)=0;
  _legController->commands[leg].kpJoint(1)=0;
  _legController->commands[leg].kpJoint(2)=0;

  _legController->commands[leg].kpCartesian= Vec3<float>(0,0,0).asDiagonal();
  _legController->commands[leg].kdCartesian= Vec3<float>(0,0,0).asDiagonal();
  OriCmd=OrientationControl();
  _legController->commands[leg].forceFeedForward=Vec3<float>(0,-kDirSign_[leg]*OriCmd[2],-kSideSign_[leg]*OriCmd[0]+kDirSign_[leg]*OriCmd[1]);
  


  // _legController->commands[leg].kdJoint(0)=5;
  // _legController->commands[leg].kdJoint(1)=5;
  // _legController->commands[leg].kdJoint(2)=5;

  _legController->commands[leg].pDes(0)=0;
  _legController->commands[leg].pDes(1)=0.02;
  _legController->commands[leg].pDes(2)=-desiredBodyHeight;
  }


  
  
}


  Vec3<float> Ori_Controller::OrientationControl(){


    
    OriEulerDes<< -M_PI/6,-M_PI/6,-M_PI/6;
    return OriKp.asDiagonal()*(OriEulerDes-OriEuler);

    // float Cmd=Kp_Pitch*(pitchdes-pitch);
    // std::cout<<"cmd"<<Cmd<<std::endl;
    // leg_cmd << Cmd, Cmd, -Cmd, -Cmd;
        
    
  }