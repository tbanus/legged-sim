/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>

#include <RobotRunner.h>
#include "Controllers/Estimators/ContactEstimator.h"
#include "Controllers/Estimators/OrientationEstimator.h"
// #include "Dynamics/Cheetah3.h"
// #include "Dynamics/MiniCheetah.h"
#include "Utilities/Utilities_print.h"
// #include "ParamHandler.hpp"
// #include "Utilities/Timer.h"
#include "Controllers/Estimators/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"
#ifdef MANUAL
#include <ReceiveLCMMessage.h>
#endif
RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name)
   {

    _robot_ctrl = robot_ctrl;
    _period = period;
    // _robot_ctrl->_controlFSM->currentState->onEnter();

  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  #ifdef MANUAL
    printf("ROBOT_RUNNER INIT");

    std::string mjcf_file;
    mjcf_file = std::string("../resource/")+std::string("opy_v05/opy_v05.xml");
    _Sim = new Simulation(mjcf_file); 
    _Sim->motor_input_type_=1;
    _Sim->SetCommand(_Command);
    _Sim->SetFeedback(_Feedback,_ImuData);
    _Sim->SetTimestep(_period);
    _Sim->RunOnce();
    #endif
  // printf("[RobotRunner] initialize\n");

  // // Build the appropriate Quadruped object
  // if (robotType == RobotType::MINI_CHEETAH) {
  //   _quadruped = buildMiniCheetah<float>();
  // } else {
  //   _quadruped = buildCheetah3<float>();
  // }
  #ifndef MANUAL
    _quadruped=ParseURDFtoQuadruped<float>(std::string("/app/legged-sim/resource/opy_v05/opy_v05.urdf"),RobotType::MINI_CHEETAH);
  #else 
    _quadruped=ParseURDFtoQuadruped<float>(std::string("../resource/opy_v05/opy_v05.urdf"),RobotType::MINI_CHEETAH);
  #endif
  // // Initialize the model and robot data
  _model = _quadruped.buildModel();
  // _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);

  // // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      _ImuData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(true);

  // memset(&rc_control, 0, sizeof(rc_control_settings));
  // // Initialize the DesiredStateCommand object
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,
        
        &_stateEstimate,
        controlParameters->controller_dt);

  // // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_ImuData=_ImuData;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  // _robot_ctrl->_visualizationData= visualizationData;
  _robot_ctrl->_robotType = robotType;
  _robot_ctrl->_driverCommand = driverCommand;
  _robot_ctrl->_controlParameters = controlParameters;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

  _robot_ctrl->initializeController();


}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {

  // Run the state estimator step
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  // //cheetahMainVisualization->p = _stateEstimate.position;
  // visualizationData->clear();

  // // Update the data from the robot
  // setupStep();

  // static int count_ini(0);
  // ++count_ini;
  // if (count_ini < 10) {
  //   _legController->setEnabled(false);
  // } else if (20 < count_ini && count_ini < 30) {
  //   _legController->setEnabled(false);
  // } else if (40 < count_ini && count_ini < 50) {
  //   _legController->setEnabled(false);
  // } else {
  //   _legController->setEnabled(true);

  //   if( (rc_control.mode == 0) && controlParameters->use_rc ) {
  //     if(count_ini%1000 ==0)   printf("ESTOP!\n");
  //     for (int leg = 0; leg < 4; leg++) {
  //       _legController->commands[leg].zero();
  //     }
  //     _robot_ctrl->Estop();
  //   }else {
  //     // Controller
  //     if (!_jpos_initializer->IsInitialized(_legController)) {
  //       Mat3<float> kpMat;
  //       Mat3<float> kdMat;
  //       // Update the jpos feedback gains
  //       if (robotType == RobotType::MINI_CHEETAH) {
  //         kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
  //         kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
  //       } else if (robotType == RobotType::CHEETAH_3) {
  //         kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
  //         kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  //       } else {
  //         assert(false);
  //       } 

  //       for (int leg = 0; leg < 4; leg++) {
  //         _legController->commands[leg].kpJoint = kpMat;
  //         _legController->commands[leg].kdJoint = kdMat;
  //       }
  //     } else {
  //       // Run Control 
        _legController->updateData(_Feedback);



        _robot_ctrl->runController();
        _legController->updateCommand(_Command);
        // std::cout << "[DEBUG] [RobotRunner.cpp] [RobotRunner::run] Updating command" << std::endl;
  //       cheetahMainVisualization->p = _stateEstimate.position;

  //       // Update Visualization
  //       _robot_ctrl->updateVisualization();
  //       cheetahMainVisualization->p = _stateEstimate.position;
  //     }
  //   }

  // }



  // // Visualization (will make this into a separate function later)
  // for (int leg = 0; leg < 4; leg++) {
  //   for (int joint = 0; joint < 3; joint++) {
  //     cheetahMainVisualization->q[leg * 3 + joint] =
  //       _legController->datas[leg].q[joint];
  //   }
  // }
  // cheetahMainVisualization->p.setZero();
  // cheetahMainVisualization->p = _stateEstimate.position;
  // cheetahMainVisualization->quat = _stateEstimate.orientation;

  // // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
  #ifdef MANUAL
  _Sim->RunOnce();
  #endif
  
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  // Update the leg data
  // if (robotType == RobotType::MINI_CHEETAH) {
    // _legController->updateData(_Feedback);
  // } else if (robotType == RobotType::CHEETAH_3) {
  //   _legController->updateData(tiBoardData);
  // } else {
  //   assert(false);
  // }

  // // Setup the leg controller for a new iteration
  // _legController->zeroCommand();
  // _legController->setEnabled(true);
  // _legController->setMaxTorqueCheetah3(208.5);

  // // state estimator
  // // check transition to cheater mode:
  // if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
  //   printf("[RobotRunner] Transitioning to Cheater Mode...\n");
  //   initializeStateEstimator(true);
  //   // todo any configuration
  //   _cheaterModeEnabled = true;
  // }

  // // check transition from cheater mode:
  // if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
  //   printf("[RobotRunner] Transitioning from Cheater Mode...\n");
  //   initializeStateEstimator(false);
  //   // todo any configuration
  //   _cheaterModeEnabled = false;
  // }

  // get_rc_control_settings(&rc_control);

  // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
//   if (robotType == RobotType::MINI_CHEETAH) {
    // _legController->updateCommand(Command);
//   } else if (robotType == RobotType::CHEETAH_3) {
//     _legController->updateCommand(tiBoardCommand);
//   } else {
//     assert(false);
//   }
#ifdef LCM
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimate.setLcm(state_estimator_lcm);
  _lcm.publish("leg_control_command", &leg_control_command_lcm);
  _lcm.publish("leg_control_data", &leg_control_data_lcm);
  _lcm.publish("state_estimator", &state_estimator_lcm);
//   _iterations++;
#endif
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  
}
// #define THIS_COM "/home/banuslegged-sim/"
void RobotRunner::initializeParameters()
{
    bool _load_parameters_from_file =1 ;
    // std::cout<<controlParameters<<std::endl;
    #ifndef MANUAL
  	std::string robotParametersPath = "/app/legged-sim/resource/opy_v05/mc-mit-ctrl-user-parameters.yaml";
    std::string userParametersPath = "/app/legged-sim/resource/opy_v05/mini-cheetah-defaults.yaml";
    #else
  	std::string robotParametersPath = "../resource/opy_v05/mc-mit-ctrl-user-parameters.yaml";
    std::string userParametersPath = "../resource/opy_v05/mini-cheetah-defaults.yaml";
    #endif
  	// userParameters.initializeFromYamlFile(path);
  	// printf("controlParameters: %d", controlParameters);
   	if(_load_parameters_from_file) {
    // printf("[Hardware Bridge] Loading parameters from file...\n");
    
    try {
      controlParameters->initializeFromYamlFile(userParametersPath);
    } catch(std::exception& e) {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if(!controlParameters->isFullyInitialized()) {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }

    if(_robot_ctrl->getUserControlParameters()) {
      try {
        _robot_ctrl->getUserControlParameters()->initializeFromYamlFile( robotParametersPath);
      } catch(std::exception& e) {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
      }

      if(!_robot_ctrl->getUserControlParameters()->isFullyInitialized()) {
        printf("Failed to initialize all user parameters\n");
        exit(1);
      }

      // printf("Loaded user parameters\n");
    } else {
      printf("Did not load user parameters because there aren't any\n");
    }
  } else {
    printf("[Hardware Bridge] Loading parameters over LCM...\n");
    while (!controlParameters->isFullyInitialized()) {
      printf("[Hardware Bridge] Waiting for robot parameters...\n");
      usleep(1000000);
    }

    if(_robot_ctrl->getUserControlParameters()) {
      while (!_robot_ctrl->getUserControlParameters()->isFullyInitialized()) {
        printf("[Hardware Bridge] Waiting for user parameters...\n");
        usleep(1000000);
      }
    }
  }



  // printf("[Hardware Bridge] Got all parameters, starting up!\n");
}
#ifdef MANUAL
void RobotRunner::ReceiveLCM()

{
RecieveLCMGamepadCommand (driverCommand);
}
#endif
RobotRunner::~RobotRunner() {
  // delete _legController;
  // delete _stateEstimator;
  // delete _jpos_initializer;
}

void RobotRunner::cleanup() {


}
void RobotRunner::reset() {
StateEstimate<float> stateEstimate;
_stateEstimate = stateEstimate;
for(int leg=0; leg<4; leg++){
  _legController->datas[leg].zero();
  _legController->commands[leg].zero();
}

}

