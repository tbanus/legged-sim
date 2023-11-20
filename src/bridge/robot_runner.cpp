/*!
 * @file RobotRunner.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#include <unistd.h>
#include "robot_runner/robot_runner.h"
// #include "Controllers/ContactEstimator.h"
// #include "Controllers/OrientationEstimator.h"
// #include "Dynamics/MiniCheetah.h"
// #include "Utilities/Utilities_print.h"
// #include "Utilities/Timer.h"
// #include "Controllers/PositionVelocityEstimator.h"
// #include "Dynamics/OPYModel.h"
// #include "LCM/RecieveLCMMessage.h"
#include <glog/logging.h>

float dt = 0.002;
RobotRunner::RobotRunner(RobotController* robot_ctrl, 
    PeriodicTaskManager* manager, 
    float period, std::string name):
  PeriodicTask(manager, period, name),
  _lcm(getLcmUrl(255)) 
  {
    _period = period;
    _robot_ctrl = robot_ctrl;
  }

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  LOG(INFO)<<"[RobotRunner] "<< "Initialize";

  // Build the appropriate Quadruped object
  // _quadruped = buildMiniCheetah<float>();

  RobotTimer.start();
  _quadruped = buildOPYModel<float>();
  

  // Initialize the model and robot data
  _model = _quadruped.buildModel();

  DVLOG(3)<<"[RobotRunner] "<< "Built Model";

  // Always initialize the leg controller and state estimator
  _legController = new LegController<float>(_quadruped);
  DVLOG(3)<<"[RobotRunner] "<< "Built _legController";
  //_legController.datas->DesHMax=0.32;

  _stateEstimator = new StateEstimatorContainer<float>(vectorNavData, _legController->datas,
      &_stateEstimate);

  DVLOG(3)<<"[RobotRunner] "<< "Built _stateEstimator";
      
  initializeStateEstimator();



  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
    new DesiredStateCommand<float>(driverCommand,&_stateEstimate,
    dt);

  //#TODO wdym dt?
  DVLOG(3)<<"[RobotRunner] "<< "Built _desiredStateCommand";

  //Initialize Mujoco 
  #ifndef HW_ENABLE
    //DLOG(INFO)<<"[RobotRunner] "<< "Initialize Simulation";
    std::string mjcf_file;
    
    mjcf_file = std::string("../resource/")+std::string("v00/comar.xml");
    _Sim = new Simulation(mjcf_file); 
  #endif 
    usleep (5000);
  // Controller initializations
  _robot_ctrl->_model = &_model;
  _robot_ctrl->_quadruped = &_quadruped;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;
  _robot_ctrl->_stateEstimate = &_stateEstimate;
  _robot_ctrl->_desiredStateCommand = _desiredStateCommand;
  _robot_ctrl->_controlParameters = controlParameters;


  //Set Sim Properties
  #ifndef HW_ENABLE
    //DVLOG(3)<<("Set Sim Properties");
    _Sim->motor_input_type_=1;
    _Sim->SetCanCommand(_CANcommand);
    _Sim->SetCanData(_CANData,vectorNavData);
    _Sim->SetTimestep(_period);
    _Sim->RunOnce();
   #endif 
  // Update once for the first configuration.
  _legController->updateData(_CANData);
  _robot_ctrl->initializeController();
  _legController->setEnabled(true);
  _legController->limits[0]=controlParameters->collection.lookup("abad_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[0];
  _legController->limits[1]=controlParameters->collection.lookup("abad_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[1];
  _legController->limits[2]=controlParameters->collection.lookup("hip_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[0];
  _legController->limits[3]=controlParameters->collection.lookup("hip_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[1];
  _legController->limits[4]=controlParameters->collection.lookup("knee_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[0];
  _legController->limits[5]=controlParameters->collection.lookup("knee_limits").get(ControlParameterValueKind::VEC3_DOUBLE).vec3d[1];

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Run the state estimator step

  LOG_FIRST_N(INFO, 1) << "[RobotRunner] " << "First run";
  //_stateEstimator->run(cheetahMainVisualization);
  _stateEstimator->run();
  //std::cout<<"estimate pos: "<<_stateEstimate.position<<std::endl;
  //std::cout<<"estimate ori: "<<_stateEstimate.rpy*180/M_PI<<std::endl;
  // Update the data from the robot
  setupStep();

  // Burası tam olarak ne yapıyor anlamadım. setenabled fonksiyonu legsenabled 
  // değişkenini değiştiriyor ancak nerede kullanıldığını göremedim.
  // count_ini değişkeni statik olduğu için ilk girişte 50 ye kadar sayıyor, 
  // sonrasında aynı kalıyor. Tekrar runController yapmak için saymamız gerekmiyor.
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
  // //   // Run Control 
  // //   // Kontrolcü burada çalışıyor !!!!
    
  //  }
  
 
 
  _robot_ctrl->runController();
  
  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();

  if (pulse_iter > 500){
    _bridge->UpdatePulseMsg(_legController->_legsEnabled, _CANData->can_status);
    pulse_iter=0;
  }
  pulse_iter++;
  DVLOG(6)<<"[RobotRunner] "<< " Complete run";
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner::setupStep() {
  DVLOG(6)<<"[RobotRunner] "<< "setupStep";
  // Update the leg data 
  _legController->updateData(_CANData);
  //std::cout<<"Leg controller q: "<<_legController->datas[3].q<<std::endl;
  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  //_legController->setEnabled(true);

  //_legController->setMaxTorqueCheetah3(208.5);

  // state estimator

//   // check transition to cheater mode:
//   if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
//     printf("[RobotRunner] Transitioning to Cheater Mode...\n");
//     initializeStateEstimator(true);
//     // todo any configuration
//     _cheaterModeEnabled = true;
//   }

//   // check transition from cheater mode:
//   if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
//     printf("[RobotRunner] Transitioning from Cheater Mode...\n");
//     initializeStateEstimator(false);
//     // todo any configuration
//     _cheaterModeEnabled = false;
//   }

//   get_rc_control_settings(&rc_control);

//   // todo safety checks, sanity checks, etc...
 }

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner::finalizeStep() {
  if (_legController->_legsEnabled){
   _legController->updateCommand(_CANcommand);
  }
  else if(!_legController->_legsEnabled)
  {
    LOG_EVERY_N(ERROR, 1500)<<"[RobotRunner] "<<"Legs are disabled";
    _legController->edampCommand(RobotType::MINI_CHEETAH, 5);
     _legController->updateCommand(_CANcommand);
  }
    #ifndef HW_ENABLE
    _Sim->RunOnce();
    #endif

   _legController->SendLCMLeg (_CANData,_CANcommand);
  //puts("Before LCM");
 
  //_desiredStateCommand->printStateCommandInfo();
  
 

  auto current_time = RobotTimer.getSeconds();
  _CANcommand->time = current_time;
  _stateEstimate.setLcm(state_estimator_lcm);
  //_lcm.publish("CAN Commands", &_CANcommand);
  //_lcm.publish("CAN Data", &_CANData);
  _lcm.publish("state_estimator", &state_estimator_lcm);
   //TODO Fix this @tarik
  if(_stateEstimator->_data.result->StandUp ){
      _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
    for(int leg = 0; leg < 4; leg++) {
          for(int axis = 0; axis < 3; axis++) {
              int idx = leg*3 + axis;
              leg_control_data_lcm.foot_pos[idx] = _stateEstimator->_data.vectorNavData->foot_pos[idx];
              leg_control_data_lcm.grf[idx] = _stateEstimator->_data.vectorNavData->grf[idx];
        
          }
    }

    _lcm.publish("leg_control_command", &leg_control_command_lcm);
    _lcm.publish("leg_control_data", &leg_control_data_lcm);
  }
  _iterations++;
}

void RobotRunner::ReceiveLCM()

{
RecieveLCMGamepadCommand (driverCommand);
}




/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner::initializeStateEstimator() {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5;
  _stateEstimator->setContactPhase(contactDefault);

  _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
  
  DVLOG(3)<<"[RobotRunner] "<< "initialized StateEstimator";
}

void RobotRunner::CheckConnection (){
        std::string ipAddress = "192.168.1.34" ;
        if (ping(ipAddress))
          {
          DVLOG(5)<<"[RobotRunner] "<< "Ping Succesfull";
            //CheckConnectionOK = true;
          ConnectionFailCount = 0;
            _robot_ctrl->_stateEstimate->CheckConnectionOK = true;
          }
        else
          {LOG_EVERY_N(ERROR, 35)<<"[RobotRunner] " << "Ping failed." ;
           //CheckConnectionOK = false;
           ConnectionFailCount++;
           DVLOG(5)<<"[RobotRunner] "<< "Connection fail counter: "<<ConnectionFailCount;
           if (ConnectionFailCount >5) 
            {_robot_ctrl->_stateEstimate->CheckConnectionOK  = false;}
          }
          

}
void RobotRunner::KeyboardCallBack () {
_desiredStateCommand->GetFromKeyboard();
}

RobotRunner::~RobotRunner() {
  LOG(WARNING)<<"[EXIT]"<<" Robot Runner Closing";

  delete _legController;
  delete _stateEstimator;
}

void RobotRunner::cleanup() {}
