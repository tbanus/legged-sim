/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include <Bridge.h>
#include "Utilities/GamepadCommand.h"
#include "Utilities/PeriodicTask.h"
#include "state_estimator_lcmt.hpp"
#include "Rob2GuiComm.hpp"
#include "RobotController.h"
#include <lcm-cpp.hpp>
#include <IMUreader_vn100.h>
#include "Utilities/Timer.h"
#ifndef HW_ENABLE
  #include "Simulation/Simulation.h"
#endif


class RobotRunner : public PeriodicTask {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController* , PeriodicTaskManager*, float, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;
  void ReceiveLCM();
  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator();
  //check whether connecto to upper computer 
  void CheckConnection ();
  void KeyboardCallBack () ;
  void UpdateCommMessage();
  virtual ~RobotRunner();

  RobotController* _robot_ctrl;

  GamepadCommand* driverCommand;
  RobotControlParameters* controlParameters;
  Bridge* _bridge;
  
  #ifndef HW_ENABLE
    Simulation* _Sim;
  #endif  

  CANData* _CANData;
  CANcommand* _CANcommand;
  IMUData* vectorNavData;
  float _period;
  DesiredStateCommand<float>* _desiredStateCommand;

 
 private:
  float _ini_yaw;
  int ConnectionFailCount = 0;
  int iter = 0;
  int pulse_iter =0;
  Timer RobotTimer;
  void setupStep();
  void finalizeStep();
  

  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;


   lcm::LCM _lcm;
   leg_control_command_lcmt leg_control_command_lcm;
   state_estimator_lcmt state_estimator_lcm;
   leg_control_data_lcmt leg_control_data_lcm;
  // Contact Estimator to calculate estimated forces and contacts

  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
