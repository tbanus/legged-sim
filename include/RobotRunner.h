/*!
 * @file RobotRunner.h
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah and cheetah 3
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "ControlParameters/MIT_UserParameters.h"
// #include "Controllers/StateEstimatorContainer.h"
// #include "SimUtilities/IMUTypes.h"
// #include "rt/rt_rc_interface.h"
// #include "Controllers/ContactEstimator.h"
// #include "Controllers/DesiredStateCommand.h"
// #include "Controllers/LegController.h"
// #include "Dynamics/Quadruped.h"
// #include "JPosInitializer.h"

// #include "SimUtilities/GamepadCommand.h"
// #include "SimUtilities/VisualizationData.h"
#include "Utilities/PeriodicTask.h"
#include <Utilities/RobotCommands.h>
#include "Utilities/utilities.h"
#include <Simulation/Simulation.h>
#include <Dynamics/ParseURDFtoQuadruped.h>
// #include "cheetah_visualization_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "RobotController.h"
#include <lcm-cpp.hpp>
#include  <eigen3/Eigen/Dense>

class RobotRunner : public PeriodicTask {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotRunner(RobotController* robot_ctrl , PeriodicTaskManager*, float, std::string);
  using PeriodicTask::PeriodicTask;
  void init() override;
  void run() override;
  void cleanup() override;

  // Initialize the state estimator with default no cheaterMode
  void initializeStateEstimator(bool cheaterMode = false);
  virtual ~RobotRunner();
  void ReceiveLCM();

  RobotController* _robot_ctrl;

  GamepadCommand* driverCommand;
  RobotType robotType;
  // VectorNavData* vectorNavData;
  // CheaterState<double>* cheaterState;
  SpiData* _Feedback;
  VectorNavData* _ImuData;
  SpiCommand* _Command;
  
  Simulation* _Sim;
  float _period;

  // TiBoardCommand* tiBoardCommand;
  // TiBoardData* tiBoardData;
  RobotControlParameters* controlParameters;
  
  // VisualizationData* visualizationData;
  // CheetahVisualization* cheetahMainVisualization;
  void initializeParameters();

 private:
  float _ini_yaw;

  int iter = 0;

  void setupStep();
  void finalizeStep();

  // JPosInitializer<float>* _jpos_initializer;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  // bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;
  // rc_control_settings rc_control;
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;
  state_estimator_lcmt state_estimator_lcm;
  leg_control_data_lcmt leg_control_data_lcm;
  // // Contact Estimator to calculate estimated forces and contacts

  FloatingBaseModel<float> _model;
  // u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
