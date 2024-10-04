#include <EmbeddedController.hpp>
// #include <FSM/ControlFSM.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
EmbeddedController::EmbeddedController() : RobotController()
{
  // initializeController();
}

// #define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void EmbeddedController::initializeController()
{
  // OriKp << 20,20,20;
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);

  // // Initialize a new ContactEstimator object
  // //_contactEstimator = new ContactEstimator<double>();
  // ////_contactEstimator->initialize();

  // // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters,
                                      &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void EmbeddedController::runController()
{
  // // Find the current gait schedule
  // _gaitScheduler->step();

  // // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();
  _desiredStateCommand->gamepadCommand->start;
  
  // std::cout<<"_driverCommand->leftStickAnalog "<<_driverCommand->leftStickAnalog<<std::endl;

  // // Run the Control FSM code
  _controlFSM->runFSM();
}
