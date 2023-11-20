#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MotorCMD.h"
#include "IMUreader_vn100.h"

#ifndef SIMULATION_H 

#define SIMULATION_H 



class Simulation 
{
    public:
    // Constructor
    Simulation(std::string filename);
    ~Simulation();

    //public variables
    int motor_input_type_; // 1: Only Torque, 2: Position, Velocity and Torque. Use actuator model
    GLFWwindow* window_;
    

    // public methods
    void RunOnce();
    
    void SetCanCommand(CANcommand* cmd);
    void SetCanData(CANData* feedback,IMUData* bodyIMU);
    void SetTimestep(double ts);
    bool CheckWindowShouldClose();cd
    void UpdateScene();
    void ShowContactForce(bool enable);

      
    private:

    float tau_abad_[4];
    float tau_hip_[4];
    float tau_knee_[4];
    mjtNum last_update_;
    CANcommand* cmd_;
    CANData* feedback_;
    IMUData* bodyIMU;

    int kAbad_ =0;
    int kHip_ =1;
    int kKnee_ =2; 
    int kDirSign_[4]={1,1,1,1};
    int kSideSign_[4]={-1,1,-1,1};

    //priv methods
    void SetInitialConditions();
    void UpdateCan();
    void StepSim();

};

#endif  

