#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
#include<math.h>
#include <iostream>
// #include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Simulation/Simulation.h>
#include <stdlib.h>
#include <glog/logging.h>
#define DEFAULT_TIMESTEP 0.0003


namespace mujoco
    {

    //This must adress the correct MJCF file before start.
    

    // MuJoCo data structures
    mjModel* m = NULL;                  // MuJoCo model
    mjData* d = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;

    // holders of one step history of time and position to calculate dertivatives
    mjtNum position_history = 0;
    mjtNum previous_time = 0;

    // controller related variables
    float_t ctrl_update_freq = 100;
    mjtNum last_update = 0.0;
    mjtNum ctrl;


    //glfw error callback
static void glfwError(int id, const char* description)
{
    LOG(ERROR)<<"[GLFW] "<<description;
}
double gain=10;
    // keyboard callback
    void keyboard(GLFWwindow* window_, int key, int scancode, int act, int mods)
    {
            double fx, fy, fz;
            fx = 0;
            fy = 0;
            fz = 0;

    
        
        
        if( act==GLFW_PRESS && key==GLFW_KEY_SPACE )
        {
        mujoco::cam.azimuth = 90;
        mujoco::cam.elevation = -45;
        mujoco::cam.distance = 4;
        mujoco::cam.lookat[0] = mujoco::d->qpos[0];
        mujoco::cam.lookat[1] = mujoco::d->qpos[1];
        mujoco::cam.lookat[2] = mujoco::d->qpos[2];
        }



        if( act==GLFW_PRESS && key==GLFW_KEY_W )
        {
            fx=gain;

        }


        if( act==GLFW_PRESS && key==GLFW_KEY_S)
        {
            fx=-gain;
        }

        if( act==GLFW_PRESS && key==GLFW_KEY_A )
        {
            fy=gain;
        }
        if( act==GLFW_PRESS && key==GLFW_KEY_D )
        {
            fy=-gain;
        }

        if( act==GLFW_PRESS && key==GLFW_KEY_R )
        {
            fz=gain;
        }
        if( act==GLFW_PRESS && key==GLFW_KEY_F )
        {
            fz=-gain;
        }

        if( act==GLFW_PRESS && key==GLFW_KEY_PAGE_DOWN )
        {
            gain+=-20;
        }

        if( act==GLFW_PRESS && key==GLFW_KEY_PAGE_UP )
        {
            gain+=20;
        }


        d->xfrc_applied[6]=fx;
        d->xfrc_applied[7]=fy;
        d->xfrc_applied[8]=fz;
        d->xfrc_applied[9]=0;
        d->xfrc_applied[10]=0;
        d->xfrc_applied[11]=0;

    }

    // mouse button callback
    void mouse_button(GLFWwindow* window_, int button, int act, int mods)
    {
        // update button state
        button_left =   (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
        button_right =  (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window_, &lastx, &lasty);
    }


    // mouse move callback
    void mouse_move(GLFWwindow* window_, double xpos, double ypos)
    {
        // no buttons down: nothing to do
        if( !button_left && !button_middle && !button_right )
            return;

        // compute mouse displacement, save
        double dx = xpos - lastx;
        double dy = ypos - lasty;
        lastx = xpos;
        lasty = ypos;

        // get current window_ size
        int width, height;
        glfwGetWindowSize(window_, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                        glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if( button_left )
            action = mod_shift ? mjMOUSE_MOVE_V : mjMOUSE_MOVE_H;
        else if( button_right )
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move camera
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
    }


    // scroll callback
    void scroll(GLFWwindow* window_, double xoffset, double yoffset)
    {
        // emulate vertical mouse motion = 5% of window_ height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
    }

    void control_callback(const mjModel* m, mjData* d){


    };
    
    
   
    }   //end mujoco namespace


/**
 * @brief Construct a new Simulation:: Simulation object
 * 
 * @param filename 
 */
Simulation::Simulation(std::string filename)

{
    
    
    
    DLOG(INFO)<<"[SIM] "<< "Begin Simulation";
    // load and compile model
    char error[100] = "Could not load binary model";
    mujoco::m = mj_loadXML(filename.c_str(), 0, error, 1000);

    if( !mujoco::m )
    {
        mju_error_s("Load model error: %s", error);
    }

    // make data
    mujoco::d = mj_makeData(mujoco::m);

    SetInitialConditions();
    
    mujoco::m->opt.timestep=DEFAULT_TIMESTEP;
    mj_step(mujoco::m, mujoco::d);
    
    mjcb_control = mujoco::control_callback;

     // init GLFW
    glfwSetErrorCallback(&mujoco::glfwError);
    if( !glfwInit() )
    {   
        mju_error("Could not initialize GLFW");
    }
    // create window_, make OpenGL context current, request v-sync
    DLOG(INFO)<<"[SIM]"<<"Create Sim Window";
    window_=glfwCreateWindow(1244, 700, " MuJoCo Simulation", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&mujoco::cam);
    mjv_defaultOption(&mujoco::opt);
    mjv_defaultScene(&mujoco::scn);
    mjr_defaultContext(&mujoco::con);
    mjv_makeScene(mujoco::m, &mujoco::scn, 2000);                // space for 2000 objects
    mjr_makeContext(mujoco::m, &(mujoco::con), mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window_, mujoco::keyboard);
    glfwSetCursorPosCallback(window_, mujoco::mouse_move);
    glfwSetMouseButtonCallback(window_, mujoco::mouse_button);
    glfwSetScrollCallback(window_, mujoco::scroll);

    double arr_view[] = {90,-45, 4, 0.000000, 0.000000, 0.000000};
    mujoco::cam.azimuth = arr_view[0];
    mujoco::cam.elevation = arr_view[1];
    mujoco::cam.distance = arr_view[2];
    mujoco::cam.lookat[0] = mujoco::d->qpos[0];
    mujoco::cam.lookat[1] = mujoco::d->qpos[1];
    mujoco::cam.lookat[2] = mujoco::d->qpos[2];
    last_update_ = mujoco::d->time;
    };

void Simulation::UpdateCan(){
/**
 * @brief Read from CAN Commands and send back CAN Data. Communication between CAN and Simulation Object. Update KP,KD 
 *  
 * 
 * 
 * 
 */ 

    mjtNum ContactForce5 [6] = {0};

            
    mjtNum ContactForce9 [6] = {0};

            
    mjtNum ContactForce13 [6] = {0};

            
    mjtNum ContactForce17 [6] = {0};


     int ContactStates[4] = { 0 };
    //std::cout<<"Number of Contacts "<<mujoco::d->ncon<<std::endl;
    for(int nc = 0; nc<mujoco::d->ncon; nc++){

       if (mujoco::d->contact[nc].geom2 == 6){
        ContactStates[0] = 1;
        mj_contactForce( mujoco::m,mujoco::d, nc,ContactForce5);
       }
       if (mujoco::d->contact[nc].geom2 == 10){
        ContactStates[1] = 1;
        mj_contactForce( mujoco::m,mujoco::d, nc,ContactForce9);   
       }
        if (mujoco::d->contact[nc].geom2 == 14){
        ContactStates[2] =1;
        mj_contactForce( mujoco::m,mujoco::d, nc,ContactForce13);   
       }
        if (mujoco::d->contact[nc].geom2 == 18){
        ContactStates[3] = 1;
        mj_contactForce( mujoco::m,mujoco::d, nc,ContactForce17);   
       }
    }

    bodyIMU->grf[0] = (float) ContactForce5[2];
    bodyIMU->grf[1] = (float) ContactForce5[1];
    bodyIMU->grf[2] = (float) ContactForce5[0];
   
    bodyIMU->grf[3] = (float) ContactForce9[2];
    bodyIMU->grf[4] = (float) ContactForce9[1];
    bodyIMU->grf[5] = (float) ContactForce9[0];

    bodyIMU->grf[6] = (float) ContactForce13[2];
    bodyIMU->grf[7] = (float) ContactForce13[1];
    bodyIMU->grf[8] = (float) ContactForce13[0];

    bodyIMU->grf[9] = (float) ContactForce17[2];
    bodyIMU->grf[10] = (float) ContactForce17[1];
    bodyIMU->grf[11] = (float) ContactForce17[0];


    for(int leg=0; leg<4; leg++){
     //std::cout<<leg<< " Contact States Sim :  "<<ContactStates[leg]<<std::endl;
        bodyIMU->ContactStatus[leg]= ContactStates[leg];
        feedback_->q_abad [leg]  =   mujoco::d->qpos[(leg)*3+    kAbad_    +7];  // Add 7 to skip the first 7 dofs from body. (Position + Quaternion)
        feedback_->q_hip  [leg]  =   mujoco::d->qpos[(leg)*3+    kHip_     +7];
        feedback_->q_knee [leg]  =   mujoco::d->qpos[(leg)*3+    kKnee_   +7];

        feedback_->qd_abad[leg]  =   mujoco::d->qvel[(leg)*3+    kAbad_    +6];
        feedback_->qd_hip [leg]  =   mujoco::d->qvel[(leg)*3+    kHip_     +6];
        feedback_->qd_knee[leg]  =   mujoco::d->qvel[(leg)*3+    kKnee_    +6];
        //std::cout<<"FOOT POS ??? "<< mujoco::d->qpos [(leg)*3+    3    +7]<<std::endl;

        /*
        geom_xpos stores n-th geom x-y-z pos in  1-d vector [1-x,1-y,1-z,2-x,2-y,2-z ....n-x,n-y,n-z]
        for geom 6, 10,14 and 18 below indexing is used such that [(6+leg*4)*3+0]
        */

            // std::cout<<"Geom 4 pos?? :: "<<mujoco::d->geom_xpos[(6+leg*4)*3+0] <<std::endl;
            // std::cout<<"Geom 4 pos?? :: "<<mujoco::d->geom_xpos[(6+leg*4)*3+1] <<std::endl;
            // std::cout<<"Geom 4 pos?? :: "<<mujoco::d->geom_xpos[(6+leg*4)*3+2] <<std::endl;

        bodyIMU->foot_pos[leg*3+0] = mujoco::d->geom_xpos[(6+leg*4)*3+0] ;
        bodyIMU->foot_pos[leg*3+1] = mujoco::d->geom_xpos[(6+leg*4)*3+1] ;
        bodyIMU->foot_pos[leg*3+2] = mujoco::d->geom_xpos[(6+leg*4)*3+2] ;

    }
    
    if(motor_input_type_)
    {
    for(int leg=0; leg<4; leg++){
        int AbadPole =1;
        // if (leg == 0 || leg == 1){AbadPole = -1;};
        // Command->tau_abad_ff[leg] = legTorque(0)*ABADpole;
        //printf("GAIN PRM:%f \n",mujoco::m->actuator_gainprm[(leg*9+kAbad_*3+1)*10]);

        
    
        mujoco::d->ctrl[(leg)*9+  kAbad_ *3   + 0 ] =cmd_->tau_abad_ff[leg]+cmd_->kp_abad[leg]*(cmd_->q_des_abad[leg] - feedback_->q_abad[leg]) +
        cmd_->kd_abad[leg]* (cmd_->qd_des_abad[leg] - feedback_->qd_abad[leg]);    //Torque
        // mujoco::d->ctrl[(leg)*9+  kAbad_  *3  + 1] =cmd_->q_des_abad[leg];     //Position
        // mujoco::d->ctrl[(leg)*9+  kAbad_  *3  + 2] =cmd_->qd_des_abad[leg];     //Velocity

        
        //  std::cout<< "F: "<< leg<< "  " <<cmd_->tau_abad_ff[leg]+cmd_->kp_abad[leg]* (cmd_->q_des_abad[leg] - feedback_->q_abad[leg]) +
        // cmd_->kd_abad[leg]* (cmd_->qd_des_abad[leg] - feedback_->qd_abad[leg])<<std::endl;

        // mujoco::m->actuator_gainprm[(leg*9+kAbad_*3+1)*10]=cmd_->kp_abad[leg];
        // mujoco::m->actuator_gainprm[(leg*9+kAbad_*3+2)*10]=cmd_->kd_abad[leg];


        mujoco::d->ctrl[(leg)*9+  kHip_   *3 + 0 ] =cmd_->tau_hip_ff[leg]+cmd_->kp_hip[leg]* (cmd_->q_des_hip[leg] - feedback_->q_hip[leg]) +
        cmd_->kd_hip[leg]* (cmd_->qd_des_hip[leg] - feedback_->qd_hip[leg]) ;   //Torque

        // mujoco::d->ctrl[(leg)*9+  kHip_    *3+ 1] =cmd_->q_des_hip[leg];     //Position
        // mujoco::d->ctrl[(leg)*9+  kHip_   *3 + 2] =cmd_->qd_des_hip[leg];     //Velocity

        // mujoco::m->actuator_gainprm[(leg*9+kHip_*3+1)*10]=cmd_->kp_hip[leg];
        // mujoco::m->actuator_gainprm[(leg*9+kHip_*3+2)*10]=cmd_->kd_hip[leg];


        mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 0 ] =cmd_->tau_knee_ff[leg]+cmd_->kp_knee[leg]* (cmd_->q_des_knee[leg] - feedback_->q_knee[leg]) +
        cmd_->kd_knee[leg]* (cmd_->qd_des_knee[leg] - feedback_->qd_knee[leg]) ;;    //Torque
        // mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 1] =cmd_->q_des_knee[leg];     //Position
        // mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 2] =cmd_->qd_des_knee[leg];     //Velocity

        // mujoco::m->actuator_gainprm[(leg*9 + kKnee_*3 + 1)*10]=cmd_->kp_knee[leg];
        // mujoco::m->actuator_gainprm[(leg*9 + kKnee_*3 + 2)*10]=cmd_->kd_knee[leg];
    }
    // printf("\n");
    }
    else 
    {
    for(int leg=0; leg<4; leg++){


        mujoco::d->ctrl[(leg)*9+  kAbad_ *3   + 0 ] =cmd_->tau_abad_ff[leg];  //Torque
        mujoco::d->ctrl[(leg)*9+  kAbad_  *3  + 1] =cmd_->q_des_abad[leg];     //Position
        mujoco::d->ctrl[(leg)*9+  kAbad_  *3  + 2] =cmd_->qd_des_abad[leg];     //Velocity



        mujoco::d->ctrl[(leg)*9+  kHip_   *3 + 0 ] =cmd_->tau_hip_ff[leg];//Torque
        mujoco::d->ctrl[(leg)*9+  kHip_    *3+ 1] =cmd_->q_des_hip[leg];     //Position
        mujoco::d->ctrl[(leg)*9+  kHip_   *3 + 2] =cmd_->qd_des_hip[leg];     //Velocity




        mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 0 ] =cmd_->tau_knee_ff[leg];    //Torque
        mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 1] =cmd_->q_des_knee[leg];     //Position
        mujoco::d->ctrl[(leg)*9+  kKnee_    *3+ 2] =cmd_->qd_des_knee[leg];     //Velocity


    }
        
        
    }








     bodyIMU->acc_x = mujoco::d->sensordata[0];
     bodyIMU->acc_y = mujoco::d->sensordata[1];
     bodyIMU->acc_z = mujoco::d->sensordata[2];

     bodyIMU->accelerometer[0] = mujoco::d->sensordata[0];
     bodyIMU->accelerometer[1] = mujoco::d->sensordata[1];
     bodyIMU->accelerometer[2] = mujoco::d->sensordata[2];

     bodyIMU->heave = mujoco::d->qvel[0] ;
     bodyIMU->heave_dt = mujoco::d->qvel[1] ;
     bodyIMU->heave_ddt = mujoco::d->qvel[2] ;


     bodyIMU->gyr_x = mujoco::d->qvel[3];
     bodyIMU->gyr_y = mujoco::d->qvel[4];
     bodyIMU->gyr_z = mujoco::d->qvel[5];

     bodyIMU->gyro[0] = mujoco::d->qvel[3];
     bodyIMU->gyro[1] = mujoco::d->qvel[4];
     bodyIMU->gyro[2] = mujoco::d->qvel[5];

     bodyIMU->quat[0] = mujoco::d->qpos[3];
     bodyIMU->quat[1] = mujoco::d->qpos[4];
     bodyIMU->quat[2] = mujoco::d->qpos[5];
     bodyIMU->quat[3] = mujoco::d->qpos[6];

     bodyIMU->pos_x = mujoco::d->qpos[0];
     bodyIMU->pos_y = mujoco::d->qpos[1];
     bodyIMU->pos_z = mujoco::d->qpos[2];

}

/**
 * @brief Checks if the close button on the simulation window_ is clicked. 
 * 
 * @return true 
 * @return false 
 */
bool Simulation::CheckWindowShouldClose(){

    return glfwWindowShouldClose(window_);
};

/**
 * @brief When this function is called the GLFW window_ image is updated as per the data from the simulation. Run this at 120 Hz for better visualization.
 * 
 */
void Simulation::UpdateScene(){

    //  #TODO do this at 60hz for efficiency.
    if(glfwWindowShouldClose(window_)){LOG(WARNING)<<"WARNING: The Simulation window was closed. The program will terminate"; 
    exit(0)
    ;} 
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};

    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    // update scene and render
    mujoco::opt.frame = mjFRAME_WORLD;
    
    
    




    mjv_updateScene(mujoco::m, mujoco::d, &mujoco::opt, NULL, &mujoco::cam, mjCAT_ALL, &mujoco::scn);

    mjr_render(viewport, &mujoco::scn, &mujoco::con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();


    // mujoco::cam.azimuth = 0;
    // mujoco::cam.elevation = -45;
    // mujoco::cam.distance = 4;
    mujoco::cam.lookat[0] = mujoco::d->qpos[0];
    mujoco::cam.lookat[1] = mujoco::d->qpos[1];
    mujoco::cam.lookat[2] = mujoco::d->qpos[2];

};

/**
 * @brief Steps the simulation by one timestep.
 * 
 */
void Simulation::StepSim(){
    mj_step(mujoco::m, mujoco::d);
};

/**
 * @brief Mathematical Model for Actuator. Incomplete  
 * 
 */


/**
 * @brief First, updates the CAN Command and then runs the simulation by one timestep. This function should be called periodically to run the simulation. 
 * 
 */
void Simulation::RunOnce()
{
    UpdateCan();
    StepSim();
    };

/**
 * @brief Set the adress of CAN Command struct that sends inputs to the simulation.
 * 
 * @param _cmd_ Pointer to the Command struct
 */
void Simulation::SetCommand(SpiCommand* _cmd_)
{
    cmd_=_cmd_;
}

/**
 * @brief Sets the adress of CAN Data struct that reads feedback_s from the simulation. 
 * 
 * @param _feedback_ Pointer to the Feedback struct.
 */
void Simulation::SetFeedback(SpiData* _feedback_,VectorNavData* _bodyIMU ){

    feedback_=_feedback_;
    bodyIMU = _bodyIMU;

}

/**
 * @brief Is used to set initial conditions of the robot. Hard-Coded to resemble the actual robot. Should be called once before the first call of run_once().
 * 
 * 
 */

void Simulation::SetInitialConditions()  //TODO: Do this from a YAML file?
{
    for(int leg=0; leg<4; leg++){
        mujoco::d->qpos[(leg)*3+  kAbad_  +7]=1*(M_PI/180)*kSideSign_[leg];     // Add 7 to skip the first 7 dofs from body. (Position + Quaternion)
        mujoco::d->qpos[(leg)*3+  kHip_   +7]=-90*(M_PI/180);//*kDirSign_[leg];
        mujoco::d->qpos[(leg)*3+  kKnee_  +7]=173*(M_PI/180);//*kDirSign_[leg];
    }

}


/**
 * @brief Sets the simulation's timestep that is used when the simulation is called each time. If not set defaults to 0.01 secs.
 * 
 * @param ts Timestep in seconds.
 */
void Simulation::SetTimestep(double ts)
{

    //TODO: Fix this, simulasyon timestep ts ile aynı olması.
    // mujoco::m->opt.timestep=0.002;
    mujoco::m->opt.timestep=ts;
}


void Simulation::ShowContactForce(bool enable){
    mujoco::opt.flags[15] = char(enable);
}


/**
 * @brief Destroy the Simulation:: Simulation object
 * 
 */
Simulation::~Simulation()
{
      LOG(WARNING)<<"[EXIT]"<<"SIM Closing";

    // free visualization storage
    mjv_freeScene(&mujoco::scn);
    mjr_freeContext(&mujoco::con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(mujoco::d);
    mj_deleteModel(mujoco::m);
    // mj_deactivate();
}