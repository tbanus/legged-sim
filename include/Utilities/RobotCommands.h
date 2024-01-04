#ifndef LEGGEDSIM_ROBOTCOMMANDS
#define LEGGEDSIM_ROBOTCOMMANDS

#include "cppTypes.h"

struct SpiCommand {
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];

  int32_t flags[4];
};

/*!
 * Data from spine board
 */
struct SpiData {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
  int32_t flags[4];
  int32_t spi_driver_status;
};


// typedef struct ImuData{


// float yaw ;   float pitch  ;     float roll ;
// float gyr_x ; float  gyr_y ;    float gyr_z ;
// float acc_x ; float acc_y ;     float acc_z ;
// float heave ; float heave_dt ;  float heave_ddt ;
// float pos_x; float pos_y; float pos_z;
// float quat[4];
// int ContactStatus[4];
// float foot_pos[12];
// float grf[12];
    
// };
struct VectorNavData {
  Vec3<float> accelerometer;
  Vec3<float> gyro;
  // Quat<float> quat;

float yaw ;   float pitch  ;     float roll ;
float gyr_x ; float  gyr_y ;    float gyr_z ;
float acc_x ; float acc_y ;     float acc_z ;


float heave ; float heave_dt ;  float heave_ddt ;
float pos_x; float pos_y; float pos_z;
float quat[4];
int ContactStatus[4];
float foot_pos[12];
float grf[12];
  // todo is there status for the vectornav?
};

#endif