#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER

#include <FSM/ControlFSMData.h>

#include <WBC_Ctrl/WBC_Ctrl.hpp>
#include <Controllers/WBC_Ctrl/ContactSet/SingleContact.hpp>
#include <Controllers/WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <Controllers/WBC_Ctrl/TaskSet/BodyPosTask.hpp>
//#include <WBC_Ctrl/TaskSet/BodyPostureTask.hpp>
#include <Controllers/WBC_Ctrl/TaskSet/LinkPosTask.hpp>
template<typename T>
class LocomotionCtrl: public WBC_Ctrl<T>{
  public:
    LocomotionCtrl(FloatingBaseModel<T> model);
    virtual ~LocomotionCtrl();

  protected:
    virtual void _ContactTaskUpdate(
        void * input, ControlFSMData<T> & data);
    virtual void _ContactTaskUpdateTEST(void * input, ControlFSMData<T> & data);
    void _ParameterSetup(const MIT_UserParameters* param);
    void _CleanUp();
    #ifdef LCM
    virtual void _LCM_PublishData();
    #endif
    LocomotionCtrlData<T>* _input_data;

    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;

    Task<T>* _foot_task[4];
    ContactSpec<T>* _foot_contact[4];

    Vec3<T> pre_foot_vel[4];

    Vec3<T> _Fr_result[4];
    Quat<T> _quat_des;
};

#endif

