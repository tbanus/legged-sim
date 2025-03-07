#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include <ControlParameters/MIT_UserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

/**
 *
 */


template<typename T>
class LocomotionCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[4];
    Vec3<T> vFoot_des[4];
    Vec3<T> aFoot_des[4];
    Vec3<T> Fr_des[4];

    Vec4<T> contact_state = Vec4<T>::Ones();
};

template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* _quadruped;
  StateEstimatorContainer<T>* _stateEstimator;
  LegController<T>* _legController;
  GaitScheduler<T>* _gaitScheduler;
  DesiredStateCommand<T>* _desiredStateCommand;
  RobotControlParameters* controlParameters;
  MIT_UserParameters* userParameters;
  LocomotionCtrlData<T> locomotionCtrlData;
  // VisualizationData* visualizationData;
};



template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

template class LocomotionCtrlData<double>;
template class LocomotionCtrlData<float>;
#endif  // CONTROLFSM_H