/*! @file OPYModel.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah model.name_.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef URDFMODEL_H
#define URDFMODEL_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"
// #include <urdf/model.h>
#include <math.h>
using namespace std;

#include <tinyxml2.h>
#include <glog/logging.h>
// #define concat(first, second) first second
// #define robot_name "comar"
/*!
 * Generate a Quadruped model of Mini opy


 */
// #ifndef ROBOT_MODEL
// #define ROBOT_MODEL "solo12_floatingbase"
// #endif
// TODO @etanfener
template <typename T>
Quadruped<T> buildModelFromURDF(string fname, RobotType robotType)
{

    // Create opy object
    Quadruped<T> opy;

    opy._robotType = robotType;

    // uncomment to override filename param

    // char fname[100];
    // strcpy(fname, "../resource/");
    // strcat(fname, ROBOT_MODEL);
    // strcat(fname, "/comar.urdf");

    // initiate xml object
    tinyxml2::XMLDocument doc;
    doc.LoadFile(fname.c_str());
    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
    {
        LOG(FATAL) << "[URDF Parser] Unable to find robot element. Make sure your urdf is properly formed" ;
    }

    double rotorRadius, rotorHeight, rotorMass;

    // Get robot name
    const char *robot_name = robot->Attribute("name");
    opy._robotName=std::string(robot_name);
    if(!robot_name)
    {
        LOG(FATAL) << "[URDF Parser] Unable to find robot name element. Make sure your urdf is properly formed" ;
    }
   



    
    tinyxml2::XMLElement *custom_attributes = robot->FirstChildElement("custom");
    if (!custom_attributes)
    {
        LOG(ERROR) << "[URDF Parser] Could not find the 'custom_attributes' element in the xml file.";
    }
    else
    {
        opy._bodyHeight = stof(custom_attributes->Attribute("body_height"));

        opy._abadGearRatio = stof(custom_attributes->Attribute("abadGearRatio"));

        opy._hipGearRatio = stof(custom_attributes->Attribute("hipGearRatio"));

        opy._kneeGearRatio = stof(custom_attributes->Attribute("kneeGearRatio"));

        opy._motorTauMax = stof(custom_attributes->Attribute("motorTauMax"));

        opy._batteryV = stof(custom_attributes->Attribute("batteryV"));

        opy._motorKT = stof(custom_attributes->Attribute("motorKT"));

        opy._motorR = stof(custom_attributes->Attribute("motorR"));

        rotorRadius = stof(custom_attributes->Attribute("rotorRadius"));

        rotorHeight = stof(custom_attributes->Attribute("rotorHeight"));

        rotorMass = stof(custom_attributes->Attribute("rotorMass"))*0;
    }


    //     //create joint objects and initiate
    // create blank link objects
    tinyxml2::XMLElement *bodyLink, *abadLink, *hipLink, *kneeLink, *footLink;
    // cursor
    tinyxml2::XMLElement *link = robot->FirstChildElement("link");
    for (int i = 0; i < 18; i++)
    {
        if (link->Attribute("name", (std::string(robot_name) + std::string("/base_link_inertia")).c_str()))
        {
            bodyLink = link;
        }
        if (link->Attribute("name", (std::string(robot_name) + std::string("/abad_2")).c_str()))
        {
            abadLink = link;
        }
        if (link->Attribute("name", (std::string(robot_name) + std::string("/hip_2")).c_str()))
        {
            hipLink = link;
        }
        if (link->Attribute("name", (std::string(robot_name) + std::string("/knee_2")).c_str()))
        {
            kneeLink = link;
        }
        if (link->Attribute("name", (std::string(robot_name) + std::string("/foot_2")).c_str()))
        {
            footLink = link;
        }

        link = link->NextSiblingElement("link");
    };

    if ( (!abadLink) || (!hipLink) || (!kneeLink) || (!footLink) )
    {
        LOG(FATAL)<< "[URDF Parser] Cannot find one of the main links. Make sure your URDF is formed right and you follow the needed name convention.";
    }



    tinyxml2::XMLElement *base_joint, *abad_2_joint, *hip_2_joint, *knee_2_joint, *foot_2_joint;
    tinyxml2::XMLElement *joint = robot->FirstChildElement("joint");
    for (int i = 0; i < 17; i++)
    {

        if (joint->Attribute("name", (std::string(robot_name) + std::string("/base_joint")).c_str()))
        {
            base_joint = joint;
        };

        if (joint->Attribute("name", (std::string(robot_name) + std::string("/abad_2_joint")).c_str()))
        {
            abad_2_joint = joint;
        };

        if (joint->Attribute("name", (std::string(robot_name) + std::string("/hip_2_joint")).c_str()))
        {
            hip_2_joint = joint;
        };

        if (joint->Attribute("name", (std::string(robot_name) + std::string("/knee_2_joint")).c_str()))
        {
            knee_2_joint = joint;
        };

        if (joint->Attribute("name", (std::string(robot_name) + std::string("/foot_2_joint")).c_str()))
        {
            foot_2_joint = joint;
        }

        joint = joint->NextSiblingElement("joint");
    }


    if ( (!abad_2_joint) || (!hip_2_joint) || (!knee_2_joint) || (!foot_2_joint) )
    {
        LOG(FATAL)<< "[URDF Parser] Cannot find one of the main joints. Make sure your URDF is formed right and you follow the needed name convention.";
    }

    // set opy prooerties from urdf link properties
    if(bodyLink->FirstChildElement("inertial")->FirstChildElement("mass")->Attribute("value"))
    {
        opy._bodyMass = stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("mass")->Attribute("value"));
    }
    else
    {
        LOG(FATAL)<< "[URDF Parser] Cannot find base link inertial. Make sure your URDF is formed right and you follow the needed name convention.";
    }



    
    std::string abad_2_joint_origin[3];
    int i = 0;
    std::stringstream abad_2_joint_origin_ss(std::string(abad_2_joint->FirstChildElement("origin")->Attribute("xyz")));
    while (abad_2_joint_origin_ss.good() && i < 3)
    {
        abad_2_joint_origin_ss >> abad_2_joint_origin[i];
        ++i;
    }
    opy._bodyLength = stof(abad_2_joint_origin[0]) * 2;
    opy._bodyWidth = stof(abad_2_joint_origin[1]) * 2;

    std::string hip_2_joint_origin[3];
    i = 0;
    std::stringstream hip_2_joint_origin_ss(std::string(hip_2_joint->FirstChildElement("origin")->Attribute("xyz")));
    while (hip_2_joint_origin_ss.good() && i < 3)
    {
        hip_2_joint_origin_ss >> hip_2_joint_origin[i];
        ++i;
    }

    std::string knee_2_joint_origin[3];
    i = 0;
    std::stringstream knee_2_joint_origin_ss(std::string(knee_2_joint->FirstChildElement("origin")->Attribute("xyz")));
    while (knee_2_joint_origin_ss.good() && i < 3)
    {
        knee_2_joint_origin_ss >> knee_2_joint_origin[i];
        ++i;
    }

    std::string foot_2_joint_origin[3];
    i = 0;
    std::stringstream foot_2_joint_origin_ss(std::string(foot_2_joint->FirstChildElement("origin")->Attribute("xyz")));
    while (foot_2_joint_origin_ss.good() && i < 3)
    {
        foot_2_joint_origin_ss >> foot_2_joint_origin[i];
        ++i;
    }
    ~i;

    // Joint lengths for the jacobian
    opy._abadLinkLength = stof(hip_2_joint_origin[1]) ;
    // opy._abadXOffset = stof(hip_2_joint_origin[0])*0.0;
    opy._hipLinkLength = -1 * stof(knee_2_joint_origin[2]);
    opy._kneeLinkLength = -1 * stof(foot_2_joint_origin[2])+0.005;
    //TODO @tarik this is defined to be? 

    opy._kneeLinkY_offset = -1 * stof(knee_2_joint_origin[1]); // TODO
  
    float upper_limit = stof(hip_2_joint->FirstChildElement("limit")->Attribute("upper"));
    if (!upper_limit)
    {
        LOG(ERROR)<<"[URDF Parser] Cannot find upper limit for the leg. Defaulting to Pi";
        upper_limit=M_PI;
    }
    opy._maxLegLength = cos(upper_limit) * (opy._hipLinkLength + cos(upper_limit) * opy._kneeLinkLength);
    opy._jointDamping = stof(hip_2_joint->FirstChildElement("dynamics")->Attribute("damping"));
    opy._jointDryFriction = stof(hip_2_joint->FirstChildElement("dynamics")->Attribute("friction"));

    opy._bodyLength = stof(abad_2_joint_origin[0]) * 2+ stof(hip_2_joint_origin[0])*2;
    opy._bodyWidth = stof(abad_2_joint_origin[1]) * 2+ stof(hip_2_joint_origin[1])*2;
    //TODO @tarik this is defined to be? 

    // spatial inertias
    Mat3<T> abadRotationalInertia;

    abadRotationalInertia << stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixx")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyy")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(abadLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("izz"));
    std::string abad_2_com_origin[3];
    i = 0;
    std::stringstream abad_2_com_origin_ss(std::string(abadLink->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz")));
    while (abad_2_com_origin_ss.good() && i < 3)
    {
        abad_2_com_origin_ss >> abad_2_com_origin[i];
        ++i;
    }

    Vec3<T> abadCOM(stof(abad_2_com_origin[0]), stof(abad_2_com_origin[1]), stof(abad_2_com_origin[2])); // LEFT

    SpatialInertia<T> abadInertia(stof(abadLink->FirstChildElement("inertial")->FirstChildElement("mass")->Attribute("value")), abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;

    hipRotationalInertia << stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixx")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyy")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(hipLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("izz"));

    std::string hip_2_com_origin[3];
    i = 0;
    std::stringstream hip_2_com_origin_ss(std::string(hipLink->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz")));
    while (hip_2_com_origin_ss.good() && i < 3)
    {
        hip_2_com_origin_ss >> hip_2_com_origin[i];
        ++i;
    }

    Vec3<T> hipCOM(stof(hip_2_com_origin[0]), stof(hip_2_com_origin[1]), stof(hip_2_com_origin[2])); // LEFT

    SpatialInertia<T> hipInertia(stof(hipLink->FirstChildElement("inertial")->FirstChildElement("mass")->Attribute("value")), hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia;

    kneeRotationalInertia << stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixx")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyy")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
                            stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("izz"));

    std::string knee_2_com_origin[3];
    i = 0;
    std::stringstream knee_2_com_origin_ss(std::string(kneeLink->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz")));
    while (knee_2_com_origin_ss.good() && i < 3)
    {
        knee_2_com_origin_ss >> knee_2_com_origin[i];
        ++i;
    }

    Vec3<T> kneeCOM(stof(knee_2_com_origin[0]), stof(knee_2_com_origin[1]), stof(knee_2_com_origin[2])); // LEFT

    SpatialInertia<T> kneeInertia(stof(kneeLink->FirstChildElement("inertial")->FirstChildElement("mass")->Attribute("value")), kneeCOM, kneeRotationalInertia);

    // // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<T> rotorRotationalInertiaZ;

    rotorRotationalInertiaZ << (3 * rotorRadius * rotorRadius + rotorHeight * rotorHeight) / 12, 0, 0, 0, (3 * rotorRadius * rotorRadius + rotorHeight * rotorHeight) / 12, 0, 0, 0, 0.5 * rotorRadius * rotorRadius;
    rotorRotationalInertiaZ = rotorMass * rotorRotationalInertiaZ;
    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();
    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(rotorMass, rotorCOM, rotorRotationalInertiaY);

    // body spatial inertias

    Mat3<T> bodyRotationalInertia;

    bodyRotationalInertia << stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixx")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixy")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyy")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("ixz")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("iyz")),
        stof(bodyLink->FirstChildElement("inertial")->FirstChildElement("inertia")->Attribute("izz"));

    std::string body_com_origin[3];
    i = 0;
    std::stringstream body_com_origin_ss(std::string(bodyLink->FirstChildElement("inertial")->FirstChildElement("origin")->Attribute("xyz")));
    while (body_com_origin_ss.good() && i < 3)
    {
        body_com_origin_ss >> body_com_origin[i];
        ++i;
    }

    Vec3<T> bodyCOM(stof(body_com_origin[0]), stof(body_com_origin[1]), stof(body_com_origin[2])); // LEFT

    SpatialInertia<T> bodyInertia(opy._bodyMass, bodyCOM, bodyRotationalInertia);

    // set properties from dummy variavles
    opy._abadInertia = abadInertia;
    opy._hipInertia = hipInertia;
    opy._kneeInertia = kneeInertia;
    opy._abadRotorInertia = rotorInertiaX;
    opy._hipRotorInertia = rotorInertiaY;
    opy._kneeRotorInertia = rotorInertiaY;
    opy._bodyInertia = bodyInertia;

    // locations
    opy._abadLocation = Vec3<T>( stof(abad_2_joint_origin[0]) ,  stof(abad_2_joint_origin[1]), 0) ;
    opy._hipLocation = Vec3<T>(stof(hip_2_joint_origin[0])*0, opy._abadLinkLength, 0);
    //TODO @tarik this is defined to be? 
    opy._kneeLocation = Vec3<T>(0,  -opy._kneeLinkY_offset, -opy._hipLinkLength);

    opy._abadRotorLocation = Vec3<T>(opy._bodyLength, opy._bodyWidth, 0) * 0.5;
    opy._hipRotorLocation = Vec3<T>(0, opy._abadLinkLength, 0);
    opy._kneeRotorLocation = Vec3<T>(0, opy._kneeLinkY_offset, 0);

    return opy;
};

#endif // PROJECT_MINICHEETAH_H
