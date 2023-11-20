cmake_minimum_required(VERSION 3.3)
find_package(cmake_modules REQUIRED)

project(OPY-Software)
include(ExternalProject)

if(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -O3")
else(WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
endif(WIN32)


option(HW_ENABLE "HW_ENABLE" ON)

if(HW_ENABLE)
    add_compile_definitions(HW_ENABLE)
endif()

#option(Awesome_ENABLE_EXTRA "Enable more awesome stuff" OFF)

option(BUILD_DEPENDENCIES "Build ${PROJECT_NAME} and the projects it depends on." OFF)

if(BUILD_DEPENDENCIES)
  include("${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild.cmake")
  return()
endif()

set (CMAKE_CXX_STANDARD 14)

include(GNUInstallDirs)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include/motor_driver)
set(INCLUDE_DIR2 ${PROJECT_SOURCE_DIR}/include/Utilities)
set(INCLUDE_DIR3 ${PROJECT_SOURCE_DIR}/include/exlcm)
set(INCLUDE_DIR4 ${PROJECT_SOURCE_DIR}/include/FSM)
set(INCLUDE_DIR5 ${PROJECT_SOURCE_DIR}/include/Simulation)
set(INCLUDE_DIR6 ${PROJECT_SOURCE_DIR}/include/Bridge)
set(INCLUDE_DIR7 ${PROJECT_SOURCE_DIR}/include/Controllers)

include_directories(${PROJECT_SOURCE_DIR}/third_party/inih)       # inih includes
include_directories(${PROJECT_SOURCE_DIR}/third_party/osqp/include)       # osqp includes
include_directories(${PROJECT_SOURCE_DIR}/third_party/qpOASES/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ParamHandler)
include_directories(${PROJECT_SOURCE_DIR}/third_party/vn100/include)
include_directories(${PROJECT_SOURCE_DIR}/Controllers/JCQP)

set(lib_INCLUDE_DIR ${INCLUDE_DIR} CACHE INTERNAL "")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${INCLUDE_DIR})
include_directories(${INCLUDE_DIR2})
include_directories(${INCLUDE_DIR3})
include_directories(${INCLUDE_DIR4})
include_directories(${INCLUDE_DIR5})
include_directories(${INCLUDE_DIR6})
include_directories(${INCLUDE_DIR7})


include_directories("/usr/local/include/lcm/")
include_directories(
	            ${PROJECT_SOURCE_DIR}/include            
	            ${PROJECT_SOURCE_DIR}/src)          
	            

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/qpOASES)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ParamHandler)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/inih)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/JCQP)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/osqp)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/vn100)
#target_include_directories(inih  PUBLIC ${PROJECT_SOURCE_DIR}/third-party/inih)
#target_include_directories(dynacore_param_handler PUBLIC ${PROJECT_SOURCE_DIR}/third-party/ParamHandler/dynacore_yaml-cpp)
set(LIBRARY_NAME ${PROJECT_NAME} CACHE INTERNAL "")

set(LIBRARY_SOURCE ${SOURCE_DIR}/MotorDriver.cpp
              ${SOURCE_DIR}/PeriodicTask.cpp
	           ${SOURCE_DIR}/CANInterface.cpp
             ${SOURCE_DIR}/asel_reader.c
             ${SOURCE_DIR}/IMUTest_helper.c
             ${SOURCE_DIR}/MotorCMD.cpp
             ${SOURCE_DIR}/SwingHelper.cpp
             ${SOURCE_DIR}/send_message.cpp
             #${SOURCE_DIR}/IMUreader.cpp
             ${SOURCE_DIR}/IMUreader_vn100.cpp
             ${SOURCE_DIR}/Controllers/LegController.cpp
             ${SOURCE_DIR}/Dynamics/Quadruped.cpp
             ${SOURCE_DIR}/Dynamics/FloatingBaseModel.cpp
             ${SOURCE_DIR}/Controllers/SingleLegController.cpp
             ${SOURCE_DIR}/LCM/RecieveLCMMessage.cpp
             ${SOURCE_DIR}/Utilities/utilities.cpp
             ${SOURCE_DIR}/Controllers/DesiredStateCommand.cpp
             ${SOURCE_DIR}/Controllers/OrientationEstimator.cpp
             ${SOURCE_DIR}/Controllers/ContactEstimator.cpp
             ${SOURCE_DIR}/Controllers/PositionVelocityEstimator.cpp
             ${SOURCE_DIR}/Controllers/WBIC.cpp
             ${SOURCE_DIR}/Controllers/WBC_Ctrl.cpp
             ${SOURCE_DIR}/Controllers/KinWBC.cpp
             ${SOURCE_DIR}/FSM/Stand_FSM.cpp
             ${SOURCE_DIR}/FSM/FSM_State.cpp
             ${SOURCE_DIR}/FSM/ControlFSM.cpp
             ${SOURCE_DIR}/FSM/FSM_State_BalanceStand.cpp
             ${SOURCE_DIR}/FSM/FSM_State_Locomotion.cpp
             ${SOURCE_DIR}/FSM/FSM_State_Passive.cpp
             ${SOURCE_DIR}/MIT_Controller.cpp
             ${SOURCE_DIR}/RobotRunner.cpp
             ${SOURCE_DIR}/Bridge/Bridge.cpp
             ${SOURCE_DIR}/Controllers/ControlParameters/ControlParameterInterface.cpp
             ${SOURCE_DIR}/Controllers/ControlParameters/ControlParameters.cpp
             ${SOURCE_DIR}/Controllers/FootstepPlanner/GraphSearch.cpp
             ${SOURCE_DIR}/Controllers/convexMPC_interface.cpp
             ${SOURCE_DIR}/Controllers/SolverMPC.cpp
             ${SOURCE_DIR}/Controllers/RobotState.cpp
             ${SOURCE_DIR}/Controllers/FootSwingTrajectory.cpp
             ${SOURCE_DIR}/Controllers/ContactSet/FixedBodyContact.cpp
             ${SOURCE_DIR}/Controllers/ContactSet/SingleContact.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/BodyOriTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/BodyPosTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/BodyPostureTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/BodyRyRzTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/JPosTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/LinkPosTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/LocalHeadPosTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/LocalPosTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/LocalRollTask.cpp
             ${SOURCE_DIR}/Controllers/TaskSet/LocalTailPosTask.cpp
             ${SOURCE_DIR}/Controllers/LocomotionCtrl.cpp
             ${SOURCE_DIR}/Controllers/ConvexMPCLocomotion.cpp
             ${SOURCE_DIR}/Controllers/SparseCMPC.cpp
             ${SOURCE_DIR}/Controllers/SparseCMPC_Math.cpp
             ${SOURCE_DIR}/Controllers/OsqpTriples.cpp
             ${SOURCE_DIR}/Controllers/GaitScheduler.cpp
             ${SOURCE_DIR}/Controllers/Gait.cpp
             ${SOURCE_DIR}/Controllers/ContactKalman.cpp
             ${PROJECT_SOURCE_DIR}/include/third-party/Goldfarb_Optimizer/QuadProg++.cc
             )   

set(LIBRARY_SOURCE2  ${SOURCE_DIR}/Simulation/Simulation.cpp
             ${SOURCE_DIR}/Simulation/SimPeriodicTask.cpp)
# find threads - needed for header only csv parser used in this project

find_package (Threads)
find_package(lcm)
FIND_LIBRARY(GLOG_LIBRARY glog)
find_package(gflags REQUIRED)
# find_package(urdf REQUIRED)
# find_package(TinyXML2)
# find_package(TinyXML REQUIRED)
# include(FetchContent)
# FetchContent_Declare(
#   glog
#   GIT_REPOSITORY https://github.com/google/glog.git
#   # GIT_TAG b33e3ba #v0.6.0
#   FIND_PACKAGE_ARGS NAMES  GLOG_LIBRARY glog
# )
# FetchContent_MakeAvailable(glog)


if (NOT HW_ENABLE)
  find_package(Curses REQUIRED)
  find_package(glfw3  REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(mujoco)
endif()  
#find_package(Poco REQUIRED COMPONENTS Foundation)
# find_package(urdf REQUIRED)
# find_package(TinyXML REQUIRED)
# FIND_LIBRARY(GLOG_LIBRARY glog)
# find_package (glog 0.6.0)
# find_package(GLEW )



# Create an Object to Make both Static and Shared Libraries in the same CMakeLists File.
if (NOT HW_ENABLE)
add_library(${PROJECT_NAME}-object OBJECT ${LIBRARY_SOURCE} ${LIBRARY_SOURCE2} )
else()
add_library(${PROJECT_NAME}-object OBJECT ${LIBRARY_SOURCE} )
endif()
#target_include_directories(${PROJECT_NAME}-object PUBLIC ${PROJECT_SOURCE_DIR}/third-party/vn100)

target_link_libraries(${PROJECT_NAME}-object libvncxx inih dynacore_param_handler qpOASES JCQP osqp ${CURSES_LIBRARIES} glog gflags)

# this is only needed for older compilers but doesn't hurt either to have it
set_target_properties(${PROJECT_NAME}-object PROPERTIES POSITION_INDEPENDENT_CODE 1)

add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

add_library(${PROJECT_NAME}-static STATIC $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

# Add execuatble for a simple on/off/zero position driver test
# add_executable(MotorDriverTest ${SOURCE_DIR}/MotorDriverTest.cpp)
# target_link_libraries(MotorDriverTest ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})
# add_executable(IMU_Test_vn100 ${SOURCE_DIR}/IMU_Test_vn100.cpp)
# target_link_libraries(IMU_Test_vn100 ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT} lcm libvncxx )
# add_executable(Swing_Demo_Main ${SOURCE_DIR}/Swing_Demo_Main.cpp)
# target_link_libraries(Swing_Demo_Main ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT} lcm   )
add_executable(main ${SOURCE_DIR}/main.cpp )

if(NOT HW_ENABLE)
  target_link_libraries(main ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT} lcm mujoco glfw GLEW inih dynacore_param_handler qpOASES JCQP osqp ${CURSES_LIBRARIES} glog gflags )
else()
  target_link_libraries(main ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT} libvncxx lcm  inih dynacore_param_handler qpOASES JCQP osqp glog gflags)
endif()
  #target_link_libraries(main ${PROJECT_NAME}-object ${CMAKE_THREAD_LIBS_INIT} inih dynacore_param_handler )
#add_executable(MotorDriverTest_original ${SOURCE_DIR}/MotorDriverTest_original.cpp)
#target_link_libraries(MotorDriverTest_original ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})




# Add execuatble for a trajectory following test
# add_executable(MotorTrajTest ${SOURCE_DIR}/MotorDriverTrajTest.cpp)
# target_link_libraries(MotorTrajTest ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})

# Add PUBLIC_HEADERS so that other applications on the computer can use this library.
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "include/motor_driver/MotorDriver.hpp;include/motor_driver/CANInterface.hpp")

INSTALL(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} 
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} 
        ARCHIVE DESTINATION lib
       )