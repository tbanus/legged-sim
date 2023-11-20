cmake_minimum_required(VERSION 3.3)
find_package(cmake_modules REQUIRED)

project(legged-sim)
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


option(BUILD_DEPENDENCIES "Build ${PROJECT_NAME} and the projects it depends on." OFF)

if(BUILD_DEPENDENCIES)
  include("${CMAKE_CURRENT_SOURCE_DIR}/SuperBuild.cmake")
  return()
endif()

set (CMAKE_CXX_STANDARD 14)

include(GNUInstallDirs)


  

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

set(lib_INCLUDE_DIR ${INCLUDE_DIR} CACHE INTERNAL "")
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${INCLUDE_DIR})


include_directories("/usr/local/include/lcm/")
include_directories(
	            ${PROJECT_SOURCE_DIR}/include            
	            ${PROJECT_SOURCE_DIR}/src)          
	            

set(LIBRARY_NAME ${PROJECT_NAME} CACHE INTERNAL "")

# set(LIBRARY_SOURCE ${SOURCE_DIR}/**)   


# find threads - needed for header only csv parser used in this project

find_package (Threads)
find_package(lcm)
FIND_LIBRARY(GLOG_LIBRARY glog)
find_package(gflags REQUIRED)
# find_package(urdf REQUIRED)
# find_package(TinyXML2)
# find_package(TinyXML REQUIRED)
find_package(Curses REQUIRED)
find_package(glfw3  REQUIRED)
find_package(OpenGL REQUIRED)
find_package(mujoco)



function(custom_executable subfolder target)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES" ${ARGN})
  add_executable(${target} src/${subfolder}/${target}.cpp)
  target_include_directories(${target} PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${target} PRIVATE
    ${ARG_DEPENDENCIES}
  )
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(bridge main
  DEPENDENCIES $${CMAKE_THREAD_LIBS_INIT} lcm mujoco glfw GLEW inih dynacore_param_handler qpOASES JCQP osqp ${CURSES_LIBRARIES} glog gflags)

# install(TARGETS ${library}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)


# Create an Object to Make both Static and Shared Libraries in the same CMakeLists File.
# add_library(${PROJECT_NAME}-object OBJECT ${LIBRARY_SOURCE} )

# target_link_libraries(${PROJECT_NAME}-object libvncxx inih dynacore_param_handler qpOASES JCQP osqp ${CURSES_LIBRARIES} glog gflags)


# add_library(${PROJECT_NAME} SHARED $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

# add_library(${PROJECT_NAME}-static STATIC $<TARGET_OBJECTS:${PROJECT_NAME}-object>)

# add_executable(main ${SOURCE_DIR}/main.cpp )

# target_link_libraries(main ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT} lcm mujoco glfw GLEW inih dynacore_param_handler qpOASES JCQP osqp ${CURSES_LIBRARIES} glog gflags )

  #target_link_libraries(main ${PROJECT_NAME}-object ${CMAKE_THREAD_LIBS_INIT} inih dynacore_param_handler )
#add_executable(MotorDriverTest_original ${SOURCE_DIR}/MotorDriverTest_original.cpp)
#target_link_libraries(MotorDriverTest_original ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})




# Add execuatble for a trajectory following test
# add_executable(MotorTrajTest ${SOURCE_DIR}/MotorDriverTrajTest.cpp)
# target_link_libraries(MotorTrajTest ${PROJECT_NAME}-static ${CMAKE_THREAD_LIBS_INIT})

# Add PUBLIC_HEADERS so that other applications on the computer can use this library.
# set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "include/motor_driver/MotorDriver.hpp;include/motor_driver/CANInterface.hpp")

# INSTALL(TARGETS ${PROJECT_NAME}
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} 
#         INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} 
#         ARCHIVE DESTINATION lib
#        )