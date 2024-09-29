# include_directories("./")
cmake_minimum_required(VERSION 3.3)
#find_package(cmake_modules REQUIRED)

project(Legged-Software)

add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/qpOASES)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/ParamHandler)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/inih)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/JCQP)
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/osqp)
# add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/vn100)

include_directories("./include")
include_directories("./include/Controllers")

include_directories("./user/Embedded_Controller")
include_directories("./user/Embedded_Controller/Controllers")

include_directories("./third-party")
# include_directories("./third-party/osqp")
# include_directories("./third-party/osqp/lin_sys")

include_directories("./third-party/osqp/include")

include_directories("./third-party/qpOASES/include")
include_directories("./third-party/GoldFarb_Optimizer")
include_directories("./third-party/ParamHandler")
include_directories("./third-party/JCQP")
include_directories("./third-party/JCQP/amd/include")

include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("./lcm_types/cpp")


include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# include_directories("./")
# include_directories("./Controllers")
# include_directories("../../include")
# # include_directories("../../common/include/")
# # include_directories("../../common/FootstepPlanner")
# include_directories("../../third-party/")
# include_directories("../../third-party/qpOASES/include")
# include_directories("../../third-party/ParamHandler")

# include_directories("../../lcm_types/cpp")
# include_directories("/usr/local/include/lcm/")   # lcm includes
# include_directories("/usr/local/include/eigen3")   # lcm includes
# include_directories(${CMAKE_BINARY_DIR})
# include_directories(${PROJECT_SOURCE_DIR})


# add_subdirectory(user)
# add_subdirectory(third-party)
include_directories("/usr/local/include/eigen3")   # lcm includes
file(GLOB_RECURSE sources
"src/*.cpp" 
"src/FSM/*.cpp" 

"user/Embedded_Controller/*.cpp"
"third-party/ParamHandler/*.cpp"
"third-party/qpOASES/*.cpp"
"third-party/Goldfarb_Optimizer/*.cpp"
"third-party/Goldfarb_Optimizer/*.cc"

# "third-party/osqp/*.cpp"
# "third-party/osqp/*.c"

"third-party/JCQP/*.cpp"
"third-party/JCQP/*.c"




)

add_compile_definitions(MANUAL)
# add_subdirectory(user)

SET (THIS_COM "../" )
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
  ${CMAKE_BINARY_DIR}/Configuration.h)
# "./*.cpp"


add_executable(legged_sim ${sources})

target_link_libraries(legged_sim glog lcm mujoco glfw qpOASES JCQP osqp GLEW  ${CURSES_LIBRARIES} glog gflags tinyxml2 ${TinyXML_LIBRARIES})

