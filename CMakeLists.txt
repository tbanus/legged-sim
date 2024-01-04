# include_directories("./")
include_directories("./include")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("./lcm-types/cpp")

include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes

file(GLOB_RECURSE sources
"src/*.cpp" 
)
# "./*.cpp"


add_executable(legged_sim ${sources})

target_link_libraries(legged_sim glog  lcm mujoco glfw GLEW  ${CURSES_LIBRARIES} glog gflags urdf tinyxml2 ${TinyXML_LIBRARIES})

