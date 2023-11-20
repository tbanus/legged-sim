ExternalProject_Add(lcm
  GIT_REPOSITORY "https://github.com/lcm-proj/lcm.git"
  GIT_TAG "v1.5.0"
  SOURCE_DIR ${CMAKE_BINARY_DIR}/lcm
  BINARY_DIR ${CMAKE_BINARY_DIR}/lcm-build
  CMAKE_CACHE_ARGS
    -DFOO_ENABLE_BAR:BOOL=1
#   INSTALL_COMMAND ""
  )

#   ExternalProject_Add(mujoco
#   GIT_REPOSITORY "https://github.com/deepmind/mujoco.git"
#   GIT_TAG "2.3.7"
#   SOURCE_DIR ${CMAKE_BINARY_DIR}/mujoco
#   BINARY_DIR ${CMAKE_BINARY_DIR}/mujoco-build
#   CMAKE_CACHE_ARGS
#     -DFOO_ENABLE_BAR:BOOL=1
# #   INSTALL_COMMAND ""
#   )


  set (CPACK_DEBIAN_PACKAGE_DEPENDS "urdf glog (>0.6.0)")