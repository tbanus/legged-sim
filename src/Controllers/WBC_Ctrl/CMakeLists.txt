# # FILE(GLOB_RECURSE headers "*.hpp")
# # FILE(GLOB_RECURSE sources *.cpp)
# file(GLOB_RECURSE sources
# "../../../../src/*.cpp" 
# "../../../../user/Embedded_Controller/*.cpp"
# "../../../../third-party/ParamHandler/*.cpp"
# "../../../../third-party/qpOASES/*.cpp"
# "../../../../third-party/Goldfarb_Optimizer/*.cpp"
# # "third-party/JCQP/*.cpp"


# )

# add_library (WBC_Ctrl SHARED ${sources} )
# target_link_libraries (WBC_Ctrl lcm)
# target_link_libraries (WBC_Ctrl WBIC dynacore_param_handler)

