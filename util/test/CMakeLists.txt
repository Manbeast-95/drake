
add_mex(debugMexTest debugMexTest.cpp)

if (eigen3_FOUND)
  add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
  target_link_libraries(testDrakeGeometryUtil drakeGeometryUtil)
  include_directories( .. )
  add_test(NAME testDrakeGeometryUtil COMMAND testDrakeGeometryUtil)
endif(eigen3_FOUND)