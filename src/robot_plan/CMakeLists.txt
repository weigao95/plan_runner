file(GLOB ROBOPLAN_SRC *.h *.hpp *.cpp)
add_library(runner_robotplan_lib ${ROBOPLAN_SRC} ${HEADER_FILES})
target_link_libraries(runner_robotplan_lib ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(runner_robotplan_lib drake::drake)

# install library
install(TARGETS runner_robotplan_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)