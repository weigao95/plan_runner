file(GLOB SUPERVISOR_SRC *.h *.hpp *.cpp)
add_library(runner_supervisor_lib ${SUPERVISOR_SRC} ${HEADER_FILES})
target_link_libraries(runner_supervisor_lib ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(runner_supervisor_lib drake::drake)
target_link_libraries(runner_supervisor_lib runner_common_lib)
target_link_libraries(runner_supervisor_lib runner_robotplan_lib)

# install library
install(TARGETS runner_supervisor_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)