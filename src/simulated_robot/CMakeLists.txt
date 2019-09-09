# Collect the library file
file(GLOB SIMROBOT_SRC *.h *.hpp *.cpp)

# Exclude the test source
file(GLOB SIMROBOT_APP_SRC *_node.cpp)
exclude(SIMROBOT_SRC "${SIMROBOT_SRC}" ${SIMROBOT_APP_SRC})

# Build the lib
add_library(runner_simrobot_lib ${SIMROBOT_SRC} ${HEADER_FILES})
target_link_libraries(runner_simrobot_lib drake::drake)

# install library
install(TARGETS runner_simrobot_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add applications
foreach(app_src ${SIMROBOT_APP_SRC})
    get_filename_component(app_name ${app_src} NAME_WE)
    add_executable(${app_name} "${app_src}")
    target_link_libraries(${app_name} runner_common_lib)
    target_link_libraries(${app_name} runner_supervisor_lib)
    target_link_libraries(${app_name} runner_simrobot_lib)
    target_link_libraries(${app_name} ${catkin_LIBRARIES})
    target_link_libraries(${app_name} ${GTEST_BOTH_LIBRARIES})
    target_link_libraries(${app_name} ${CMAKE_THREAD_LIBS_INIT})
    install(TARGETS ${app_name}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach()