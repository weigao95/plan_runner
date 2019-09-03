# Collect the library file
file(GLOB KUKA_LCM_SRC *.h *.hpp *.cpp)

# Exclude the test source
file(GLOB KUKA_LCM_APP_SRC *_node.cpp)
exclude(KUKA_LCM_SRC "${KUKA_LCM_SRC}" ${KUKA_LCM_APP_SRC})

# Build the lib
add_library(kuka_lcm_lib ${KUKA_LCM_SRC} ${HEADER_FILES})
target_link_libraries(kuka_lcm_lib drake::drake)

# Add applications
foreach(app_src ${KUKA_LCM_APP_SRC})
    get_filename_component(app_name ${app_src} NAME_WE)
    add_executable(${app_name} "${app_src}")
    target_link_libraries(${app_name} runner_common_lib)
    target_link_libraries(${app_name} kuka_lcm_lib)
    target_link_libraries(${app_name} runner_simrobot_lib)
    target_link_libraries(${app_name} ${YAMLCPP_LIBRARY})
    target_link_libraries(${app_name} ${catkin_LIBRARIES})
    target_link_libraries(${app_name} ${GTEST_BOTH_LIBRARIES})
    target_link_libraries(${app_name} ${CMAKE_THREAD_LIBS_INIT})
    install(TARGETS ${app_name}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
endforeach()