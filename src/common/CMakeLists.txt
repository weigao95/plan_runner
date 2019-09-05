file(GLOB COMMON_SRC *.h *.hpp *.cpp ./robot_plan/*.cpp)
source_group("src" FILES ${COMMON_SRC})
add_library(runner_common_lib ${COMMON_SRC} ${HEADER_FILES})
target_link_libraries(runner_common_lib ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(runner_common_lib drake::drake)

# install library
install(TARGETS runner_common_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)