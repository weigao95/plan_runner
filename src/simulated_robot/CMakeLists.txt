file(GLOB SIMROBOT_SRC *.h *.hpp *.cpp)
add_library(simrobot_lib ${SIMROBOT_SRC} ${HEADER_FILES})
target_link_libraries(simrobot_lib drake::drake)