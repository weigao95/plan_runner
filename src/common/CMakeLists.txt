file(GLOB COMMON_SRC *.h *.hpp *.cpp)
source_group("src" FILES ${COMMON_SRC})
add_library(common_lib ${COMMON_SRC} ${HEADER_FILES})
target_link_libraries(common_lib ${Boost_FILESYSTEM_LIBRARY})