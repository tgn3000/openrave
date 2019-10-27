cmake_minimum_required(VERSION 2.9)
set(CMAKE_CXX_STANDARD 11)

file(GLOB ann_files ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
# for windows compilation, just exports the symbols (even though library is being built statically)
add_definitions(-DANN_STATIC_LIBRARY)
add_library(ANN STATIC ${ann_files} include/ANN/ANN.h include/ANN/ANNx.h)
set_target_properties(ANN PROPERTIES COMPILE_FLAGS "${EXTRA_COMPILE_FLAGS}")

install(TARGETS ANN DESTINATION lib)

set(ANN_FOUND ON CACHE BOOL PARENT_SCOPE)