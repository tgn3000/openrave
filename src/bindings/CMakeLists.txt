cmake_minimum_required(VERSION 2.8.9)
project(bindings)
set(CMAKE_CXX_STANDARD 11)

find_package (Threads REQUIRED)

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")

include_directories(
	${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/../python3/include/python3.7 
	${CMAKE_SOURCE_DIR}/../python3/lib/python3.7/site-packages/numpy/core/include
)
link_directories(
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/../python3/lib
)
add_definitions(-DOPENRAVE_BININGS_PYARRAY)

set(libraries_to_link boost_python37 boost_numpy37 dl util python3.7m ${CMAKE_THREAD_LIBS_INIT})

add_executable(bindings bindings.cpp)
set_target_properties(bindings
    PROPERTIES 
    COMPILE_FLAGS "-L${CMAKE_SOURCE_DIR}/lib -g -O0"
)

target_link_libraries(bindings ${libraries_to_link})

