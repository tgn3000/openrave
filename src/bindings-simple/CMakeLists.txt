include_directories(
	${BOOST_INCLUDEDIR}  # boost
	${Python3_INCLUDE_DIR} # python3
	${Python3_NUMPY_INCLUDE_DIR} # numpy
)
link_directories(
	${BOOST_LIBRARYDIR} # boost
	# ${Python3_LIBRARY}  # python3
	${BOOST_PYTHON3_LIBRARYDIR} # numpy
)
add_definitions(-DOPENRAVE_BININGS_PYARRAY)
add_executable(bindings bindings.cpp)

if(APPLE)
    set(Python3_MINOR_VERSION "7")
else()
    set(Python3_MINOR_VERSION "8")
endif()

target_link_libraries(bindings
	PUBLIC ${STDC_LIBRARY} openrave openrave_core
	boost_python3${Python3_MINOR_VERSION} boost_numpy3${Python3_MINOR_VERSION} 
	dl util ${Python3_LIBRARY} ${CMAKE_THREAD_LIBS_INIT}
)
