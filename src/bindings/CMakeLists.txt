# -*- coding: utf-8 -*-
cmake_minimum_required(VERSION 2.8.9)
project(openravepy)
set(CMAKE_CXX_STANDARD 11)

if(NOT OPENRAVE_DISABLE_ASSERT_HANDLER)
  add_definitions("-DBOOST_ENABLE_ASSERT_HANDLER") # turns segfault into exception
endif()

set(OPENRAVE_PYTHON_BINDING_NAME openravepy_int)
add_definitions(${Boost_CFLAGS})
set(CMAKE_REQUIRED_INCLUDES ${Python3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${REQUIRED_INCLUDES})
set(CMAKE_REQUIRED_LIBRARIES ${Python3_LIBRARY} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY})
set(CMAKE_REQUIRED_FLAGS ${Boost_CFLAGS} "-DCMAKE_BUILD_TYPE:STRING=Release")

set(HAVE_ALL_PYTHON_HEADERS ON)
set(HAVE_ALL_PYTHON_HEADERS ${HAVE_ALL_PYTHON_HEADERS} PARENT_SCOPE)
if( HAVE_ALL_PYTHON_HEADERS )
  message(STATUS "python and boost-python found")
  include_directories(${Python3_INCLUDE_DIR} ${Python3_NUMPY_INCLUDE_DIR}
    ${OPENRAVE_INCLUDE_LOCAL_DIRS} ${OPENRAVE_CORE_INCLUDE_LOCAL_DIRS}
  )
  
  set(extralibs)
  if( CLOCK_GETTIME_FOUND )
    set(extralibs ${extralibs} rt)
  endif()

  set(OPENRAVEPY_COMPILE_FLAGS "-DOPENRAVE_CORE_DLL")
  if( MSVC )
    check_cxx_compiler_flag("/bigobj" COMPILER_HAS_BIGOBJ)
    if( COMPILER_HAS_BIGOBJ )
      set(OPENRAVEPY_COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS} /bigobj")
    endif()
  endif()
  ## openrave bindings
  add_library(${OPENRAVE_PYTHON_BINDING_NAME}
    SHARED openravepy_kinbody.cpp openravepy_int.cpp openravepy_robot.cpp openravepy_global.cpp
    openravepy_collisionchecker.cpp openravepy_controller.cpp openravepy_ikparameterization.cpp 
    openravepy_iksolver.cpp openravepy_module.cpp openravepy_physicsengine.cpp openravepy_planner.cpp 
    openravepy_planningutils.cpp openravepy_sensor.cpp openravepy_sensorsystem.cpp openravepy_spacesampler.cpp 
    openravepy_trajectory.cpp openravepy_viewer.cpp bindings.cpp docstrings.cpp
  )
  # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/Segfault_in_cxa_allocate_exception#Workaround)
  if(APPLE)
    set(Python3_MINOR_VERSION "7")
  else()
    set(Python3_MINOR_VERSION "8")
  endif()

  target_link_libraries(${OPENRAVE_PYTHON_BINDING_NAME} 
    PUBLIC ${STDC_LIBRARY} openrave openrave_core
    boost_python3${Python3_MINOR_VERSION} boost_numpy3${Python3_MINOR_VERSION} 
    dl util ${Python3_LIBRARY} ${CMAKE_THREAD_LIBS_INIT}
    # ${Python3_LIBRARY} ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${extralibs}
    # ${Python3_NUMPY_LIBRARY}
  )
  # PRIVATE boost_assertion_failed)
  # add_dependencies(${OPENRAVE_PYTHON_BINDING_NAME} libopenrave libopenrave-core)
  set_target_properties(${OPENRAVE_PYTHON_BINDING_NAME} 
    PROPERTIES PREFIX "" SUFFIX ""
    COMPILE_FLAGS "${OPENRAVEPY_COMPILE_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}"
  )
  if( WIN32 )
    set_target_properties(${OPENRAVE_PYTHON_BINDING_NAME} PROPERTIES SUFFIX ".pyd")
  elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # on mac osx, python cannot import libraries with .dylib extension
    set_target_properties(${OPENRAVE_PYTHON_BINDING_NAME} PROPERTIES SUFFIX ".so")
  endif()
  install(TARGETS ${OPENRAVE_PYTHON_BINDING_NAME}
    DESTINATION ${Python3_SITELIB}/openravepy
    COMPONENT ${COMPONENT_PREFIX}python
  )

  ## ann bindings
  if( ANN_FOUND )
    add_library(pyANN_int SHARED pyann.cpp bindings.cpp bindings.h)
    include_directories(${CMAKE_SOURCE_DIR}/3rdparty/ann/include)
    # include_directories(${ANN_INCLUDE_DIR})
    # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/  Segfault_in_cxa_allocate_exception#Workaround)
    target_link_libraries(pyANN_int 
      PUBLIC ANN ${STDC_LIBRARY} openrave openrave_core
      boost_python3${Python3_MINOR_VERSION} boost_numpy3${Python3_MINOR_VERSION} 
      dl util ${Python3_LIBRARY} ${CMAKE_THREAD_LIBS_INIT}
    ) # PRIVATE boost_assertion_failed)
    set_target_properties(pyANN_int 
      PROPERTIES PREFIX "" COMPILE_FLAGS "${ANN_CFLAGS}"
    )
    add_dependencies(pyANN_int ANN)
    if( WIN32 )
      set_target_properties(pyANN_int 
        PROPERTIES SUFFIX ".pyd"
      )
      if( MSVC )
        # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
        # this is such an unbelievable hack, that it's disgusting
        set_target_properties(pyANN_int 
          PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true"
        )
      endif()
    elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      # on mac osx, python cannot import libraries with .dylib extension
      set_target_properties(pyANN_int PROPERTIES SUFFIX ".so")
    endif()
    install(TARGETS pyANN_int DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
  endif()

  ## convexdecomposition bindings
  if( CONVEXDECOMPOSITION_FOUND )
    add_library(convexdecompositionpy SHARED convexdecompositionpy.cpp bindings.cpp bindings.h)
    include_directories(${CONVEXDECOMPOSITION_INCLUDE_DIR})
    # stdc++ has to be included before opengl libraries due to some ATI bug (http://wiki.fifengine.de/  Segfault_in_cxa_allocate_exception#Workaround)
    target_link_libraries(convexdecompositionpy 
      PUBLIC ${STDC_LIBRARY} convexdecomposition ${Python3_LIBRARY} 
      ${Boost_PYTHON_LIBRARY} ${Boost_THREAD_LIBRARY} ${extralibs}
    ) # PRIVATE boost_assertion_failed)
    set_target_properties(convexdecompositionpy 
      PROPERTIES PREFIX "" COMPILE_FLAGS "${CONVEXDECOMPOSITION_CFLAGS}"
    )
    add_dependencies(convexdecompositionpy convexdecomposition)
    if( WIN32 )
      set_target_properties(convexdecompositionpy
        PROPERTIES SUFFIX ".pyd"
      )
      if( MSVC )
        # set "link library dependencies" for visual studio in order to include symbols for other statically linked libs
        # this is such an unbelievable hack, that it's disgusting
        set_target_properties(convexdecompositionpy
          PROPERTIES STATIC_LIBRARY_FLAGS "\" LinkLibraryDependencies=\"true"
        )
      endif()
    elseif( APPLE OR ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      # on mac osx, python cannot import libraries with .dylib extension
      set_target_properties(convexdecompositionpy
        PROPERTIES SUFFIX ".so"
      )
    endif()
    install(TARGETS convexdecompositionpy DESTINATION ${OPENRAVEPY_VER_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}python)
  endif()
else(HAVE_ALL_PYTHON_HEADERS)
  message(STATUS "failed to use boost python libraries, check if python-numpy is installed.")
endif(HAVE_ALL_PYTHON_HEADERS)
