add_subdirectory(src)

if( NOT WIN32 AND OPENRAVE_BIN_SUFFIX )
  set(OPENRAVEPY_VER_NAME "_openravepy_${OPENRAVE_VERSION_MAJOR}_${OPENRAVE_VERSION_MINOR}")
else()
  set(OPENRAVEPY_VER_NAME "_openravepy_")
endif()
set(OPENRAVEPY_VER_INSTALL_DIR "${OPENRAVEPY_INSTALL_DIR}/${OPENRAVEPY_VER_NAME}") # used by openrave-config.cmake

add_subdirectory(python)

