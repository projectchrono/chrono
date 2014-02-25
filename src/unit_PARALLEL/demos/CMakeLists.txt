#--------------------------------------------------------------
# Additional include paths

SET(UTILITIES ../utils/input_output.h)
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")


#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

ADD_EXECUTABLE(demo_ballDVI  demo_ballDVI.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_ballDVI.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_ballDVI PROPERTIES
	FOLDER demos
	COMPILE_FLAGS "${CH_BUILDFLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ballDVI ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_ballDVI ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_ballDEM  demo_ballDEM.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_ballDEM.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_ballDEM PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ballDEM ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_ballDEM ${CHRONO_LIB} ChronoEngine_Parallel)

install(TARGETS demo_ballDVI DESTINATION bin)
install(TARGETS demo_ballDEM DESTINATION bin)




