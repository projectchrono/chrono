#--------------------------------------------------------------
# Additional include paths

SET(UTILITIES
	../utils/common.h
	../utils/input_output.h
	../utils/creators.h
	../utils/samplers.h
	../utils/generators.h)
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

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

ADD_EXECUTABLE(demo_ball  demo_ball.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_ball.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_ball PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ball ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_ball ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_box  demo_box.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_box.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_box PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_box ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_box ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_capsule  demo_capsule.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_capsule.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_capsule PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_capsule ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_capsule ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_mesh  demo_mesh.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_mesh.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_mesh PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_mesh ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_mesh ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_soilbin  demo_soilbin.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_soilbin.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_soilbin PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_soilbin ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_soilbin ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_crater  demo_crater.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_crater.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_crater PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_crater ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_crater ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_foam  demo_foam.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_foam.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_foam PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_foam ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_foam ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_wheel  demo_wheel.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_wheel.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_wheel PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_wheel ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_wheel ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(demo_utils  demo_utils.cpp ${UTILITIES})
SOURCE_GROUP(""  FILES demo_utils.cpp ${UTILITIES})
SET_TARGET_PROPERTIES(demo_utils PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_utils ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(demo_utils ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(test_scaling  test_scaling.cpp)
SOURCE_GROUP(""  FILES test_scaling.cpp)
SET_TARGET_PROPERTIES(test_scaling PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_scaling ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(test_scaling ${CHRONO_LIB} ChronoEngine_Parallel)

ADD_EXECUTABLE(test_collision  test_collision.cpp)
SOURCE_GROUP(""  FILES test_collision.cpp)
SET_TARGET_PROPERTIES(test_collision PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_collision ${CHRONO_LIB} ChronoEngine_Parallel)
ADD_DEPENDENCIES(test_collision ${CHRONO_LIB} ChronoEngine_Parallel)

install(TARGETS demo_ball DESTINATION bin)
install(TARGETS demo_box DESTINATION bin)
install(TARGETS demo_capsule DESTINATION bin)
install(TARGETS demo_mesh DESTINATION bin)
install(TARGETS demo_soilbin DESTINATION bin)
install(TARGETS demo_crater DESTINATION bin)
install(TARGETS demo_foam DESTINATION bin)
install(TARGETS demo_wheel DESTINATION bin)
install(TARGETS demo_utils DESTINATION bin)
install(TARGETS test_collision DESTINATION bin)
install(TARGETS test_scaling DESTINATION bin)



