#--------------------------------------------------------------
# Additional include paths (for ChronoEngine_Utils)
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")

# Libraries (ChronoEngine, ChronoEngine_Parallel, and chronoEngine_Utils)
SET(LIBRARIES
    ${CHRONO_LIB}
    ChronoEngine_Parallel
    ChronoEngine_Utils
    )

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

ADD_EXECUTABLE(demo_ball  demo_ball.cpp)
SOURCE_GROUP(""  FILES demo_ball.cpp)
SET_TARGET_PROPERTIES(demo_ball PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_ball ${LIBRARIES})
ADD_DEPENDENCIES(demo_ball ${LIBRARIES})

ADD_EXECUTABLE(demo_box  demo_box.cpp)
SOURCE_GROUP(""  FILES demo_box.cpp)
SET_TARGET_PROPERTIES(demo_box PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_box ${LIBRARIES})
ADD_DEPENDENCIES(demo_box ${LIBRARIES})

ADD_EXECUTABLE(demo_capsule  demo_capsule.cpp)
SOURCE_GROUP(""  FILES demo_capsule.cpp)
SET_TARGET_PROPERTIES(demo_capsule PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_capsule ${LIBRARIES})
ADD_DEPENDENCIES(demo_capsule ${LIBRARIES})

ADD_EXECUTABLE(demo_mesh  demo_mesh.cpp)
SOURCE_GROUP(""  FILES demo_mesh.cpp)
SET_TARGET_PROPERTIES(demo_mesh PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_mesh ${LIBRARIES})
ADD_DEPENDENCIES(demo_mesh ${LIBRARIES})

ADD_EXECUTABLE(demo_soilbin  demo_soilbin.cpp)
SOURCE_GROUP(""  FILES demo_soilbin.cpp)
SET_TARGET_PROPERTIES(demo_soilbin PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_soilbin ${LIBRARIES})
ADD_DEPENDENCIES(demo_soilbin ${LIBRARIES})

ADD_EXECUTABLE(demo_crater  demo_crater.cpp)
SOURCE_GROUP(""  FILES demo_crater.cpp)
SET_TARGET_PROPERTIES(demo_crater PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_crater ${LIBRARIES})
ADD_DEPENDENCIES(demo_crater ${LIBRARIES})

ADD_EXECUTABLE(demo_foam  demo_foam.cpp)
SOURCE_GROUP(""  FILES demo_foam.cpp)
SET_TARGET_PROPERTIES(demo_foam PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_foam ${LIBRARIES})
ADD_DEPENDENCIES(demo_foam ${LIBRARIES})

ADD_EXECUTABLE(demo_wheel  demo_wheel.cpp)
SOURCE_GROUP(""  FILES demo_wheel.cpp)
SET_TARGET_PROPERTIES(demo_wheel PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_wheel ${LIBRARIES})
ADD_DEPENDENCIES(demo_wheel ${LIBRARIES})

ADD_EXECUTABLE(demo_utils  demo_utils.cpp)
SOURCE_GROUP(""  FILES demo_utils.cpp)
SET_TARGET_PROPERTIES(demo_utils PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_utils ${LIBRARIES})
ADD_DEPENDENCIES(demo_utils ${LIBRARIES})

ADD_EXECUTABLE(test_scaling  test_scaling.cpp)
SOURCE_GROUP(""  FILES test_scaling.cpp)
SET_TARGET_PROPERTIES(test_scaling PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_scaling ${LIBRARIES})
ADD_DEPENDENCIES(test_scaling ${LIBRARIES})

ADD_EXECUTABLE(test_collision  test_collision.cpp)
SOURCE_GROUP(""  FILES test_collision.cpp)
SET_TARGET_PROPERTIES(test_collision PROPERTIES
  FOLDER demos
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_collision ${LIBRARIES})
ADD_DEPENDENCIES(test_collision ${LIBRARIES})

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



