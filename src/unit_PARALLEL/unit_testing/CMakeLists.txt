#--------------------------------------------------------------
# Additional include paths (for ChronoEngine_Utils)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}
                    ${CMAKE_BINARY_DIR}
)

# Libraries
SET(LIBRARIES
    ${CHRONO_LIB}
    ChronoEngine_Parallel
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

#IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#	SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
#ENDIF()

#--------------------------------------------------------------
# List of all executables

SET(TESTS
    test_real3
    test_real4
    test_mat33
    test_gravity
    test_jacobians
)

MESSAGE(STATUS "Executable programs...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_BUILDFLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

ENDFOREACH(PROGRAM)

#--------------------------------------------------------------
# Executables that use Bullet

SET(TESTS_B
    test_mpr
)

# Because Bullet classes are not DLL exported, the sources must be explicitly
# included.
IF (MSVC)
  SET(BULLET_FILES
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConeShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btConeShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h
        ${CHRONO_INC}/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CHRONO_INC}/collision/bullet/LinearMath/btAlignedAllocator.h
        ${CHRONO_INC}/collision/bullet/LinearMath/btAlignedAllocator.cpp
  )
  SOURCE_GROUP(bullet FILES ${BULLET_FILES})
ELSE()
    SET(BULLET_FILES "")
ENDIF()

FOREACH(PROGRAM ${TESTS_B})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  ${PROGRAM}.cpp ${BULLET_FILES})
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_BUILDFLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})

ENDFOREACH(PROGRAM)
