# Unit tests for the Chrono::Multicore module
# ==================================================================

#--------------------------------------------------------------
# Additional include paths (for ChronoEngine_Utils)
INCLUDE_DIRECTORIES(${CH_MULTICORE_INCLUDES})

# Libraries
SET(LIBRARIES
    ChronoEngine
    ChronoEngine_multicore
)

#--------------------------------------------------------------
# List of all executables

SET(TESTS
    utest_MCORE_real3
    utest_MCORE_real4
    #utest_MCORE_mat33
    utest_MCORE_matrix
    utest_MCORE_gravity
    #utest_MCORE_rhs
    utest_MCORE_r
    utest_MCORE_shafts
    utest_MCORE_rotmotors
    utest_MCORE_other_math
    #utest_MCORE_svd
    #utest_MCORE_collision_system
)

MESSAGE(STATUS "Unit test programs for MULTICORE module...")

FOREACH(PROGRAM ${TESTS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
 
#--------------------------------------------------------------
# Executables that use Bullet

SET(TESTS_B
    utest_MCORE_mpr
)

# Because Bullet classes are not DLL exported, the sources must be explicitly included.
IF (MSVC)
  SET(BULLET_FILES
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConeShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btConeShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btAlignedAllocator.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btAlignedAllocator.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btGeometryUtil.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btVector3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/btConvexHullComputer.cpp
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
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    #INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)


#--------------------------------------------------------------
# Executables that use OpenGL if it is available

IF (ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
ENDIF()

SET(TESTS_G
    utest_MCORE_linactuator
    utest_MCORE_bodyauxref
    utest_MCORE_joints_dvi
    utest_MCORE_narrowphase
    utest_MCORE_jacobians
    utest_MCORE_contact_forces
)

FOREACH(PROGRAM ${TESTS_G})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
ENDFOREACH(PROGRAM)
