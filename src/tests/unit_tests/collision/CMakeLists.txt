set(LIBRARIES ChronoEngine)
include_directories( ${CH_INCLUDES} )

set(TESTS
    utest_COLL_bullet_utils
)

if (${THRUST_FOUND})
   set(TESTS ${TESTS}
       utest_COLL_narrow_prims
       utest_COLL_narrow_mpr
   )
endif()

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
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylindricalShellShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/btCylindricalShellShape.cpp
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

message(STATUS "Unit test programs for COLLISION...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  ${PROGRAM}.cpp ${BULLET_FILES})
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
endforeach(PROGRAM)


