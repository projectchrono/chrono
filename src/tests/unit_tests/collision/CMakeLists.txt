set(TESTS
    utest_COLL_bullet_utils
)

if (CHRONO_THRUST_FOUND)
   set(TESTS ${TESTS}
       utest_COLL_narrow_prims
       utest_COLL_narrow_mpr
   )
endif()

# Because Bullet classes are not DLL exported, the sources must be explicitly included.
IF (MSVC)
  SET(BULLET_FILES
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCollisionShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexInternalShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtPolyhedralConvexShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConvexPolyhedron.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtBoxShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtConeShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylinderShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCylindricalShellShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/BulletCollision/CollisionShapes/cbtSphereShape.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.h
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtAlignedAllocator.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtGeometryUtil.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtVector3.cpp
        ${CMAKE_SOURCE_DIR}/src/chrono/collision/bullet/LinearMath/cbtConvexHullComputer.cpp
  )
  SOURCE_GROUP(bullet FILES ${BULLET_FILES})
ELSE()
  SET(BULLET_FILES "")
ENDIF()

set(LIBS Chrono_core)

MESSAGE(STATUS "Add unit test programs for COLLISION")
build_utests(YES "${TESTS}" "${LIBS}" "${BULLET_FILES}")


