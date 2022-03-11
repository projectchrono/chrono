
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
)

SET(LinearMath_SRCS
	cbtAlignedAllocator.cpp
	cbtConvexHull.cpp
	cbtConvexHullComputer.cpp
	cbtGeometryUtil.cpp
	cbtPolarDecomposition.cpp
	cbtQuickprof.cpp
	cbtSerializer.cpp
	cbtSerializer64.cpp
	cbtThreads.cpp
	cbtVector3.cpp
	TaskScheduler/cbtTaskScheduler.cpp
	TaskScheduler/cbtThreadSupportPosix.cpp
	TaskScheduler/cbtThreadSupportWin32.cpp
)

SET(LinearMath_HDRS
	cbtAabbUtil2.h
	cbtAlignedAllocator.h
	cbtAlignedObjectArray.h
	cbtConvexHull.h
	cbtConvexHullComputer.h
	cbtDefaultMotionState.h
	cbtGeometryUtil.h
	cbtGrahamScan2dConvexHull.h
	cbtHashMap.h
	cbtIDebugDraw.h
	cbtList.h
	cbtMatrix3x3.h
	cbtMinMax.h
	cbtMotionState.h
	cbtPolarDecomposition.h
	cbtPoolAllocator.h
	cbtQuadWord.h
	cbtQuaternion.h
	cbtQuickprof.h
	cbtRandom.h
	cbtScalar.h
	cbtSerializer.h
	cbtStackAlloc.h
	cbtThreads.h
	cbtTransform.h
	cbtTransformUtil.h
	cbtVector3.h
	TaskScheduler/cbtThreadSupportInterface.h
)

ADD_LIBRARY(LinearMath ${LinearMath_SRCS} ${LinearMath_HDRS})
SET_TARGET_PROPERTIES(LinearMath PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(LinearMath PROPERTIES SOVERSION ${BULLET_VERSION})

IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		#FILES_MATCHING requires CMake 2.6
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS LinearMath DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS LinearMath
					RUNTIME DESTINATION bin
					LIBRARY DESTINATION lib${LIB_SUFFIX}
					ARCHIVE DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h"  PATTERN
".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(LinearMath PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(LinearMath PROPERTIES PUBLIC_HEADER "${LinearMath_HDRS}")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
