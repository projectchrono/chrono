CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(OpenCRG)

set(MY_POSTFIX "")

IF(CMAKE_BUILD_TYPE MATCHES Debug) 
	set(MY_POSTFIX "d")
endif()

INCLUDE_DIRECTORIES(baselib/inc) 

SET ( LIBSOURCES
	baselib/src/crgContactPoint.c
	baselib/src/crgEvalpk.c
	baselib/src/crgEvaluv2xy.c
	baselib/src/crgEvalxy2uv.c
	baselib/src/crgEvalz.c
	baselib/src/crgLoader.c
	baselib/src/crgMgr.c
	baselib/src/crgMsg.c
	baselib/src/crgOptionMgmt.c
	baselib/src/crgPortability.c
	baselib/src/crgStatistics.c
	)

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	ADD_COMPILE_OPTIONS(-fPIC)
endif()

ADD_LIBRARY(OpenCRG${MY_POSTFIX} STATIC ${LIBSOURCES})

INSTALL(FILES baselib/inc/crgBaseLib.h
        DESTINATION include
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

INSTALL(TARGETS OpenCRG${MY_POSTFIX}
        DESTINATION lib
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
