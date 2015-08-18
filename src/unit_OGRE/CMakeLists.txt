cmake_minimum_required(VERSION 2.8)
project(ChOgre)
#set path for local find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/SBELUtils.cmake)
#enable_openmp_support()
#find_package(OGRE REQUIRED)
find_package(CHRONOENGINE REQUIRED)
find_package(SDL REQUIRED)

SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(BOOST COMPONENTS thread REQUIRED)



# User specified path to ChronoEngine
#SET(CHRONO_INC           ""    CACHE   PATH    "Where is ChronoEngine Source located")
# User specified path to ChronoEngine Libraries
#SET(CHRONO_LIB_PATH           ""   CACHE    PATH    "Where is ChronoEngine Lib located")
# Find required Libraries
#FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)

SET(OGRE_INCLUDEDIR "OGRE_INCLUDEDIR-NOT_FOUND" CACHE PATH "Path to Ogre include dir")
SET(OGRE_LIBDIR_DEBUG "OGRE_LIBDIR_DEBUG-NOT_FOUND" CACHE PATH "Path to Ogre debug libs")
SET(OGRE_LIBDIR_RELEASE "OGRE_LIBDIR_RELEASE-NOT_FOUND" CACHE PATH "Path to Ogre release libs")

SET(Boost_THREAD_LIBDIR "Boost_THREAD_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost thread lib dir")
SET(Boost_SYSTEM_LIBDIR "Boost_SYSTEM_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost system lib dir")
SET(Boost_DATE_TIME_LIBDIR "Boost_DATE_TIME_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost date_time lib dir")
SET(Boost_CHRONO_LIBDIR "Boost_CHRONO_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost chrono lib dir")

SET(MyGUI_LIBDIR_DEBUG "MyGUI_LIBDIR_DEBUG-NOT_FOUND" CACHE PATH "Path to MyGUI debug lib dir")
SET(MyGUI_LIBDIR_RELEASE "MyGUI_LIBDIR_RELEASE-NOT_FOUND" CACHE PATH "Path to MyGUI release lib dir")
SET(MyGUI_INCLUDEDIR "MyGUI_INCLUDEDIR-NOT_FOUND" CACHE PATH "Path to MyGUI include dir")

IF (EXISTS "${MyGUI_INCLUDEDIR}/MYGUI/MyGUI.h")
	include_directories(${MyGUI_INCLUDEDIR})
ELSE()
	SET(MyGUI_INCLUDEDIR "MyGUI_INCLUDEDIR-NOT_FOUND")
ENDIF()

FIND_LIBRARY(MyGUI_LIB_DEBUG
	NAMES MyGUIEngine_d
	PATHS "${MyGUI_LIBDIR_DEBUG}/"
	)
	
FIND_LIBRARY(MyGUI_LIB_RELEASE
	NAMES MyGUIEngine
	PATHS "${MyGUI_LIBDIR_RELEASE}/"
	)
	
SET(MyGUI_lib optimized ${MyGUI_LIB_RELEASE} debug ${MyGUI_LIB_DEBUG})
	
FIND_LIBRARY(MyGUI_LIBOGRE_DEBUG
	NAMES MyGUI.OgrePlatform_d
	PATHS "${MyGUI_LIBDIR_DEBUG}/"
	)
	
FIND_LIBRARY(MyGUI_LIBOGRE_RELEASE
	NAMES MyGUI.OgrePlatform
	PATHS "${MyGUI_LIBDIR_RELEASE}/"
	)
	
SET(MyGUI_ogrelib optimized ${MyGUI_LIBOGRE_RELEASE} debug ${MyGUI_LIBOGRE_DEBUG})
	


FIND_LIBRARY(Boost_THREADLIB_DEBUG
	NAMES libboost_thread-vc120-mt-gd-1_55
	PATHS "${Boost_THREAD_LIBDIR}/debug/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_THREADLIB_RELEASE
	NAMES libboost_thread-vc120-mt-1_55
	PATHS "${Boost_THREAD_LIBDIR}/release/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_SYSTEMLIB_DEBUG
	NAMES libboost_system-vc120-mt-gd-1_55
	PATHS "${Boost_SYSTEM_LIBDIR}/debug/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_SYSTEMLIB_RELEASE
	NAMES libboost_system-vc120-mt-1_55
	PATHS "${Boost_SYSTEM_LIBDIR}/release/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_DATE_TIMELIB_DEBUG
	NAMES libboost_date_time-vc120-mt-gd-1_55
	PATHS "${Boost_DATE_TIME_LIBDIR}/debug/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_DATE_TIMELIB_RELEASE
	NAMES libboost_date_time-vc120-mt-1_55
	PATHS "${Boost_DATE_TIME_LIBDIR}/release/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_CHRONOLIB_DEBUG
	NAMES libboost_chrono-vc120-mt-gd-1_55
	PATHS "${Boost_CHRONO_LIBDIR}/debug/link-static/threading-multi"
	)
	
FIND_LIBRARY(Boost_CHRONOLIB_RELEASE
	NAMES libboost_chrono-vc120-mt-1_55
	PATHS "${Boost_CHRONO_LIBDIR}/release/link-static/threading-multi"
	)
	
SET(Boost_threadlib optimized ${Boost_THREADLIB_RELEASE} debug ${Boost_THREADLIB_DEBUG})
SET(Boost_systemlib optimized ${Boost_SYSTEMLIB_RELEASE} debug ${Boost_SYSTEMLIB_DEBUG})
SET(Boost_date_timelib optimized ${Boost_DATE_TIMELIB_RELEASE} debug ${Boost_DATE_TIMELIB_DEBUG})
SET(Boost_chronolib optimized ${Boost_CHRONOLIB_RELEASE} debug ${Boost_CHRONOLIB_DEBUG})
	
	
IF(EXISTS "${OGRE_INCLUDEDIR}/OGRE/Ogre.h")
	include_directories(${OGRE_INCLUDEDIR})
ELSE()
	SET(OGRE_INCLUDEDIR "OGRE_INCLUDEDIR-NOT_FOUND")
ENDIF()

FIND_LIBRARY(OGRE_MAINLIB_DEBUG
	NAMES OgreMain_d
	PATHS ${OGRE_LIBDIR_DEBUG}
	)
	
FIND_LIBRARY(OGRE_PAGINGLIB_DEBUG
	NAMES OgrePaging_d
	PATHS ${OGRE_LIBDIR_DEBUG}
	)
	
FIND_LIBRARY(OGRE_PROPERTYLIB_DEBUG
	NAMES OgreProperty_d
	PATHS ${OGRE_LIBDIR_DEBUG}
	)
	
FIND_LIBRARY(OGRE_RTSHADERLIB_DEBUG
	NAMES OgreRTShaderSystem_d
	PATHS ${OGRE_LIBDIR_DEBUG}
	)
	
FIND_LIBRARY(OGRE_MAINLIB_RELEASE
	NAMES OgreMain
	PATHS ${OGRE_LIBDIR_RELEASE}
	)
	
FIND_LIBRARY(OGRE_PAGINGLIB_RELEASE
	NAMES OgrePaging
	PATHS ${OGRE_LIBDIR_RELEASE}
	)
	
FIND_LIBRARY(OGRE_PROPERTYLIB_RELEASE
	NAMES OgreProperty
	PATHS ${OGRE_LIBDIR_RELEASE}
	)
	
FIND_LIBRARY(OGRE_RTSHADERLIB_RELEASE
	NAMES OgreRTShaderSystem
	PATHS ${OGRE_LIBDIR_RELEASE}
	)
	
SET(OGRE_LIBRARIES optimized ${OGRE_MAINLIB_RELEASE} optimized ${OGRE_PAGINGLIB_RELEASE} optimized ${OGRE_PROPERTYLIB_RELEASE} optimized ${OGRE_RTSHADERLIB_RELEASE} 
	debug ${OGRE_MAINLIB_DEBUG} debug ${OGRE_PAGINGLIB_DEBUG} debug ${OGRE_PROPERTYLIB_DEBUG} debug ${OGRE_RTSHADERLIB_DEBUG}
	)


SET(LIBRARIES ${CHRONOENGINE_LIBRARIES} 
	${SDL_LIBRARY} 
	${Boost_threadlib}
	${Boost_systemlib}
	${Boost_date_timelib}
	${Boost_chronolib}
	${Boost_LIBRARIES} 
	${OGRE_LIBRARIES}
	${MyGUI_lib}
	${MyGUI_ogrelib}
	)

include_directories(
    ${CHRONOENGINE_INCLUDES}
	"${OGRE_INCLUDEDIR}/OGRE"
	${Boost_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
	${MyGUI_INCLUDEDIR}
    "${CHRONOENGINE_INCLUDES}/collision" 
    "${CHRONOENGINE_INCLUDES}/collision/bullet" 
    )




	
ADD_SUBDIRECTORY(source/ChOgre)
ADD_SUBDIRECTORY(source/demo_ball_plane)
ADD_SUBDIRECTORY(source/demo_wheel)
ADD_SUBDIRECTORY(source/demo_gui)
ADD_SUBDIRECTORY(source/demo_brick_collide)
ADD_SUBDIRECTORY(source/demo_vehicle)
ADD_SUBDIRECTORY(source/demo_heightmap)
