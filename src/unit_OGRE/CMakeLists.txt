

option(ENABLE_UNIT_OGRE "Enable the Chrono Matlab module" OFF)

if(NOT ENABLE_UNIT_OGRE)
  return()
endif()

find_package(SDL2 REQUIRED)
find_package(MyGUI REQUIRED)
find_package(OGRE REQUIRED)

SET(Boost_USE_STATIC_LIBS ON)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(BOOST COMPONENTS thread system chrono date_time REQUIRED)


# SET(Boost_THREAD_LIBDIR "Boost_THREAD_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost thread lib dir")
# SET(Boost_SYSTEM_LIBDIR "Boost_SYSTEM_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost system lib dir")
# SET(Boost_DATE_TIME_LIBDIR "Boost_DATE_TIME_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost date_time lib dir")
# SET(Boost_CHRONO_LIBDIR "Boost_CHRONO_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost chrono lib dir")

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
	
# FIND_LIBRARY(Boost_THREADLIB_DEBUG
# 	NAMES libboost_thread-vc120-mt-gd-1_55
# 	PATHS "${Boost_THREAD_LIBDIR}/debug/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_THREADLIB_RELEASE
# 	NAMES libboost_thread-vc120-mt-1_55
# 	PATHS "${Boost_THREAD_LIBDIR}/release/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_SYSTEMLIB_DEBUG
# 	NAMES libboost_system-vc120-mt-gd-1_55
# 	PATHS "${Boost_SYSTEM_LIBDIR}/debug/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_SYSTEMLIB_RELEASE
# 	NAMES libboost_system-vc120-mt-1_55
# 	PATHS "${Boost_SYSTEM_LIBDIR}/release/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_DATE_TIMELIB_DEBUG
# 	NAMES libboost_date_time-vc120-mt-gd-1_55
# 	PATHS "${Boost_DATE_TIME_LIBDIR}/debug/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_DATE_TIMELIB_RELEASE
# 	NAMES libboost_date_time-vc120-mt-1_55
# 	PATHS "${Boost_DATE_TIME_LIBDIR}/release/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_CHRONOLIB_DEBUG
# 	NAMES libboost_chrono-vc120-mt-gd-1_55
# 	PATHS "${Boost_CHRONO_LIBDIR}/debug/link-static/threading-multi"
# 	)
	
# FIND_LIBRARY(Boost_CHRONOLIB_RELEASE
# 	NAMES libboost_chrono-vc120-mt-1_55
# 	PATHS "${Boost_CHRONO_LIBDIR}/release/link-static/threading-multi"
# 	)
	
# SET(Boost_threadlib optimized ${Boost_THREADLIB_RELEASE} debug ${Boost_THREADLIB_DEBUG})
# SET(Boost_systemlib optimized ${Boost_SYSTEMLIB_RELEASE} debug ${Boost_SYSTEMLIB_DEBUG})
# SET(Boost_date_timelib optimized ${Boost_DATE_TIMELIB_RELEASE} debug ${Boost_DATE_TIMELIB_DEBUG})
# SET(Boost_chronolib optimized ${Boost_CHRONOLIB_RELEASE} debug ${Boost_CHRONOLIB_DEBUG})
	

SET(LIBRARIES ${CHRONOENGINE_LIBRARIES} 
	${SDL_LIBRARY} 
	#${Boost_threadlib}
	#${Boost_systemlib}
	#${Boost_date_timelib}
	#${Boost_chronolib}
	${Boost_LIBRARIES} 
	${OGRE_LIBRARIES}
	${MyGUI_lib}
	${MyGUI_ogrelib}
	)

include_directories(
    ${CHRONOENGINE_INCLUDES}
	${OGRE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
	${MyGUI_INCLUDEDIR}
    "${CHRONOENGINE_INCLUDES}/collision" 
    "${CHRONOENGINE_INCLUDES}/collision/bullet" 
    )


ADD_SUBDIRECTORY(chrono_ogre)
