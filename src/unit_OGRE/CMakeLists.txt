

option(ENABLE_UNIT_OGRE "Enable the Chrono Matlab module" OFF)

if(NOT ENABLE_UNIT_OGRE)
  return()
endif()

find_package(SDL2 REQUIRED)
find_package(MyGUI REQUIRED)
find_package(Ogre REQUIRED)

#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_MULTITHREADED ON)
#SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS thread system chrono date_time REQUIRED)


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


SET(LIBRARIES ${CHRONOENGINE_LIBRARIES} 
	${SDL2_LIBRARY} 
	#${Boost_threadlib}
	#${Boost_systemlib}
	#${Boost_date_timelib}
	#${Boost_chronolib}
	${Boost_LIBRARIES} 
	${OGRE_LIBRARIES}
	${MYGUI_LIBRARIES}
	${MyGUI_ogrelib}
	)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
	${OGRE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
	${MyGUI_INCLUDEDIR}
	${SDL2_INCLUDE_DIR}
    "${CHRONOENGINE_INCLUDES}/collision" 
    "${CHRONOENGINE_INCLUDES}/collision/bullet" 
    )


ADD_SUBDIRECTORY(chrono_ogre)
