

option(ENABLE_UNIT_OGRE "Enable the Chrono Matlab module" OFF)

if(NOT ENABLE_UNIT_OGRE)
  return()
endif()

find_package(SDL2 REQUIRED)

find_package(Ogre REQUIRED)
find_package(MyGUI REQUIRED)

#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_MULTITHREADED ON)
#SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS thread system chrono date_time REQUIRED)


# SET(Boost_THREAD_LIBDIR "Boost_THREAD_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost thread lib dir")
# SET(Boost_SYSTEM_LIBDIR "Boost_SYSTEM_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost system lib dir")
# SET(Boost_DATE_TIME_LIBDIR "Boost_DATE_TIME_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost date_time lib dir")
# SET(Boost_CHRONO_LIBDIR "Boost_CHRONO_LIBDIR-NOT_FOUND" CACHE PATH "Path to Boost chrono lib dir")



SET(LIBRARIES 
    ChronoEngine
	${SDL2_LIBRARY} 
	#${Boost_threadlib}
	#${Boost_systemlib}
	#${Boost_date_timelib}
	#${Boost_chronolib}
	${Boost_LIBRARIES} 
	${OGRE_LIBRARIES}
	${MYGUI_LIBRARIES}
	${MYGUI_PLATFORM_LIBRARIES}
	)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
	${OGRE_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${SDL_INCLUDE_DIR}
	${MYGUI_INCLUDE_DIRS}
	${SDL2_INCLUDE_DIR}
    "${CHRONOENGINE_INCLUDES}/collision" 
    "${CHRONOENGINE_INCLUDES}/collision/bullet" 
    )


ADD_SUBDIRECTORY(chrono_ogre)
