option(ENABLE_MODULE_OGRE "Enable the Chrono Ogre module" OFF)

if(NOT ENABLE_MODULE_OGRE)
  return()
endif()

message(STATUS "==== Chrono Ogre module ====")

find_package(SDL2 REQUIRED)
find_package(Ogre REQUIRED)
find_package(MyGUI REQUIRED)

#SET(Boost_USE_STATIC_LIBS ON)
#SET(Boost_USE_MULTITHREADED ON)
#SET(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS thread system chrono date_time REQUIRED)

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

set(CH_OGRE_INCLUDES
      ${OGRE_INCLUDE_DIRS}
       ${Boost_INCLUDE_DIR}
       ${SDL_INCLUDE_DIR}
       ${MYGUI_INCLUDE_DIRS}
       ${SDL2_INCLUDE_DIR})
set(CH_OGRE_INCLUDES "${CH_OGRE_INCLUDES}" PARENT_SCOPE)
include_directories(${CH_OGRE_INCLUDES})


SET(ROOT_INCLUDE
	ChOgreApi.h
	)
	
SET(CORE_FILES
	Core/ChOgreApplication.cpp
	Core/ChOgreApplication.h
	Core/ChOgreCallback.h
	)
	
SET(GRAPHICS_FILES
	Graphics/ChOgreBody.cpp
	Graphics/ChOgreBody.h
	Graphics/ChOgreCameraManager.cpp
	Graphics/ChOgreCameraManager.h
	Graphics/ChOgreCamera.cpp
	Graphics/ChOgreCamera.h
	Graphics/ChOgreScene.cpp
	Graphics/ChOgreScene.h
	Graphics/ChOgreMeshBase.cpp
	Graphics/ChOgreMeshBase.h
	Graphics/ChOgreModel.cpp
	Graphics/ChOgreModel.h
	Graphics/ChOgreLight.cpp
	Graphics/ChOgreLight.h
	)
	
SET(UTIL_FILES
	Util/ChOgreBodyHandle.h
	Util/ChOgreBodyHandle.cpp
	Util/ChOgreLightHandle.h
	Util/ChOgreLightHandle.cpp
	)

SET(GUI_FILES
	GUI/ChOgreGUIManager.cpp
	GUI/ChOgreGUIManager.h
	GUI/ChOgreGUIButton.cpp
	GUI/ChOgreGUIButton.h
	GUI/ChOgreGUICallback.h
	GUI/ChOgreGUIElement.cpp
	GUI/ChOgreGUIElement.h
	GUI/ChOgreGUIText.cpp
	GUI/ChOgreGUIText.h
	)
	
SET(INPUT_FILES
	Input/ChOgre_SDLInputHandler.cpp
	Input/ChOgre_SDLInputHandler.h
	Input/ChOgreControllerCallback.h
	Input/ChOgreMouseCallback.h
	Input/ChOgreKeyboardCallback.h
	Input/ChOgreInputCallback.h
	Input/ChOgreWindowCallback.h
	Input/ChOgreInputDataStructures.h
	)
	
SET(TINYOBJLOADER_FILES
	${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.cc
	${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/tinyobjloader/tiny_obj_loader.h
	)
	
SOURCE_GROUP("" FILES ${ROOT_INCLUDE})
SOURCE_GROUP("Core" FILES ${CORE_FILES})
SOURCE_GROUP("Util" FILES ${UTIL_FILES})
SOURCE_GROUP("Graphics" FILES ${GRAPHICS_FILES})
SOURCE_GROUP("GUI" FILES ${GUI_FILES})
SOURCE_GROUP("Input" FILES ${INPUT_FILES})
SOURCE_GROUP("tinyobjloader" FILES ${TINYOBJLOADER_FILES})
	
ADD_LIBRARY(ChronoEngine_ogre SHARED 	
							${ROOT_INCLUDE} 
							${CORE_FILES} 
							${UTIL_FILES}
							${GRAPHICS_FILES} 
							${GUI_FILES} 
							${INPUT_FILES} 
							${TINYOBJLOADER_FILES}
			)
			
TARGET_LINK_LIBRARIES(ChronoEngine_ogre ${LIBRARIES})

  SET_TARGET_PROPERTIES(ChronoEngine_ogre PROPERTIES
    COMPILE_FLAGS "${CH_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_OGRE"
  )

INSTALL(TARGETS ChronoEngine_ogre
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib64
	ARCHIVE DESTINATION lib64
	)
	
