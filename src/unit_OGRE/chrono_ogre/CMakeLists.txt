SET(ROOT_INCLUDE
	ChOgre.h
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
	tinyobjloader/tiny_obj_loader.cc
	tinyobjloader/tiny_obj_loader.h
	)
	
SOURCE_GROUP("" FILES ${ROOT_INCLUDE})
SOURCE_GROUP("Core" FILES ${CORE_FILES})
SOURCE_GROUP("Util" FILES ${UTIL_FILES})
SOURCE_GROUP("Graphics" FILES ${GRAPHICS_FILES})
SOURCE_GROUP("GUI" FILES ${GUI_FILES})
SOURCE_GROUP("Input" FILES ${INPUT_FILES})
SOURCE_GROUP("tinyobjloader" FILES ${TINYOBJLOADER_FILES})
	
ADD_DEFINITIONS(-D_CHOGRE_BUILD)
	
ADD_LIBRARY(ChOgre SHARED 	${ROOT_INCLUDE} 
							${CORE_FILES} 
							${UTIL_FILES}
							${GRAPHICS_FILES} 
							${GUI_FILES} 
							${INPUT_FILES} 
							${TINYOBJLOADER_FILES}
			)
			
TARGET_LINK_LIBRARIES(ChOgre ${LIBRARIES})

  SET_TARGET_PROPERTIES(ChOgre PROPERTIES
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_GPU}"
    COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_OGRE"
  )

INSTALL(TARGETS ChOgre
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)
	
