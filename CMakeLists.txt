SET(ROOT_INCLUDE
	ChOgre.h
	)
	
SET(CORE_FILES
	Core/ChOgreApplication.cpp
	Core/ChOgreApplication.h
	)
	
SET(GRAPHICS_FILES
	Graphics/ChOgreBody.cpp
	Graphics/ChOgreBody.h
	Graphics/ChOgreCameraManager.cpp
	Graphics/ChOgreCameraManager.h
	Graphics/ChOgreScene.cpp
	Graphics/ChOgreScene.h
	)
	
#SET(GUI_FILES
#	GUI/ChOgreGUIButton.cpp
#	GUI/ChOgreGUIButton.h
#	GUI/ChOgreGUIElement.cpp
#	GUI/ChOgreGUIEelement.h
#	GUI/ChOgreGUIManager.cpp
#	GUI/ChOgreGUIManager.h
#	GUI/ChOgreGUIPanel.cpp
#	GUI/ChOgreGUIPanel.h
#	GUI/ChOgreGUIText.cpp
#	GUI/ChOgreGUIText.h
#	)
	
SET(INPUT_FILES
	Input/ChOgre_SDLInputHandler.cpp
	Input/ChOgre_SDLInputHandler.h
	)
	
SET(TINYOBJLOADER_FILES
	tinyobjloader/tiny_obj_loader.cc
	tinyobjloader/tiny_obj_loader.h
	)
	
SOURCE_GROUP("" FILES ${ROOT_INCLUDE})
SOURCE_GROUP("Core" FILES ${CORE_FILES})
SOURCE_GROUP("Graphics" FILES ${GRAPHICS_FILES})
#SOURCE_GROUP("GUI" FILES ${GUI_FILES})
SOURCE_GROUP("Input" FILES ${INPUT_FILES})
SOURCE_GROUP("tinyobjloader" FILES ${TINYOBJLOADER_FILES})
	
ADD_DEFINITIONS(-D_CHOGRE_BUILD)
	
ADD_LIBRARY(ChOgre SHARED 	${ROOT_INCLUDE} 
							${CORE_FILES} 
							${GRAPHICS_FILES} 
							#${GUI_FILES} 
							${INPUT_FILES} 
							${TINYOBJLOADER_FILES}
			)
			
TARGET_LINK_LIBRARIES(ChOgre ${LIBRARIES})

set_target_properties( ChOgre
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/debug"
)

set_target_properties( ChOgre
    PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/release"
)

INSTALL(TARGETS ChOgre
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)