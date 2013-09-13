#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_FEMbasic  	demo_FEMbasic.cpp)
SOURCE_GROUP(demos\\fem FILES  		demo_FEMbasic.cpp)
SET_TARGET_PROPERTIES(demo_FEMbasic PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_FEMbasic 
 	ChronoEngine
	ChronoEngine_FEM
	)
ADD_DEPENDENCIES (demo_FEMbasic ChronoEngine ChronoEngine_FEM)

install(TARGETS demo_FEMbasic DESTINATION bin)


# For demo executables that use Irrlicht:

IF (ENABLE_UNIT_IRRLICHT)

	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	
	
	ADD_EXECUTABLE(demo_meshless  	demo_meshless.cpp)
	SOURCE_GROUP(demos\\fem FILES  		demo_meshless.cpp)
	SET_TARGET_PROPERTIES(demo_meshless PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_meshless 
		${CH_IRRLICHTLIB}
		ChronoEngine
		ChronoEngine_FEM
		)
	ADD_DEPENDENCIES (demo_meshless ChronoEngine ChronoEngine_FEM)

	install(TARGETS demo_meshless DESTINATION bin)

ENDIF()