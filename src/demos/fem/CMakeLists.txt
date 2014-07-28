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


ADD_EXECUTABLE(demo_FEMdynamics  	demo_FEMdynamics.cpp)
SOURCE_GROUP(demos\\fem FILES  		demo_FEMdynamics.cpp)
SET_TARGET_PROPERTIES(demo_FEMdynamics PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_FEMdynamics 
 	ChronoEngine
	ChronoEngine_FEM
	)
ADD_DEPENDENCIES (demo_FEMdynamics ChronoEngine ChronoEngine_FEM)

install(TARGETS demo_FEMdynamics DESTINATION bin)



# For demo executables that use Irrlicht:

IF (ENABLE_UNIT_IRRLICHT)

	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	INCLUDE_DIRECTORIES( ${CH_MATLABINC} )
	
	
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
		ChronoEngine_IRRLICHT
		)
	ADD_DEPENDENCIES (demo_meshless ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

	install(TARGETS demo_meshless DESTINATION bin)
	
	ADD_EXECUTABLE(demo_FEMthermal  	demo_FEMthermal.cpp)
	SOURCE_GROUP(demos\\fem FILES  		demo_FEMthermal.cpp)
	SET_TARGET_PROPERTIES(demo_FEMthermal PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_FEMthermal 
		${CH_IRRLICHTLIB}
		ChronoEngine
		ChronoEngine_FEM
		ChronoEngine_IRRLICHT
		)
	ADD_DEPENDENCIES (demo_FEMthermal ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

	install(TARGETS demo_FEMthermal DESTINATION bin)
	
	# for demos that use Matlab
	IF(ENABLE_UNIT_MABLAB)
	
		ADD_EXECUTABLE(demo_FEMvisualize  	demo_FEMvisualize.cpp)
		SOURCE_GROUP(demos\\fem FILES  		demo_FEMvisualize.cpp)
		SET_TARGET_PROPERTIES(demo_FEMvisualize PROPERTIES 
			FOLDER demos
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
			)
		TARGET_LINK_LIBRARIES(demo_FEMvisualize 
			${CH_IRRLICHTLIB}
			${CH_MATLABLIB}
			ChronoEngine
			ChronoEngine_FEM
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES (demo_FEMvisualize ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

		install(TARGETS demo_FEMvisualize DESTINATION bin)
	
	
		ADD_EXECUTABLE(demo_FEMbeams  		demo_FEMbeams.cpp)
		SOURCE_GROUP(demos\\fem FILES  		demo_FEMbeams.cpp)
		SET_TARGET_PROPERTIES(demo_FEMbeams PROPERTIES 
			FOLDER demos
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
			)
		TARGET_LINK_LIBRARIES(demo_FEMbeams 
			${CH_IRRLICHTLIB}
			${CH_MATLABLIB}
			ChronoEngine
			ChronoEngine_FEM
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES (demo_FEMbeams ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

		install(TARGETS demo_FEMbeams DESTINATION bin)

		
		ADD_EXECUTABLE(demo_FEMbeamsconstr  demo_FEMbeamsconstr.cpp)
		SOURCE_GROUP(demos\\fem FILES  		demo_FEMbeamsconstr.cpp)
		SET_TARGET_PROPERTIES(demo_FEMbeamsconstr PROPERTIES 
			FOLDER demos
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
			)
		TARGET_LINK_LIBRARIES(demo_FEMbeamsconstr 
			${CH_IRRLICHTLIB}
			${CH_MATLABLIB}
			ChronoEngine
			ChronoEngine_FEM
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES (demo_FEMbeamsconstr ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

		install(TARGETS demo_FEMbeamsconstr DESTINATION bin)
	
	
		ADD_EXECUTABLE(demo_FEMminiature demo_FEMminiature.cpp)
		SOURCE_GROUP(demos\\fem FILES  		demo_FEMminiature.cpp)
		SET_TARGET_PROPERTIES(demo_FEMminiature PROPERTIES 
			FOLDER demos
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
			)
		TARGET_LINK_LIBRARIES(demo_FEMminiature 
			${CH_IRRLICHTLIB}
			${CH_MATLABLIB}
			ChronoEngine
			ChronoEngine_FEM
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES (demo_FEMminiature ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

		install(TARGETS demo_FEMminiature DESTINATION bin)
	
	ENDIF()
	
	ADD_EXECUTABLE(demo_FEMelectrostatics  		demo_FEMelectrostatics.cpp)
	SOURCE_GROUP(demos\\fem FILES  		demo_FEMelectrostatics.cpp)
	SET_TARGET_PROPERTIES(demo_FEMelectrostatics PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_FEMelectrostatics 
		${CH_IRRLICHTLIB}
		ChronoEngine
		ChronoEngine_FEM
		ChronoEngine_IRRLICHT
		)
	ADD_DEPENDENCIES (demo_FEMelectrostatics ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)

	install(TARGETS demo_FEMelectrostatics DESTINATION bin)
	
	

ENDIF()