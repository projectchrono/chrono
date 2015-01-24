# List all FEM demos that do not use Irrlich
SET(FEM_DEMOS
	demo_FEMbasic
	demo_FEMdynamics
)

# List all FEM demos that use Irrlicht
SET(FEM_DEMOS_IRR
	demo_FEMthermal
	demo_FEMelectrostatics
	demo_FEMvisualize
)

#List all FEM demos that use both Irrlicht and Matlab
SET(FEM_DEMOS_IRR_MATLAB
	demo_FEMbeams
	demo_FEMbeamsconstr
	demo_FEMminiature
)

#--------------------------------------------------------------
# Add executables for demos that do not use Irrlicht

FOREACH(PROGRAM ${FEM_DEMOS})

	ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
	SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
	SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
		FOLDER demos
		COMPILE_FLAGS "${CH_BUILDFLAGS}"
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(${PROGRAM}
		ChronoEngine
		ChronoEngine_FEM
		)
	ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEM)
	INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()


#--------------------------------------------------------------
# For demo executables that use Irrlicht (and posibly Matlab)

IF(ENABLE_UNIT_IRRLICHT)

	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	FOREACH(PROGRAM ${FEM_DEMOS_IRR})
	
		ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
		SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
			FOLDER demos
			COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
			)
		TARGET_LINK_LIBRARIES(${PROGRAM}
			${CH_IRRLICHTLIB}
			ChronoEngine
			ChronoEngine_FEM
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT)
		INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
	
	ENDFOREACH()

	IF(ENABLE_UNIT_MATLAB)

		INCLUDE_DIRECTORIES( ${CH_MATLABINC} )

		FOREACH(PROGRAM ${FEM_DEMOS_IRR_MATLAB})
		
			ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
			SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
			SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
				FOLDER demos
				COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
				LINK_FLAGS "${CH_LINKERFLAG_EXE}"
				)
			TARGET_LINK_LIBRARIES(${PROGRAM}
				${CH_IRRLICHTLIB}
				${CH_MATLABLIB}
				ChronoEngine
				ChronoEngine_FEM
				ChronoEngine_IRRLICHT
				ChronoEngine_MATLAB
				)
			ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEM ChronoEngine_IRRLICHT ChronoEngine_MATLAB)
			INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
		
		ENDFOREACH()

	ENDIF()

ENDIF()

