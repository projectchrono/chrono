# List all FEA demos that do not use Irrlich
SET(FEA_DEMOS
	demo_FEAbasic
	demo_FEAdynamics
)

# List all FEA demos that use Irrlicht
SET(FEA_DEMOS_IRR
	demo_FEAthermal
	demo_FEAelectrostatics
	demo_FEAvisualize
	demo_FEAcontacts
)

#List all FEA demos that use both Irrlicht and Matlab
SET(FEA_DEMOS_IRR_MATLAB
	demo_FEAbeams
	demo_FEAbeamsconstr
	demo_FEAminiature
	demo_FEAshells
)



IF(ENABLE_UNIT_MKL)

	INCLUDE_DIRECTORIES( ${CH_MKLINC} )
	INCLUDE_DIRECTORIES( ${CH_EIGEN_PATH} )
	INCLUDE_DIRECTORIES( ${ChronoEngine_UNIT_MKL_INCLUDE})
	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

	ADD_EXECUTABLE(demo_FEAcables demo_FEAcables.cpp)
	# SOURCE_GROUP("" FILES demo_FEAcables.cpp)
	SET_TARGET_PROPERTIES(demo_FEAcables PROPERTIES 
		FOLDER demos
		LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
		)
	TARGET_LINK_LIBRARIES(demo_FEAcables
 		ChronoEngine
		ChronoEngine_FEA
 		ChronoEngine_MKL
		ChronoEngine_IRRLICHT	
		)
	ADD_DEPENDENCIES (demo_FEAcables ChronoEngine ChronoEngine_FEA ChronoEngine_MKL ChronoEngine_IRRLICHT)
	INSTALL(TARGETS demo_FEAcables DESTINATION bin)

ENDIF()



#--------------------------------------------------------------
# Add executables for demos that do not use Irrlicht

MESSAGE(STATUS "Demo programs for FEA module...")

FOREACH(PROGRAM ${FEA_DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

	ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
	SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
	SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
		FOLDER demos
		COMPILE_FLAGS "${CH_BUILDFLAGS}"
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(${PROGRAM}
		ChronoEngine
		ChronoEngine_FEA
		)
	ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEA)
	INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()


#--------------------------------------------------------------
# For demo executables that use Irrlicht (and posibly Matlab)

IF(ENABLE_UNIT_IRRLICHT)

	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	
	FOREACH(PROGRAM ${FEA_DEMOS_IRR})
	    MESSAGE(STATUS "...add ${PROGRAM}")

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
			ChronoEngine_FEA
			ChronoEngine_IRRLICHT
			)
		ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEA ChronoEngine_IRRLICHT)
		INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
	
	ENDFOREACH()

	IF(ENABLE_UNIT_MATLAB)

		INCLUDE_DIRECTORIES( ${CH_MATLABINC} )

		FOREACH(PROGRAM ${FEA_DEMOS_IRR_MATLAB})
		    MESSAGE(STATUS "...add ${PROGRAM}")
		    
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
				ChronoEngine_FEA
				ChronoEngine_IRRLICHT
				ChronoEngine_MATLAB
				)
			ADD_DEPENDENCIES(${PROGRAM} ChronoEngine ChronoEngine_FEA ChronoEngine_IRRLICHT ChronoEngine_MATLAB)
			INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
		
		ENDFOREACH()

	ENDIF()

ENDIF()

