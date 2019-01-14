# ------------------------------------------------------------------------------
# List of demos according their depencies
# ------------------------------------------------------------------------------

# List all FSI demos that use chrono
SET(FSI_DEMOS 
demo_FSI_DamBreak
demo_FSI_Compressibility
demo_FSI_Poiseuille_flow
demo_FSI_Flexible_Shell
demo_FSI_CylinderDrop
)

#add fluid demos + chrono_mkl here
SET(FSI_MKL_DEMOS 
)

#--------------------------------------------------------------
SET(LIBRARIES
    ChronoEngine
    ChronoEngine_fsi
)
message( STATUS "FSI_DEMO_LIBRARY: ${ChronoEngine_fsi}")
INCLUDE_DIRECTORIES(${CH_FSI_INCLUDES})


MESSAGE(STATUS "Demo programs for FSI module...")
# Add executables for demos that have no other dependencies
IF(ENABLE_MODULE_MKL)
       INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})

        FOREACH(PROGRAM ${FSI_MKL_DEMOS})
            MESSAGE(STATUS "...add ${PROGRAM}")

            CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
            SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")    
            SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                FOLDER demos
                COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MKL_CXX_FLAGS} ${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
                LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS} ${CH_LINK_FLAGS} ")
            TARGET_LINK_LIBRARIES(${PROGRAM}
            	${LIBRARIES}
                ChronoEngine
                ChronoEngine_mkl
                ChronoEngine_fsi)
            TARGET_INCLUDE_DIRECTORIES(${PROGRAM} PUBLIC
			${CH_MKL_INCLUDES})
            ADD_DEPENDENCIES(${PROGRAM}
             	${LIBRARIES}
                ChronoEngine
                ChronoEngine_mkl
                ChronoEngine_fsi)
            INSTALL(TARGETS ${PROGRAM} DESTINATION bin)
        ENDFOREACH()
	
ENDIF()
	
FOREACH(PROGRAM ${FSI_DEMOS})
	    MESSAGE(STATUS "...add ${PROGRAM}")

	    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
	    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

	    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
			FOLDER demos
			COMPILE_FLAGS "${CH_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	    )

	    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
	    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

	    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()





