# ------------------------------------------------------------------------------
# List of demos according their depencies
# ------------------------------------------------------------------------------

# List FSI demos that do not have other dependencies
SET(FSI_DEMOS 
demo_FSI_DamBreak
demo_FSI_Compressibility
demo_FSI_Poiseuille_flow
demo_FSI_CylinderDrop
demo_FSI_SingleWheelTest
)

# FSI demos that require Chrono::PardisoMKL
SET(FSI_MKL_DEMOS 
demo_FSI_Flexible_Elements
)

#--------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_FSI_INCLUDES})

MESSAGE(STATUS "Demo programs for FSI module...")
	
IF(ENABLE_MODULE_PARDISO_MKL)

   INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
   
    FOREACH(PROGRAM ${FSI_MKL_DEMOS})
        MESSAGE(STATUS "...add ${PROGRAM}")

        CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
        SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

        SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
            FOLDER demos
            COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_MKL_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS} ")
        SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
        TARGET_LINK_LIBRARIES(${PROGRAM}
            ChronoEngine
            ChronoEngine_pardisomkl
            ChronoEngine_fsi)
        TARGET_INCLUDE_DIRECTORIES(${PROGRAM} PUBLIC ${CH_MKL_INCLUDES})
        ADD_DEPENDENCIES(${PROGRAM}
            ChronoEngine
            ChronoEngine_pardisomkl
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
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_fsi)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
