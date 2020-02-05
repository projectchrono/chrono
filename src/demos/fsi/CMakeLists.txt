# ------------------------------------------------------------------------------
# List of demos according their depencies
# ------------------------------------------------------------------------------

# List FSI demos that do not have other dependencies
SET(FSI_DEMOS
  demo_FSI_DamBreak
  demo_FSI_Compressibility
  demo_FSI_Poiseuille_flow
  demo_FSI_Flexible_Shell
  demo_FSI_CylinderDrop
)

#--------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_FSI_INCLUDES})

SET(COMPILER_FLAGS "${CH_CXX_FLAGS}")
SET(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
LIST(APPEND LIBS "")

IF(ENABLE_MODULE_MKL)
  INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
  SET(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  SET(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  LIST(APPEND LIBS "ChronoEngine_mkl")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for FSI module...")

FOREACH(PROGRAM ${FSI_DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${COMPILER_FLAGS}"
    LINK_FLAGS "${LINKER_FLAGS}")
  TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_fsi ${LIBS})

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
