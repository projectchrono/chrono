#=============================================================================
# CMake configuration file for the M113 demos.
# These example programs requires Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMOS
    demo_VEH_M113
    demo_VEH_M113_DefSoil
)

#--------------------------------------------------------------
# Compiler and linker flags

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "")

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_mkl")
endif()

#--------------------------------------------------------------
# Add executables

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    TARGET_LINK_LIBRARIES(${DEMO} 
                          ChronoEngine
                          ChronoEngine_irrlicht
                          ChronoEngine_vehicle
                          ChronoModels_vehicle
                          ${LIBS})
    INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})
    
ENDFOREACH()
