#--------------------------------------------------------------
# Files for comparing SynChrono scaling performance to baseline Chrono::Vehicle performance
#
# Requires the SynChrono and Chrono::Vehicle modules 
#--------------------------------------------------------------

if(NOT CH_ENABLE_MODULE_VEHICLE OR NOT CH_ENABLE_MODULE_SYNCHRONO)
  return()
endif()  

# ------------------------------------------------------------------------------

set(TESTS
    btest_SCM_SYNscaling
    btest_SCM_VEHscaling
    )

# ------------------------------------------------------------------------------

set(LIBS
    Chrono_core
    Chrono_vehicle
    Chrono_synchrono
    ChronoModels_vehicle
    )

if (CH_ENABLE_MODULE_IRRLICHT)
    set(LIBS ${LIBS} Chrono_irrlicht Chrono_vehicle_irrlicht)
endif()

include_directories(${SYN_INCLUDES})

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for SCM scaling")
build_btests("${TESTS}" "${LIBS}")
