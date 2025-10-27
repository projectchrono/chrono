if(NOT CH_ENABLE_MODULE_VEHICLE OR NOT CH_ENABLE_MODULE_VEHICLE_MODELS)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_VEH_hmmwvDLC
    btest_VEH_hmmwvSCM
    btest_VEH_m113Acc
    )

# ------------------------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)
list(APPEND LIBS ChronoModels_vehicle)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS Chrono_irrlicht)
  list(APPEND LIBS Chrono_vehicle_irrlicht)
endif()

# ------------------------------------------------------------------------------

message(STATUS "Add benchmark test programs for VEHICLE module")
build_btests("${TESTS}" "${LIBS}")
