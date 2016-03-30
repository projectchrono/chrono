# Various test rigs for testing tire models
# Required: Chrono::Irrlicht module
# Optional: Chrono::FEA and Chrono::MKL modules
# ------------------------------------------------------

if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

# ------------------------------------------------------

set(TESTS
    test_VEH_quarterVehicle
    test_VEH_tireRig
)

# ------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

include_directories(${CH_IRRLICHTINC})

set(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle)

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

if (ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_mkl)
endif()

# ------------------------------------------------------
# Add all tests in this suite

foreach(PROGRAM ${TESTS})
  message(STATUS "...add ${PROGRAM}")
  source_group("" FILES "${PROGRAM}.cpp")
  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  set_target_properties(${PROGRAM} PROPERTIES 
                        COMPILE_FLAGS "${COMPILER_FLAGS}"
                        LINK_FLAGS "${LINKER_FLAGS}")
  target_link_libraries(${PROGRAM} ${LIBRARIES})
endforeach()
