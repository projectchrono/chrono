# Various test rigs for testing tire models
# ------------------------------------------------------------------------------

# Required: Chrono::Irrlicht
# Optional: Chrono::FEA, Chrono::MKL

if(NOT ENABLE_MODULE_IRRLICHT)
  return()
endif()  

# List all tests
set(VEH_TESTS_IRRLICHT
    test_VEH_quarterVehicle
    test_VEH_tireRig
)

# ------------------------------------------------------------------------------
# Set compiler and linker flags and libraries for the required modules
# (Chrono::Engine, Chrono::Vehicle, Chrono::Irrlicht), as well as optional modules
# (Chrono::FEA, Chrono::MKL, Chrono::OpenGL)

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

include_directories(${CH_IRRLICHTINC})

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_mkl)
endif()

# ------------------------------------------------------------------------------
# Build all tests

foreach(PROGRAM ${VEH_TESTS_IRRLICHT})
    message(STATUS "...add ${PROGRAM}")
    source_group("" FILES "${PROGRAM}.cpp")
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    set_target_properties(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${PROGRAM} ${LIBRARIES} ChronoEngine_irrlicht)
endforeach()
