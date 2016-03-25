# Test for a vehicle with four toroidal tires
# Required: Chrono::FEA and Chrono::Irrlicht modules
# Optional: Chrono::MKL module
# ------------------------------------------------------

if(NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

# ------------------------------------------------------

set(TEST_FILES
    test_4ToroidalTires.cpp
    )

source_group("" FILES ${TEST_FILES})

# ------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

include_directories(${CH_IRRLICHTINC})

set(LIBRARIES
    ChronoEngine
    ChronoEngine_fea
    ChronoEngine_irrlicht
    ChronoEngine_vehicle)

if (ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_mkl)
endif()

# ------------------------------------------------------

message(STATUS "...add test_4ToroidalTires")
add_executable(test_4ToroidalTires ${TEST_FILES})
set_target_properties(test_4ToroidalTires PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(test_4ToroidalTires ${LIBRARIES})

