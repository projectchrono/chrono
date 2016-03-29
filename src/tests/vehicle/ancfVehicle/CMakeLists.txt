# Test for a vehicle with four toroidal tires
# Required: Chrono::FEA and Chrono::Irrlicht modules
# Optional: Chrono::MKL module
# ------------------------------------------------------

if(NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

# ------------------------------------------------------
# List all model files for this test

SET(MODEL_FILES
    ../ancfToroidalTire/ANCFToroidalTire.cpp
    ../ancfToroidalTire/ANCFToroidalTire.h
)

set(TEST_FILES
    test_4ToroidalTires.cpp
    )

SOURCE_GROUP("ToroidalTire" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${TEST_FILES})

# ------------------------------------------------------
# Additional include directories
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/tests/vehicle/ancfToroidalTire")

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
add_executable(test_4ToroidalTires ${TEST_FILES} ${MODEL_FILES})
set_target_properties(test_4ToroidalTires PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(test_4ToroidalTires ${LIBRARIES})

