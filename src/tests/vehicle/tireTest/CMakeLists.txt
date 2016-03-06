# Require Irrlicht support
if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

# ------------------------------------------------------

set(TEST_FILES test_tire.cpp)

if(ENABLE_MODULE_FEA)
    set(TEST_FILES ${TEST_FILES}
        ANCFToroidalTire.h
        ANCFToroidalTire.cpp
    )
endif()

source_group("" FILES ${TEST_FILES})

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

message(STATUS "...add test_tire")
add_executable(test_tire ${TEST_FILES})
set_target_properties(test_tire PROPERTIES 
                      COMPILE_FLAGS "${COMPILER_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
target_link_libraries(test_tire ${LIBRARIES})

