#--------------------------------------------------------------
set(FEA_TESTS
    test_FEA_shellANCF
    test_FEA_4HumveeTires
    test_FEA_CorotationalTire
    test_FEA_ANCFHumveeTire
)

set (COMMON_FILES
    ../BaseTest.h
 )

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "")

if(ENABLE_MODULE_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_mkl")
endif()

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "${CH_IRRLICHTLIB}")
endif()

message(STATUS "Additional test programs for FEA module...")

#--------------------------------------------------------------

foreach(PROGRAM ${FEA_TESTS})
    message(STATUS "...add ${PROGRAM}")

    set(PROGRAM_FILES "${PROGRAM}.cpp" ${COMMON_FILES})
    add_executable(${PROGRAM}  ${PROGRAM_FILES})
    source_group(""  FILES  ${PROGRAM_FILES})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )

    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_fea ${LIBS})

endforeach()

