set(MKL_DEMOS
    demo_MKL_MklEngine
 )

# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MKL_INCLUDES} )

# ---------------------------------------------------------------------
MESSAGE(STATUS "Demo programs for MKL module...")

FOREACH(PROGRAM ${MKL_DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          FOLDER demos
                          LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}")
    TARGET_LINK_LIBRARIES(${PROGRAM} ChronoEngine ChronoEngine_mkl)
    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()

