#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES(${CH_MATLABINC})
				    
#--------------------------------------------------------------
# Add executables

SET(DEMOS
  demo_matlab
  demo_chfunctions_plot
)

MESSAGE(STATUS "Demo programs for MATLAB module...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_BUILDFLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${CH_MATLABLIB} ChronoEngine ChronoEngine_MATLAB)
    ADD_DEPENDENCIES(${PROGRAM} ChronoEngine)

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)
