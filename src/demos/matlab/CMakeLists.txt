
set(DEMOS
  demo_MTLB_matlab
  demo_MTLB_functions_plot
)

#--------------------------------------------------------------

message(STATUS "Add demo programs for MATLAB module")

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")
    
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${CH_MATLABLIB} ChronoEngine ChronoEngine_matlab)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
