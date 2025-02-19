#--------------------------------------------------------------
# Add executables

set(DEMOS
  demo_CH_coords
  demo_CH_linalg
  demo_CH_matrix_ref
  demo_CH_math
  demo_CH_archive
  demo_CH_buildsystem
  demo_CH_powertrain
  demo_CH_functions
  demo_CH_solver
  demo_CH_EulerAngles
  demo_CH_filesystem
)

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS "Chrono_mumps")
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for CORE module")

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)

