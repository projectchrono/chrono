set(PROGRAM run_chrono)

#--------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_parsers)

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS Chrono_vsg)
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS Chrono_mumps)
endif()

if(CH_ENABLE_MODULE_FSI)
  list(APPEND LIBS Chrono_fsi)
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  list(APPEND LIBS Chrono_fsisph)
  if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_fsisph_vsg")
  endif()
endif()

if(CH_ENABLE_MODULE_FSI_TDPF)
  list(APPEND LIBS Chrono_fsitdpf)
  if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_fsitdpf_vsg")
  endif()
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  list(APPEND LIBS "Chrono_vehicle")
  if(CH_ENABLE_MODULE_VSG)
      list(APPEND LIBS "Chrono_vehicle_vsg")
  endif()
endif()

#--------------------------------------------------------------

# Add executable

message(STATUS "Add run_chrono program")

add_executable(${PROGRAM}  "${PROGRAM}.cpp")
source_group(""  FILES  "${PROGRAM}.cpp")

if(APPLE)
  set(CMAKE_MACOSX_BUNDLE OFF)
  set(MACOSX_BUNDLE_INFO_PLIST "${PROJECT_SOURCE_DIR}/cmake/MacOSXBundleInfo.plist.in")
  set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
endif()
if(MSVC)
  set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBS})  

install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
