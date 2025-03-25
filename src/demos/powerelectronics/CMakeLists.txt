if(NOT ENABLE_MODULE_POWERELECTRONICS)
    return()
endif()

#--------------------------------------------------------------
# List of all MBS demos

# Add demos that can work with poerelectronics
set(DEMOS
    demo_PE_MotorControl
    demo_PE_RCvalidation
    demo_PE_RLvalidation
    demo_PE_RLCvalidation)
#    demo_PowerElectronics_MotorControl_validation
#)

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#--------------------------------------------------------------
# Include nlohmann.json to export results in json format.
#--------------------------------------------------------------
include_directories(${CMAKE_INSTALL_PREFIX}/../../libreries/json-develop/include)

include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBS "ChronoEngine_vsg")
endif()

if(ENABLE_MODULE_POWERELECTRONICS)
    include_directories(${CH_INCLUDES} "${CH_INCLUDES}/chrono_powerelectronics")
    set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
    set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
    list(APPEND LIBS "ChronoEngine_powerelectronics")
endif()

#--------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Demo programs for MBS module...")

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  if(MSVC)
    target_compile_options(${PROGRAM} PRIVATE /bigobj)
  endif()
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})  

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
