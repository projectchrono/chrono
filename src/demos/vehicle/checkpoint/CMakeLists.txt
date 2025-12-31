#--------------------------------------------------------------
# Chrono libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS "Chrono_mumps")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBS "Chrono_postprocess")
endif()

#--------------------------------------------------------------
# Create the executable

set(PROGRAM demo_VEH_Checkpoint)
set(FILES 
    demo_VEH_Checkpoint.cpp
    ../WheeledVehicleModels.h
    ../../SetChronoSolver.h
)

add_executable(${PROGRAM} ${FILES})
source_group("" FILES ${FILES})

if(APPLE)
  set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
endif()
if(MSVC)
  set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBS})

install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

