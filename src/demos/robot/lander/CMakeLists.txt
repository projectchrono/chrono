#=============================================================================
# CMake configuration file for the ROBOT lander demos.
#=============================================================================

set(DEMOS demo_ROBOT_Lander)
if(CH_ENABLE_MODULE_FSI_SPH)
   set(DEMOS ${DEMOS} demo_ROBOT_Lander_CRM)
endif()

set(MODEL_FILES
    model/Lander.cpp
    model/Lander.h
)

#--------------------------------------------------------------

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_FSI_SPH)
    list(APPEND LIBS Chrono_fsisph)
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_vsg)
    if(CH_ENABLE_MODULE_FSI_SPH)
        list(APPEND LIBS Chrono_fsisph_vsg)
    endif()
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MUMPS)
    list(APPEND LIBS Chrono_mumps)
endif()

#--------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
    source_group("" FILES ${PROGRAM}.cpp)
    source_group("model" FILES ${MODEL_FILES})

    target_include_directories(${PROGRAM} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
        set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
        set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

endforeach()
