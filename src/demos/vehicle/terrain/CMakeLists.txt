#=============================================================================
# CMake configuration file for demos of various Chrono::Vehicle terrain models.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_DeformableSoil
    demo_VEH_RigidTerrain
)

if(HAVE_OPENCRG AND ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
        demo_VEH_CRGTerrain_IRR
    )
endif()

if(HAVE_OPENCRG AND ENABLE_MODULE_VSG)
    set(DEMOS ${DEMOS}
        demo_VEH_CRGTerrain_VSG
    )
endif()

if(ENABLE_MODULE_MULTICORE AND ENABLE_MODULE_OPENGL)
    set(DEMOS ${DEMOS}
        demo_VEH_GranularTerrain
        demo_VEH_MovingPatch
    )
endif()

if(ENABLE_MODULE_PARDISO_MKL)
    set(DEMOS ${DEMOS}
        demo_VEH_DeformableSoilAndTire
    )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)

    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()

if(ENABLE_MODULE_MULTICORE)
    include_directories(${CH_MULTICORE_INCLUDES})
    set(COMPILER_FLAGS "${CH_CXX_FLAGS}  ${CH_MULTICORE_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_multicore")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_MUMPS)
    include_directories(${CH_MUMPS_INCLUDES})
    list(APPEND LIBS "ChronoEngine_mumps")
endif()

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
