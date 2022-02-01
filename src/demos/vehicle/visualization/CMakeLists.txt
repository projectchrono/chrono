#=============================================================================
# CMake configuration file for demos of rendering Chrono::Vehicle simulations.
#=============================================================================

if(NOT ENABLE_MODULE_OPENGL)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS "")

if(ENABLE_MODULE_OPENGL)
    set(DEMOS ${DEMOS}
        demo_VEH_RenderJSON
    )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "")
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS}  ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    if(${CMAKE_SYSTEM_NAME} MATCHES "Emscripten")
		set (EM_GL_LINKERFLAGS "-s FULL_ES3=1 -s USE_GLFW=3")
    endif()
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
						  LINK_FLAGS "${LINKER_FLAGS} ${EM_GL_LINKERFLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM}
                          ChronoEngine
                          ChronoEngine_vehicle
                          ChronoModels_vehicle
                          ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
