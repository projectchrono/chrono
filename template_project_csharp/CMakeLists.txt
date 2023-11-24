cmake_minimum_required(VERSION 3.12)

project(MyCSharpProject 
        CSharp)

find_package(Chrono
             COMPONENTS CSharp
             CONFIG)


# Gather .cs files
file(GLOB CHRONO_CSHARP_SOURCE 
     LIST_DIRECTORIES false CONFIGURE_DEPENDS "${CHRONO_CSHARP_SOURCE_DIR}/*.cs")



# Add C# executable
add_executable(ChronoDemo
    main.cs
    ${CHRONO_CSHARP_SOURCE}
)

# set_target_properties(ChronoDemo PROPERTIES CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "v4.8")

source_group(Chrono FILES ${CHRONO_CSHARP_SOURCE}) # doesn't work

message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}")

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# DLLs will be copied into ${PROJECT_BINARY_DIR}/bin/${config} by default
# or in ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${config} if only CMAKE_RUNTIME_OUTPUT_DIRECTORY is set
# or to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG>} if the specific CMAKE_RUNTIME_OUTPUT_DIRECTORY_<CONFIG> has been set
#--------------------------------------------------------------

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "<mycustompathforrelease>")
add_DLL_copy_command()


# # getting Chrono DLLs - can be replaced with COPY_DLLS macro
# get_property(isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

# if (isMultiConfig)
#     string(REPLACE "$<CONFIG>" "Release" CHRONO_RUNTIME_DIR_RELEASE "${CHRONO_RUNTIME_DIR}")

#     foreach(CONF ${CMAKE_CONFIGURATION_TYPES})
#         # link always to Release version, since it is not possible to switch sources depending on release
#         string(REPLACE "$<CONFIG>" ${CONF} CHRONO_RUNTIME_DIR_CONFIG "${CHRONO_RUNTIME_DIR}")

#         foreach(CH_DLL ${CHRONO_DLL_NAMES})

#             if (EXISTS "${CHRONO_RUNTIME_DIR_CONFIG}/${CH_DLL}")
#                 message(STATUS "Exists: ${CHRONO_RUNTIME_DIR_CONFIG}/${CH_DLL}")
#                 target_sources(ChronoDemo PUBLIC
#                     "$<$<CONFIG:${CONF}>:${CHRONO_RUNTIME_DIR_CONFIG}/${CH_DLL}>"
#                 )
#             else()
#                 message(STATUS "WARNING: Linking ${CONF} to ${CHRONO_RUNTIME_DIR_RELEASE}/${CH_DLL}")
#                 target_sources(ChronoDemo PUBLIC
#                     "$<$<CONFIG:${CONF}>:${CHRONO_RUNTIME_DIR_RELEASE}/${CH_DLL}>"
#                 )
#             endif()

#         endforeach(CH_DLL ${CHRONO_DLL_NAMES})
#     endforeach(CONF ${CMAKE_CONFIGURATION_TYPES})

# else(isMultiConfig)

#     message(STATUS "NOT Multi-config")


#     foreach(CH_DLL ${CHRONO_DLL_NAMES})
#         message(STATUS "Exists: ${CHRONO_RUNTIME_DIR}/${CH_DLL}")
#         target_sources(ChronoDemo PUBLIC
#             "${CHRONO_RUNTIME_DIR}/${CH_DLL}"
#         )
#     endforeach(CH_DLL ${CHRONO_DLL_NAMES})

# endif(isMultiConfig)

