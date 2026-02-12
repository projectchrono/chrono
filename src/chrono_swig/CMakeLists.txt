add_subdirectory(chrono_python)
add_subdirectory(chrono_csharp)

if(NOT CH_ENABLE_MODULE_PYTHON AND NOT ENABLE_MODULE_CSHARP)
    return()
endif()

# Optionally install SWIG interface files
option(CH_INSTALL_SWIG_INTERFACE "Install the Chrono SWIG interface files" OFF)

if(CH_INSTALL_SWIG_INTERFACE)
    message(STATUS "\n==== Chrono SWIG module ====\n")

    file(COPY ${CMAKE_SOURCE_DIR}/src/chrono_swig/interface/ DESTINATION ${CMAKE_BINARY_DIR}/chrono_swig)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_swig/interface/ DESTINATION include/chrono_swig)

    message(STATUS "Chrono SWIG interface files copied to ${CMAKE_BINARY_DIR}/chrono_swig")
    message(STATUS "Chrono SWIG interface will be installed to ${CMAKE_INSTALL_PREFIX}/include/chrono_swig")
endif()
