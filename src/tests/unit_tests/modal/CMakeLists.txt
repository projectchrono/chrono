# Unit tests for the Chrono::Modal module
# ==================================================================

set(TESTS
    utest_MOD_eigensolver
    utest_MOD_curved_beam
)

#--------------------------------------------------------------

# A hack to set the working directory in which to execute the CTest
# runs.  This is needed for tests that need to access the Chrono data
# directory (since we use a relative path to it)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
  set(MY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(LIBRARIES Chrono_core Chrono_modal)

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBRARIES "Chrono_pardisomkl")
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Add unit test programs for MODAL module")

foreach(PROGRAM ${TESTS})
  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_include_directories(${PROGRAM} PRIVATE "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty")
  target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
  set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
endforeach()
