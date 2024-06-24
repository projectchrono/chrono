# Unit tests for the Chrono::Modal module
# ==================================================================

message(STATUS "Unit tests for Chrono::Modal...")

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

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
set(LIBRARIES ChronoEngine ChronoEngine_modal)

if(ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBRARIES "ChronoEngine_pardisomkl")
endif()

#--------------------------------------------------------------
# Add executables

foreach(PROGRAM ${TESTS})
  message(STATUS "...${PROGRAM}")
  
  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM}  PROPERTIES
    FOLDER tests
    COMPILE_FLAGS "${COMPILER_FLAGS}"
    LINK_FLAGS "${LINKER_FLAGS}")
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_include_directories(${PROGRAM} PRIVATE "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty")
  target_link_libraries(${PROGRAM} ${LIBRARIES} gtest_main)

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

  add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
  set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${MY_WORKING_DIR})
endforeach()
