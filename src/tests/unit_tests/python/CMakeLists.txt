message(STATUS "\n==== PyChrono Python Unit Tests ====\n")

if(NOT Python3_EXECUTABLE)
	find_package(Python3 REQUIRED COMPONENTS Interpreter)
endif()

function(build_pyutests ADD_TESTS TESTS LIBS)
	# A hack to set the working directory in which to execute the CTest runs.
	# This is needed for tests that need to access the Chrono data directory (since we use a relative path to it)
	if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
	else()
		set(MY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}")
	endif()

	if(WIN32)
		set(_py_path_sep ";")
	else()
		set(_py_path_sep ":")
	endif()

  	set(_pythonpath "${PYTHONPATH_BUILD}${_py_path_sep}$ENV{PYTHONPATH}")

	if(WIN32)
		# Escape semicolons for Windows environment variables
		string(REPLACE ";" "\\;" _pythonpath "${_pythonpath}")
	endif()

	foreach(PROGRAM ${TESTS})
		if(${ADD_TESTS})
			add_test(
				NAME ${PROGRAM}
				COMMAND "${Python3_EXECUTABLE}" -m pytest "${CMAKE_CURRENT_SOURCE_DIR}/${PROGRAM}.py"
			)

			set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY "${MY_WORKING_DIR}")
			set_tests_properties(${PROGRAM} PROPERTIES
				WORKING_DIRECTORY "${MY_WORKING_DIR}"
				ENVIRONMENT "PYTHONPATH=${_pythonpath}"
			)
		endif()
	endforeach()

endfunction()

#--------------------------------------------------------------
# Propagate to subdirectories

option(BUILD_TESTING_BASE "Build PyChrono unit tests for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_TESTING_BASE)
if(BUILD_TESTING_BASE)
  ADD_SUBDIRECTORY(core)
  ADD_SUBDIRECTORY(physics)
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  option(BUILD_TESTING_VEHICLE "Build PyChrono unit tests for Vehicle module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_VEHICLE)
  if(BUILD_TESTING_VEHICLE)
    ADD_SUBDIRECTORY(vehicle)
  endif()
endif()

if(CH_ENABLE_MODULE_SENSOR)
  option(BUILD_TESTING_SENSOR "Build PyChrono unit tests for Sensor module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_SENSOR)
  if(BUILD_TESTING_SENSOR)
    ADD_SUBDIRECTORY(sensor)
  endif()
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  option(BUILD_TESTING_FSI_SPH "Build PyChrono unit tests for FSI_SPH module" TRUE)
  mark_as_advanced(FORCE BUILD_TESTING_FSI_SPH)
  if(BUILD_TESTING_FSI_SPH)
    ADD_SUBDIRECTORY(fsi/sph)
  endif()
endif()

