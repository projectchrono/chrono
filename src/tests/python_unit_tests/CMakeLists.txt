message(STATUS "\n==== PyChrono Python Unit Tests ====\n")

set(CH_PYUTEST_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(build_pyutests ADD_TESTS TESTS LIBS)

	if(NOT Python3_EXECUTABLE)
		find_package(Python3 REQUIRED COMPONENTS Interpreter)
	endif()

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

	# Set up runtime loader path so extension modules can find Chrono shared libraries.
	set(_libdir "${PROJECT_BINARY_DIR}/lib")
	if(WIN32)
		set(_loader_var "PATH")
		set(_loader_sep ";")
	elseif(APPLE)
		set(_loader_var "DYLD_LIBRARY_PATH")
		set(_loader_sep ":")
	else()
		set(_loader_var "LD_LIBRARY_PATH")
		set(_loader_sep ":")
	endif()

	set(_loader_path "${_libdir}")
	if(DEFINED ENV{${_loader_var}} AND NOT "$ENV{${_loader_var}}" STREQUAL "")
		set(_loader_path "${_loader_path}${_loader_sep}$ENV{${_loader_var}}")
	endif()

	if(WIN32)
		# Escape semicolons for Windows environment variables
		string(REPLACE ";" "\\;" _pythonpath "${_pythonpath}")
		string(REPLACE ";" "\\;" _loader_path "${_loader_path}")
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
				ENVIRONMENT "PYTHONPATH=${_pythonpath};${_loader_var}=${_loader_path}"
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
endif()

