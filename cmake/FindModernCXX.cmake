#.rst:
# FindModernCXX
# -------------
#
# This module determines the versions of C++ supported by the compiler.
#
# The following variables are set
#
# ::
#
#   CMAKE_CXX11_STANDARD_COMPILE_OPTION    - compiler option for C++11
#   CMAKE_CXX11_EXTENSION_COMPILE_OPTION   - compiler option for C++11 w/extensions
#   CMAKE_CXX14_STANDARD_COMPILE_OPTION    - compiler option for C++14
#   CMAKE_CXX14_EXTENSION_COMPILE_OPTION   - compiler option for C++14 w/extensions
#   CMAKE_CXX17_STANDARD_COMPILE_OPTION    - compiler option for C++17
#   CMAKE_CXX17_EXTENSION_COMPILE_OPTION   - compiler option for C++17 w/extensions
#
# Note: These variables are set by default in CMake >= 3.1.0.
#
# TODO: Generalize to support C11, make more DRY.

include(CheckCXXCompilerFlag)

macro(_check_compiler_option COMPILEROPTION SHORTNAME)
	CHECK_CXX_COMPILER_FLAG("${COMPILEROPTION}" COMPILER_SUPPORTS_${SHORTNAME})
endmacro()

macro(_find_cxx_flags CXXVERSION CXXPREVERSION)
	_check_compiler_option("-std=c++${CXXVERSION}" "CXX${CXXVERSION}")
	_check_compiler_option("-std=c++${CXXPREVERSION}" "CXX${CXXPREVERSION}")
	_check_compiler_option("-std=gnu++${CXXVERSION}" "GNUXX${CXXVERSION}")
	_check_compiler_option("-std=gnu++${CXXPREVERSION}" "GNUXX${CXXPREVERSION}")

	# Hack: these should only be set to empty if the compiler does not require a
	# specific flag.
	set(CMAKE_CXX${CXXVERSION}_STANDARD_COMPILE_OPTION "")
	set(CMAKE_CXX${CXXVERSION}_EXTENSION_COMPILE_OPTION "")

	if(COMPILER_SUPPORTS_CXX${CXXVERSION})
		set(CMAKE_CXX${CXXVERSION}_STANDARD_COMPILE_OPTION "-std=c++${CXXVERSION}")
	elseif(COMPILER_SUPPORTS_CXX${CXXPREVERSION})
		set(CMAKE_CXX${CXXVERSION}_STANDARD_COMPILE_OPTION "-std=c++${CXXPREVERSION}")
	endif()

	if(COMPILER_SUPPORTS_GNUXX${CXXVERSION})
		set(CMAKE_CXX${CXXVERSION}_EXTENSION_COMPILE_OPTION "-std=gnu++${CXXVERSION}")
	elseif(COMPILER_SUPPORTS_GNUXX${CXXPREVERSION})
		set(CMAKE_CXX${CXXVERSION}_EXTENSION_COMPILE_OPTION "-std=gnu++${CXXPREVERSION}")
	endif()
endmacro()


macro(_check_or_find_cxx_flags CXXVERSION CXXPREVERSION)
	if(NOT CMAKE_CXX${CXXVERSION}_STANDARD_COMPILE_OPTION)
		_find_cxx_flags(${CXXVERSION} ${CXXPREVERSION})
	endif()

	if(NOT CMAKE_CXX${CXXVERSION}_EXTENSION_COMPILE_OPTION)
		_find_cxx_flags(${CXXVERSION} ${CXXPREVERSION})
	endif()
endmacro()

_check_or_find_cxx_flags(11 0x)
_check_or_find_cxx_flags(14 1y)
_check_or_find_cxx_flags(17 1z)
