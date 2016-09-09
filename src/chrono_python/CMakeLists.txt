#=============================================================================
# CMake configuration file for the Chrono Python module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PYTHON "Enable the Chrono Python module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_PYTHON)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono Python module ====")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

#-----------------------------------------------------------------------------

set(Python_ADDITIONAL_VERSIONS 3.4)
FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT) 

GET_FILENAME_COMPONENT(CH_PYTHONDIR "${PYTHON_EXECUTABLE}" PATH)
SET (CH_PYTHONINC "${PYTHON_INCLUDE_DIR}")
SET (CH_PYTHONLIB "${PYTHON_LIBRARIES}")

# this to fix warnings about the use of LOCATION with CMake version >= 3.0
if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif()

#-----------------------------------------------------------------------------
# List the files in the Python parser library to e used via C++

SET(ChronoEngine_PYPARSER_SOURCES 
	ChPython.cpp 
	)

SET(ChronoEngine_PYPARSER_HEADERS
	ChApiPyParser.h 
	ChPython.h
	ChSwigutils.h 
	)

SOURCE_GROUP("" FILES 
			${ChronoEngine_PYPARSER_SOURCES} 
			${ChronoEngine_PYPARSER_HEADERS})
			
				
#-------------------------------------------------------------------------------

SET (CH_CHRONOPYPARSERLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYPARSER${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOPYPARSERSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_PYPARSER${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOPYPARSERLIB  "${CH_CHRONOPYPARSERLIBRARYNAME}")
SET (CH_CHRONOPYPARSERLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYPARSERDBG${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOPYPARSERSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_PYPARSERDBG${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOPYPARSERLIB_DBG  "${CH_CHRONOPYPARSERLIBRARYNAME_DBG}")


INCLUDE_DIRECTORIES( ${CH_PYTHONINC} )


#-------------------------------------------------------------------------------
# Create the ChronoEngine_pyparser library

ADD_LIBRARY(ChronoEngine_pyparser SHARED 
			${ChronoEngine_PYPARSER_SOURCES}
			${ChronoEngine_PYPARSER_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_pyparser PROPERTIES 
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                      COMPILE_DEFINITIONS "CH_API_COMPILE_PYPARSER")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_pyparser 
	                  ${CH_PYTHONLIB}
	                  ChronoEngine)
	
ADD_DEPENDENCIES(ChronoEngine_pyparser ChronoEngine)

INSTALL(TARGETS ChronoEngine_pyparser
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

SET(CH_PYTHONINC  "${CH_PYTHONINC}"  PARENT_SCOPE )
SET(CH_CHRONOPYPARSERLIB  "${CH_CHRONOPYPARSERLIB}"  PARENT_SCOPE )
SET(CH_CHRONOPYPARSERLIB_DBG  "${CH_CHRONOPYPARSERLIB_DBG}"  PARENT_SCOPE )
SET(CH_CHRONOPYPARSERSHAREDLNAME  "${CH_CHRONOPYPARSERSHAREDLNAME}"  PARENT_SCOPE )
SET(CH_CHRONOPYPARSERSHAREDLNAME_DBG  "${CH_CHRONOPYPARSERSHAREDLNAME_DBG}"  PARENT_SCOPE )

#----------------------------------------------------------------------------
# From this point, define the projects for building the Python modules.

FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

SET(CH_PYTHONDIR_PYD "${CH_PYTHONDIR}/DLLs")
SET(CH_PYTHONDIR_PY  "${CH_PYTHONDIR}/Lib")

# The generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
ENDIF()

#-----------------------------------------------------------------------------
# MODULE for the ChronoEngine_python_core wrapper, including most of the C::E
#
# This will build a .pyd python module to be imported via Python language,
# not a .dll to be used with c++!!
#	
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#

SET(ChronoEngine_SWIG_PYTHON_CONFIG 
	ChModuleCore.i
	)

SET(ChronoEngine_SWIG_PYTHON_WRAPPERS
	ChException.i
	ChCoordsys.i
	ChVector.i
	ChQuaternion.i
	)

			

INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})


SET(CMAKE_SWIG_FLAGS "-c++")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG}
	  PROPERTIES COMPILE_FLAGS "-D_WIN32")
ENDIF()
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
ENDIF()
SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG} PROPERTIES CPLUSPLUS ON)


SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_WRAPPERS} PROPERTIES HEADER_FILE_ONLY TRUE)
SOURCE_GROUP(wrappers FILES  ${ChronoEngine_SWIG_PYTHON_WRAPPERS})

SWIG_ADD_MODULE(ChronoEngine_python_core python ${ChronoEngine_SWIG_PYTHON_CONFIG}) 
SWIG_LINK_LIBRARIES(ChronoEngine_python_core 
					${PYTHON_LIBRARY}
					ChronoEngine)
SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_core_REAL_NAME} PROPERTIES 
				PROJECT_LABEL "ChronoEngine_python_core"
				OUTPUT_NAME   "_ChronoEngine_python_core"
				RUNTIME_OUTPUT_DIRECTORY "${CH_PYTHONDIR_PYD}"
				LIBRARY_OUTPUT_DIRECTORY "${CH_PYTHONDIR_PYD}")
				
ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_python_core_REAL_NAME} ChronoEngine)

					
# To complete the build process, the .pyd and necessary .dll libraries 
# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
		#  ... get the name of the most recent Chrono::Engine library to copy:
GET_TARGET_PROPERTY(CH_CHRONO_LIBNAME ChronoEngine LOCATION)
		#  ... get the name of the .pyd library to copy:
GET_TARGET_PROPERTY(CH_CHRONOPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_python_core_REAL_NAME} LOCATION)
		#  ...then copy it to the DLLs Python folder with .pyd and .dll files
ADD_CUSTOM_COMMAND(
			TARGET ${SWIG_MODULE_ChronoEngine_python_core_REAL_NAME}
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy 
				${CH_CHRONO_LIBNAME} 
				${CH_PYTHONDIR_PYD}/.
			COMMAND ${CMAKE_COMMAND} -E copy 
				${CH_CHRONOPYD_LIBNAME} 
				${CH_PYTHONDIR_PYD}/.
			)
			
				
#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_core_REAL_NAME} PROPERTIES
						FOLDER "ChronoEngine_python")
	
	
	
IF(ENABLE_MODULE_POSTPROCESS)
	
	#-----------------------------------------------------------------------------
	#
	# MODULE for the ChronoEngine_python_postprocess python wrapper.
	#
	# This will build a .pyd python module to be imported via Python language,
	# not a .dll to be used with c++!!
	#
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_SWIG_PYTHON_CONFIG_postprocess 
		ChModulePostprocess.i
		)
				
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
	
	SET(CMAKE_SWIG_FLAGS "-c++")
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_postprocess}
		  PROPERTIES COMPILE_FLAGS "-D_WIN32")
	ENDIF()
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
	ENDIF()
	 
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_postprocess} PROPERTIES CPLUSPLUS ON)

	
	# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_WRAPPERS_postprocess} PROPERTIES HEADER_FILE_ONLY TRUE)

	SWIG_ADD_MODULE(ChronoEngine_python_postprocess python ${ChronoEngine_SWIG_PYTHON_CONFIG_postprocess}) 
	SWIG_LINK_LIBRARIES(ChronoEngine_python_postprocess 
						${PYTHON_LIBRARY}
						ChronoEngine
						ChronoEngine_postprocess)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME} PROPERTIES 
					PROJECT_LABEL "ChronoEngine_python_postprocess"
					OUTPUT_NAME   "_ChronoEngine_python_postprocess"
					LIBRARY_OUTPUT_DIRECTORY ".")
	
	ADD_DEPENDENCIES(${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME} ChronoEngine)
	ADD_DEPENDENCIES(${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME} ChronoEngine_postprocess)
	
	# To complete the build process, the .pyd and necessary .dll libraries 
	# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
			#  ... get the name of the most recent Chrono::Engine postprocess library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOPOSTPROCESS_LIBNAME ChronoEngine_postprocess LOCATION)
			#  ... get the name of the .pyd library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOPOSTPROCESSPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME} LOCATION)
			#  ... then copy them to the DLLs Python folder for .pyd and .dll files
	ADD_CUSTOM_COMMAND(TARGET ${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME}
				       POST_BUILD
				       COMMAND ${CMAKE_COMMAND} -E copy 
					           ${CH_CHRONOPOSTPROCESS_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.
				       COMMAND ${CMAKE_COMMAND} -E copy 
					           ${CH_CHRONOPOSTPROCESSPYD_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.)
				
	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_postprocess_REAL_NAME} PROPERTIES
						  FOLDER "ChronoEngine_python")

ENDIF()	


IF(ENABLE_MODULE_FEA)
	
	#-----------------------------------------------------------------------------
	#
	# MODULE for the ChronoEngine_python_fea python wrapper.
	#
	# This will build a .pyd python module to be imported via Python language,
	# not a .dll to be used with c++!!
	#
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_SWIG_PYTHON_CONFIG_fea 
		ChModuleFea.i
		)
				
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
	
	SET(CMAKE_SWIG_FLAGS "-c++")
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_fea}
		  PROPERTIES COMPILE_FLAGS "-D_WIN32")
	ENDIF()
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
	ENDIF()
	 
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_fea} PROPERTIES CPLUSPLUS ON)

	
	# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_WRAPPERS_fea} PROPERTIES HEADER_FILE_ONLY TRUE)

	SWIG_ADD_MODULE(ChronoEngine_python_fea python ${ChronoEngine_SWIG_PYTHON_CONFIG_fea}) 
	SWIG_LINK_LIBRARIES(ChronoEngine_python_fea 
						${PYTHON_LIBRARY}
						ChronoEngine
						ChronoEngine_fea)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME} PROPERTIES 
					PROJECT_LABEL "ChronoEngine_python_fea"
					OUTPUT_NAME   "_ChronoEngine_python_fea"
					LIBRARY_OUTPUT_DIRECTORY ".")
					
	ADD_DEPENDENCIES(${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME} ChronoEngine)
	ADD_DEPENDENCIES(${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME} ChronoEngine_fea)

	# To complete the build process, the .pyd and necessary .dll libraries 
	# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
			#  ... get the name of the most recent Chrono::Engine fea library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOFEA_LIBNAME ChronoEngine_fea LOCATION)
			#  ... get the name of the .pyd library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOFEAPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME} LOCATION)
			#  ... then copy them to the DLLs Python folder for .pyd and .dll files
	ADD_CUSTOM_COMMAND(TARGET ${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME}
				       POST_BUILD
				       COMMAND ${CMAKE_COMMAND} -E copy 
					           ${CH_CHRONOFEA_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.
				       COMMAND ${CMAKE_COMMAND} -E copy 
					           ${CH_CHRONOFEAPYD_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.)

	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_fea_REAL_NAME} PROPERTIES
						  FOLDER "ChronoEngine_python")
		
ENDIF()	


IF(ENABLE_MODULE_IRRLICHT)
	
	#-----------------------------------------------------------------------------
	#
	# MODULE for the ChronoEngine_python_irrlicht python wrapper.
	#
	# This will build a .pyd python module to be imported via Python language,
	# not a .dll to be used with c++!!
	#
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_SWIG_PYTHON_CONFIG_irrlicht 
		ChModuleIrrlicht.i
		)
				
	INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
	
	SET(CMAKE_SWIG_FLAGS "-c++")
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_irrlicht}
		  PROPERTIES COMPILE_FLAGS "-D_WIN32")
	ENDIF()
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
	ENDIF()
	 
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_CONFIG_irrlicht} PROPERTIES CPLUSPLUS ON)

	
	# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_SWIG_PYTHON_WRAPPERS_irrlicht} PROPERTIES HEADER_FILE_ONLY TRUE)
	
	SWIG_ADD_MODULE(ChronoEngine_python_irrlicht python ${ChronoEngine_SWIG_PYTHON_CONFIG_irrlicht}) 
	SWIG_LINK_LIBRARIES(ChronoEngine_python_irrlicht 
						${PYTHON_LIBRARY}
						ChronoEngine
						ChronoEngine_irrlicht
						${CH_IRRLICHTLIB})
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME} PROPERTIES 
					      PROJECT_LABEL "ChronoEngine_python_irrlicht"
					      OUTPUT_NAME   "_ChronoEngine_python_irrlicht"
					      LIBRARY_OUTPUT_DIRECTORY ".")
	
	ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME} ChronoEngine)  
	# ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME} ChronoEngine_IRRLICHT)  *** not needed since C::E Irrlicht lib is header-only
	
	# To complete the build process, the .pyd and necessary .dll libraries 
	# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
			#  ... get the name of the most recent Chrono::Engine irrlicht library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOIRRLICHT_LIBNAME ChronoEngine_irrlicht LOCATION) 
			#  ... get the name of the .pyd library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOIRRLICHTPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME} LOCATION)
			#  ... then copy them to the DLLs Python folder for .pyd and .dll files
	ADD_CUSTOM_COMMAND(TARGET ${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME}
				       POST_BUILD
				       COMMAND ${CMAKE_COMMAND} -E copy 
				 	           ${CH_CHRONOIRRLICHT_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.
				       COMMAND ${CMAKE_COMMAND} -E copy 
					           ${CH_CHRONOIRRLICHTPYD_LIBNAME} 
					           ${CH_PYTHONDIR_PYD}/.)
			
	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_python_irrlicht_REAL_NAME} PROPERTIES
						  FOLDER "ChronoEngine_python")
							
ENDIF()		

			
INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/python/ DESTINATION python)	
