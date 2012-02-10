#=============================================================================
# CHRONO::ENGINE   CMake configuration file for PYTHON unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_PYTHON     FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine PYTHON unit.")
SET(CH_PYTHONDIR           ""       CACHE PATH   "Where is your Python installed? Ex: 'C:/Python27'. This path should contain the Lib, libs, etc. directories.")


# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_PYTHON)
	MARK_AS_ADVANCED(CLEAR CH_PYTHONDIR)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_PYTHONDIR)
ENDIF()


IF(ENABLE_UNIT_PYTHON)

	FIND_PACKAGE(SWIG REQUIRED)
	INCLUDE(${SWIG_USE_FILE})
	
	FIND_PACKAGE(PythonLibs REQUIRED)
	
	SET(CH_PYTHONDIR_PYD "${CH_PYTHONDIR}/DLLs")
	SET(CH_PYTHONDIR_PY  "${CH_PYTHONDIR}/Lib")
	
	# the generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
	SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
	
	#-----------------------------------------------------------------------------
	# 
	# MODULE for the ChronoEngine_POSTPROCESS wrapper, including most of the C::E
	#
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG 
		ChModuleCore.i
		)
	SET(ChronoEngine_unit_SWIG_PYTHON_WRAPPERS
		ChException.i
		ChCoordsys.i
		ChVector.i
		ChQuaternion.i
		ChHashFunction.i
		)

		
	SET(ChronoEngine_unit_PYTHON_SOURCES
		ChPython.cpp
		)
	SET(ChronoEngine_unit_PYTHON_HEADERS
		ChApiPYTHON.h
		ChPython.h
		)
				
	
	INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
	
	
	SET(CMAKE_SWIG_FLAGS "-c++")
	SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
	 
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG} PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG} PROPERTIES COMPILE_FLAGS "-DWIN32")
	
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS} PROPERTIES HEADER_FILE_ONLY TRUE)
	SOURCE_GROUP(wrappers FILES  ${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS})
	
	SWIG_ADD_MODULE(ChronoEngine_PYTHON_core python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG}) 
	SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_core 
						${PYTHON_LIBRARY}
						optimized 	${CH_CHRONOLIB}
						debug 		${CH_CHRONOLIB_DBG}
						)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} PROPERTIES 
					PROJECT_LABEL "ChronoEngine_PYTHON_core"
					LIBRARY_OUTPUT_DIRECTORY "."
					ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
					RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_PYTHONDIR_PYD}"
					RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_PYTHONDIR_PYD}"
					LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_PYTHONDIR_PYD}"
					LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_PYTHONDIR_PYD}"
					RUNTIME_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_core_DBG"
					LIBRARY_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_core_DBG"
					ARCHIVE_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_core_DBG")
						
	
	
	ADD_CUSTOM_COMMAND(
				TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME}
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy
					${CH_CHRONOSHAREDLNAME}
					"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}${CH_LIBNAMEPOSTFIXSHARED}"
				COMMAND ${CMAKE_COMMAND} -E copy
					${CH_CHRONOSHAREDLNAME_DBG}
					"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}DBG${CH_LIBNAMEPOSTFIXSHARED}"
					)
					
	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} PROPERTIES
							FOLDER "ChronoEngine_PYTHON")
		
	
		
		
		
		
	IF(ENABLE_UNIT_POSTPROCESS)
		
		#-----------------------------------------------------------------------------
		#
		# MODULE for the ChronoEngine_POSTPROCESS python wrapper
		#
		# NOTE: to add a new source to this unit, just add its name
		# here and re-run the CMake.
		#
		
		SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess 
			ChModulePostprocess.i
			)
					
		INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
		
		SET(CMAKE_SWIG_FLAGS "-c++")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
		 
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess} PROPERTIES CPLUSPLUS ON)
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess} PROPERTIES COMPILE_FLAGS "-DWIN32")
		
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS_postprocess} PROPERTIES HEADER_FILE_ONLY TRUE)
		
		SWIG_ADD_MODULE(ChronoEngine_PYTHON_postprocess python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess}) 
		SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_postprocess 
							${PYTHON_LIBRARY}
							optimized 	${CH_CHRONOLIB}
							debug 		${CH_CHRONOLIB_DBG}
							optimized 	${CH_CHRONOPOSTPROCESSLIB} 
 							debug 		${CH_CHRONOPOSTPROCESSLIB_DBG}
							)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} PROPERTIES 
						PROJECT_LABEL "ChronoEngine_PYTHON_postprocess"
						LIBRARY_OUTPUT_DIRECTORY "."
						ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
						ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
						RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_PYTHONDIR_PYD}"
						RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_PYTHONDIR_PYD}"
						LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_PYTHONDIR_PYD}"
						LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_PYTHONDIR_PYD}"
						RUNTIME_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_postprocess_DBG"
						LIBRARY_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_postprocess_DBG"
						ARCHIVE_OUTPUT_NAME_DEBUG "_ChronoEngine_PYTHON_postprocess_DBG")
								
		ADD_CUSTOM_COMMAND(
					TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME}
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy
						${CH_CHRONOPOSTPROCESSSHAREDLNAME}
						"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}_POSTPROCESS${CH_LIBNAMEPOSTFIXSHARED}"
					COMMAND ${CMAKE_COMMAND} -E copy
						${CH_CHRONOPOSTPROCESSSHAREDLNAME_DBG}
						"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}_POSTPROCESSDBG${CH_LIBNAMEPOSTFIXSHARED}"
						)
						
		#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} PROPERTIES
								FOLDER "ChronoEngine_PYTHON")
								
								
	ENDIF(ENABLE_UNIT_POSTPROCESS)						
								
		
ENDIF(ENABLE_UNIT_PYTHON)
