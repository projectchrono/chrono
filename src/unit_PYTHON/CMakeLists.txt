#=============================================================================
# CHRONO::ENGINE   CMake configuration file for PYTHON unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_PYTHON     FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine PYTHON unit(s).")


IF(ENABLE_UNIT_PYTHON)


	FIND_PACKAGE(PythonLibs REQUIRED)
	FIND_PACKAGE(PythonInterp REQUIRED)

	GET_FILENAME_COMPONENT(CH_PYTHONDIR "${PYTHON_EXECUTABLE}" PATH)
	SET (CH_PYTHONINC "${PYTHON_INCLUDE_PATH}")
	SET (CH_PYTHONLIB "${PYTHON_LIBRARIES}")
	
	
	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE PYTHON PARSER LIBRARY to be used via C++
	#
	#
	
	SET(ChronoEngine_UNIT_PYPARSER_SOURCES 
		ChPython.cpp 
		)
	SET(ChronoEngine_UNIT_PYPARSER_HEADERS
		ChApiPyParser.h 
		ChPython.h
		ChSwigutils.h 
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_UNIT_PYPARSER_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_PYPARSER FILES 
				${ChronoEngine_UNIT_PYPARSER_SOURCES} 
				${ChronoEngine_UNIT_PYPARSER_HEADERS})
				
					
	# In most cases, you do not need to edit the lines below

	
	SET (CH_CHRONOPYPARSERLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYPARSER${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOPYPARSERSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_PYPARSER${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOPYPARSERLIB  "${CH_CHRONOPYPARSERLIBRARYNAME}")
	SET (CH_CHRONOPYPARSERLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYPARSERDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOPYPARSERSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_PYPARSERDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOPYPARSERLIB_DBG  "${CH_CHRONOPYPARSERLIBRARYNAME_DBG}")
	

	INCLUDE_DIRECTORIES( ${CH_PYTHONINC} )
	
	
	# The PYPARSER library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_PYPARSER SHARED 
				${ChronoEngine_UNIT_PYPARSER_SOURCES}
				${ChronoEngine_UNIT_PYPARSER_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_PYPARSER PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_PYPARSER")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_PYPARSER 
		${CH_PYTHONLIB}
		ChronoEngine)
		
	ADD_DEPENDENCIES (ChronoEngine_PYPARSER ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_PYTHONINC      		"${CH_PYTHONINC}" 			 PARENT_SCOPE )
	SET (CH_CHRONOPYPARSERLIB  		"${CH_CHRONOPYPARSERLIB}" 	 PARENT_SCOPE )
	SET (CH_CHRONOPYPARSERLIB_DBG  	"${CH_CHRONOPYPARSERLIB_DBG}" PARENT_SCOPE )
	SET (CH_CHRONOPYPARSERSHAREDLNAME  "${CH_CHRONOPYPARSERSHAREDLNAME}" PARENT_SCOPE )
	SET (CH_CHRONOPYPARSERSHAREDLNAME_DBG  "${CH_CHRONOPYPARSERSHAREDLNAME_DBG}" PARENT_SCOPE )


	#----------------------------------------------------------------------------
	# 
	# From this point, define the projects for building the Python modules.
	



	FIND_PACKAGE(SWIG REQUIRED)
	INCLUDE(${SWIG_USE_FILE})
	
	SET(CH_PYTHONDIR_PYD "${CH_PYTHONDIR}/DLLs")
	SET(CH_PYTHONDIR_PY  "${CH_PYTHONDIR}/Lib")
	
	# the generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
	ENDIF()
	#-----------------------------------------------------------------------------
	# 
	# MODULE for the ChronoEngine_PYTHON_core wrapper, including most of the C::E
	#
	# This will build a .pyd python module to be imported via Python language,
	# not a .dll to be used with c++!!
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

	IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG} PROPERTIES COMPILE_FLAGS "-DWIN32")
	ENDIF()
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
	 ENDIF()
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG} PROPERTIES CPLUSPLUS ON)

	
	SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS} PROPERTIES HEADER_FILE_ONLY TRUE)
	SOURCE_GROUP(wrappers FILES  ${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS})
	
	SWIG_ADD_MODULE(ChronoEngine_PYTHON_core python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG}) 
	SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_core 
						${PYTHON_LIBRARY}
						ChronoEngine
						)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} PROPERTIES 
					PROJECT_LABEL "ChronoEngine_PYTHON_core"
					LIBRARY_OUTPUT_DIRECTORY ".")
						
	
	
	#ADD_CUSTOM_COMMAND(
	#			TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME}
	#			POST_BUILD
	#			COMMAND ${CMAKE_COMMAND} -E copy
	#				${CH_CHRONOSHAREDLNAME}
	#				"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}${CH_LIBNAMEPOSTFIXSHARED}"
	#			COMMAND ${CMAKE_COMMAND} -E copy
	#				${CH_CHRONOSHAREDLNAME_DBG}
	#				"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}DBG${CH_LIBNAMEPOSTFIXSHARED}"
	#				)
					
	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} PROPERTIES
							FOLDER "ChronoEngine_PYTHON")
		
	
	ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} ChronoEngine)	
		
		
		
	IF(ENABLE_UNIT_POSTPROCESS)
		
		#-----------------------------------------------------------------------------
		#
		# MODULE for the ChronoEngine_PYTHON_postprocess python wrapper.
		#
		# This will build a .pyd python module to be imported via Python language,
		# not a .dll to be used with c++!!
		#
		# NOTE: to add a new source to this unit, just add its name
		# here and re-run the CMake.
		#
		
		SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess 
			ChModulePostprocess.i
			)
					
		INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
		
		SET(CMAKE_SWIG_FLAGS "-c++")
		IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
		SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess} PROPERTIES COMPILE_FLAGS "-DWIN32")
	ENDIF()
	IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
		SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
	 ENDIF()
		 
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess} PROPERTIES CPLUSPLUS ON)

		
		# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS_postprocess} PROPERTIES HEADER_FILE_ONLY TRUE)
		
		SWIG_ADD_MODULE(ChronoEngine_PYTHON_postprocess python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG_postprocess}) 
		SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_postprocess 
							${PYTHON_LIBRARY}
							ChronoEngine
							ChronoEngine_POSTPROCESS
							)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} PROPERTIES 
						PROJECT_LABEL "ChronoEngine_PYTHON_postprocess"
						LIBRARY_OUTPUT_DIRECTORY ".")
								
		#ADD_CUSTOM_COMMAND(
		#			TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME}
		#			POST_BUILD
		#			COMMAND ${CMAKE_COMMAND} -E copy
		#				${CH_CHRONOPOSTPROCESSSHAREDLNAME}
		#				"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}_POSTPROCESS${CH_LIBNAMEPOSTFIXSHARED}"
		#			COMMAND ${CMAKE_COMMAND} -E copy
		#				${CH_CHRONOPOSTPROCESSSHAREDLNAME_DBG}
		#				"${CH_PYTHONDIR_PYD}/${CH_LIBNAME}_POSTPROCESSDBG${CH_LIBNAMEPOSTFIXSHARED}"
		#				)
						
		#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} PROPERTIES
								FOLDER "ChronoEngine_PYTHON")


		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} ChronoEngine)  
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} ChronoEngine_POSTPROCESS)  						
								
	ENDIF(ENABLE_UNIT_POSTPROCESS)						
								
	
ENDIF(ENABLE_UNIT_PYTHON)
