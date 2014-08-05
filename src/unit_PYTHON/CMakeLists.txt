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
	SET (CH_PYTHONINC "${PYTHON_INCLUDE_DIR}")
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
		
	INSTALL(TARGETS ChronoEngine_PYPARSER
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)	
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
					OUTPUT_NAME   "_ChronoEngine_PYTHON_core"
					RUNTIME_OUTPUT_DIRECTORY "${CH_PYTHONDIR_PYD}"
					LIBRARY_OUTPUT_DIRECTORY "${CH_PYTHONDIR_PYD}"	
					)
					
	ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} ChronoEngine)
						
						
	# To complete the build process, the .pyd and necessary .dll libraries 
	# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
			#  ... get the name of the most recent Chrono::Engine library to copy:
	GET_TARGET_PROPERTY(CH_CHRONO_LIBNAME ChronoEngine LOCATION)
			#  ... get the name of the .pyd library to copy:
	GET_TARGET_PROPERTY(CH_CHRONOPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} LOCATION)
			#  ...then copy it to the DLLs Python folder with .pyd and .dll files
	ADD_CUSTOM_COMMAND(
				TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME}
				POST_BUILD
				COMMAND ${CMAKE_COMMAND} -E copy 
					${CH_CHRONO_LIBNAME} 
					${CH_PYTHONDIR_PYD}/.
				COMMAND ${CMAKE_COMMAND} -E copy 
					${CH_CHRONOPYD_LIBNAME} 
					${CH_PYTHONDIR_PYD}/.
				)
				
					
	#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
	SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_core_REAL_NAME} PROPERTIES
							FOLDER "ChronoEngine_PYTHON")
		
	
	
		
		
		
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
						OUTPUT_NAME   "_ChronoEngine_PYTHON_postprocess"
						LIBRARY_OUTPUT_DIRECTORY "."	
						)
		
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} ChronoEngine)  
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} ChronoEngine_POSTPROCESS)  
		
		
		# To complete the build process, the .pyd and necessary .dll libraries 
		# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
				#  ... get the name of the most recent Chrono::Engine postprocess library to copy:
		GET_TARGET_PROPERTY(CH_CHRONOPOSTPROCESS_LIBNAME ChronoEngine_POSTPROCESS LOCATION)
				#  ... get the name of the .pyd library to copy:
		GET_TARGET_PROPERTY(CH_CHRONOPOSTPROCESSPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} LOCATION)
				#  ... then copy them to the DLLs Python folder for .pyd and .dll files
		ADD_CUSTOM_COMMAND(
					TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME}
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy 
						${CH_CHRONOPOSTPROCESS_LIBNAME} 
						${CH_PYTHONDIR_PYD}/.
					COMMAND ${CMAKE_COMMAND} -E copy 
						${CH_CHRONOPOSTPROCESSPYD_LIBNAME} 
						${CH_PYTHONDIR_PYD}/.
					)
					
						
		#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_postprocess_REAL_NAME} PROPERTIES
								FOLDER "ChronoEngine_PYTHON")
						
								
	ENDIF(ENABLE_UNIT_POSTPROCESS)	
	
	
	IF(ENABLE_UNIT_FEM)
		
		#-----------------------------------------------------------------------------
		#
		# MODULE for the ChronoEngine_PYTHON_fem python wrapper.
		#
		# This will build a .pyd python module to be imported via Python language,
		# not a .dll to be used with c++!!
		#
		# NOTE: to add a new source to this unit, just add its name
		# here and re-run the CMake.
		#
		
		SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG_fem 
			ChModuleFem.i
			)
					
		INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
		
		SET(CMAKE_SWIG_FLAGS "-c++")
		IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
			SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
			SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_fem} PROPERTIES COMPILE_FLAGS "-DWIN32")
		ENDIF()
		IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
			SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
		ENDIF()
		 
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_fem} PROPERTIES CPLUSPLUS ON)

		
		# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS_fem} PROPERTIES HEADER_FILE_ONLY TRUE)
	
		SWIG_ADD_MODULE(ChronoEngine_PYTHON_fem python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG_fem}) 
		SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_fem 
							${PYTHON_LIBRARY}
							ChronoEngine
							ChronoEngine_FEM
							)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME} PROPERTIES 
						PROJECT_LABEL "ChronoEngine_PYTHON_fem"
						OUTPUT_NAME   "_ChronoEngine_PYTHON_fem"
						LIBRARY_OUTPUT_DIRECTORY "."	
						)
						
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME} ChronoEngine)  
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME} ChronoEngine_FEM) 
							
							
		# To complete the build process, the .pyd and necessary .dll libraries 
		# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
				#  ... get the name of the most recent Chrono::Engine fem library to copy:
		GET_TARGET_PROPERTY(CH_CHRONOFEM_LIBNAME ChronoEngine_FEM LOCATION)
				#  ... get the name of the .pyd library to copy:
		GET_TARGET_PROPERTY(CH_CHRONOFEMPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME} LOCATION)
				#  ... then copy them to the DLLs Python folder for .pyd and .dll files
		ADD_CUSTOM_COMMAND(
					TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME}
					POST_BUILD
					COMMAND ${CMAKE_COMMAND} -E copy 
						${CH_CHRONOFEM_LIBNAME} 
						${CH_PYTHONDIR_PYD}/.
					COMMAND ${CMAKE_COMMAND} -E copy 
						${CH_CHRONOFEMPYD_LIBNAME} 
						${CH_PYTHONDIR_PYD}/.
					)
					
						
		#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_fem_REAL_NAME} PROPERTIES
								FOLDER "ChronoEngine_PYTHON")
 						
								
	ENDIF(ENABLE_UNIT_FEM)	
	
	
	
	
	IF(ENABLE_UNIT_IRRLICHT)
		
		#-----------------------------------------------------------------------------
		#
		# MODULE for the ChronoEngine_PYTHON_irrlicht python wrapper.
		#
		# This will build a .pyd python module to be imported via Python language,
		# not a .dll to be used with c++!!
		#
		# NOTE: to add a new source to this unit, just add its name
		# here and re-run the CMake.
		#
		
		SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG_irrlicht 
			ChModuleIrrlicht.i
			)
					
		INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
		
		SET(CMAKE_SWIG_FLAGS "-c++")
		IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
			SET(CMAKE_SWIG_OUTDIR "${CH_PYTHONDIR_PY}")
			SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_irrlicht} PROPERTIES COMPILE_FLAGS "-DWIN32")
		ENDIF()
		IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
			SET(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/python")
		ENDIF()
		 
		SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_CONFIG_irrlicht} PROPERTIES CPLUSPLUS ON)

		
		# SET_SOURCE_FILES_PROPERTIES(${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS_irrlicht} PROPERTIES HEADER_FILE_ONLY TRUE)
		
		SWIG_ADD_MODULE(ChronoEngine_PYTHON_irrlicht python ${ChronoEngine_unit_SWIG_PYTHON_CONFIG_irrlicht}) 
		SWIG_LINK_LIBRARIES(ChronoEngine_PYTHON_irrlicht 
							${PYTHON_LIBRARY}
							ChronoEngine
							ChronoEngine_IRRLICHT
							${CH_IRRLICHTLIB}  
							)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME} PROPERTIES 
						PROJECT_LABEL "ChronoEngine_PYTHON_irrlicht"
						OUTPUT_NAME   "_ChronoEngine_PYTHON_irrlicht"
						LIBRARY_OUTPUT_DIRECTORY "."	
						)
		
		ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME} ChronoEngine)  
		# ADD_DEPENDENCIES (${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME} ChronoEngine_IRRLICHT)  *** not needed since C::E Irrlicht lib is header-only
		
		
		# To complete the build process, the .pyd and necessary .dll libraries 
		# must be copied into the DLLs directory (it will be needed by the .pyd file), so..
				#  ... get the name of the most recent Chrono::Engine irrlicht library to copy:
		# GET_TARGET_PROPERTY(CH_CHRONOIRRLICHT_LIBNAME ChronoEngine_IRRLICHT LOCATION) *** not needed since C::E Irrlicht lib is header-only
				#  ... get the name of the .pyd library to copy:
		GET_TARGET_PROPERTY(CH_CHRONOIRRLICHTPYD_LIBNAME ${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME} LOCATION)
				#  ... then copy them to the DLLs Python folder for .pyd and .dll files
		ADD_CUSTOM_COMMAND(
					TARGET ${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME}
					POST_BUILD
					#COMMAND ${CMAKE_COMMAND} -E copy 
					#	${CH_CHRONOIRRLICHT_LIBNAME} 
					#	${CH_PYTHONDIR_PYD}/.           **** REMEMBER TO COPY THE IRRLICH.DLL INTO THE Python32/DLLs/ directory!!! ****
					COMMAND ${CMAKE_COMMAND} -E copy 
						${CH_CHRONOIRRLICHTPYD_LIBNAME} 
						${CH_PYTHONDIR_PYD}/.
					)
					
						
		#SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
		SET_TARGET_PROPERTIES(${SWIG_MODULE_ChronoEngine_PYTHON_irrlicht_REAL_NAME} PROPERTIES
								FOLDER "ChronoEngine_PYTHON")

								
	ENDIF(ENABLE_UNIT_IRRLICHT)		
	
	
	
					
	INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/python/ DESTINATION python)	
	
	
							
	
ENDIF(ENABLE_UNIT_PYTHON)
