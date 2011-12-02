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
	
	
	
	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE PYTHON LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_SWIG_PYTHON_CONFIG 
		ChModuleCore.i
		)
	SET(ChronoEngine_unit_SWIG_PYTHON_WRAPPERS
		ChModuleCore.i
		ChException.i
		ChCoordsys.i
		ChVector.i
		ChQuaternion.i
		ChHashFunction.i
		)
	SET(ChronoEngine_unit_SWIG_PYTHON_SOURCES
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
						
	SOURCE_GROUP(Wrappers FILES  ${ChronoEngine_unit_SWIG_PYTHON_WRAPPERS})
	
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
		
	

	#SOURCE_GROUP(unit_PYTHON FILES 
	#			${ChronoEngine_unit_SWIG_PYTHON_CONFIG}
	#			${ChronoEngine_unit_PYTHON_HEADERS}
	#			${ChronoEngine_unit_PYTHON_SOURCES})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	

	#SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/ros/win32/vc8/lib/")
	#SET (CH_CASCADEINC "${CH_CASCADEDIR}/ros/inc")
	#SET (CH_CASCADELIBS 
 	#		${CH_CASCADELIBDIR}FWOSPlugin.lib
 	#		${CH_CASCADELIBDIR}PTKernel.lib
 	#)
	
	
	# SET (CH_CHRONOPYTHONLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYTHON${CH_LIBNAMEPOSTFIX}")
	# SET (CH_CHRONOPYTHONSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_PYTHON${CH_LIBNAMEPOSTFIXSHARED}")
	# SET (CH_CHRONOPYTHONLIB  "${CH_CHRONOPYTHONLIBRARYNAME}")
	# SET (CH_CHRONOPYTHONLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_PYTHONDBG${CH_LIBNAMEPOSTFIX}")
	# SET (CH_CHRONOPYTHONSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_PYTHONDBG${CH_LIBNAMEPOSTFIXSHARED}")
	# SET (CH_CHRONOPYTHONLIB_DBG  "${CH_CHRONOPYTHONLIBRARYNAME_DBG}")
	
	
	# The library is added to the project,
	# and some custom properties of this target are set.
	
	
	# ADD_LIBRARY(ChronoEngine_PYTHON SHARED 
	#			${ChronoEngine_unit_PYTHON_SOURCES}
	#			${ChronoEngine_unit_PYTHON_HEADERS})
	
	# SET_TARGET_PROPERTIES(ChronoEngine_PYTHON PROPERTIES 
	#                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	#                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_PYTHON"
	#                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	#                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	#                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	 #                         RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	 #                         LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	 #                         LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	  #                        RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_PYTHONDBG"
	 #                         LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngine_PYTHONDBG"
	 #                         ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_PYTHONDBG")
	                          
	# TARGET_LINK_LIBRARIES(ChronoEngine_PYTHON
	#	${CH_CASCADELIBS} 
	#	optimized 	${CH_CHRONOLIB}
	#	debug 		${CH_CHRONOLIB_DBG}  )
		
	# ADD_DEPENDENCIES (ChronoEngine_PYTHON ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	# SET (CH_PYTHONINC      			"${CH_PYTHONINC}" 			 PARENT_SCOPE )
	# SET (CH_PYTHONLIBS      		"${CH_PYTHONLIBS}" 		 	 PARENT_SCOPE )
	# SET (CH_CHRONOPYTHONLIB  		"${CH_CHRONOPYTHONLIB}" 	 PARENT_SCOPE )
	# SET (CH_CHRONOPYTHONLIB_DBG  	"${CH_CHRONOPYTHONLIB_DBG}"  PARENT_SCOPE )
		
		
ENDIF(ENABLE_UNIT_PYTHON)
