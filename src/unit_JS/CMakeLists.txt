#=============================================================================
# CHRONO::ENGINE   CMake configuration file for JS JavaScript unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_JS       TRUE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine Javascript unit.")
SET(CH_JAVASRC           ""      CACHE PATH   "Where is your SpiderMonkey Javascript SDK installed? You must set this path to compile the JS unit.")


# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_JS)
	MARK_AS_ADVANCED(CLEAR CH_JAVASRC)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_JAVASRC)
ENDIF()


IF(ENABLE_UNIT_JS)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE JS LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_JS_SOURCES 
		ChControlsJS.cpp
		ChFunctionJS.cpp
		ChFxJavascript.cpp
		ChGlobalJS.cpp
		ChJs_all.cpp
		ChJs_body.cpp
		ChJs_chobject.cpp
		ChJs_constraint.cpp
		ChJs_controller.cpp
		ChJs_Engine.cpp
		ChJs_force.cpp
		ChJs_funct.cpp
		ChJs_geometry.cpp
		ChJs_impact.cpp
		ChJs_link.cpp
		ChJs_marker.cpp
		ChJs_math.cpp
		ChJs_optimizer.cpp
		ChJs_system.cpp
		ChJs_utils.cpp
		ChOptvar.cpp
		)
	SET(ChronoEngine_unit_JS_HEADERS
		ChApiJS.h
		ChControlsJS.h
		ChFunctionJS.h
		ChFxJavascript.h
		ChGlobalJS.h
		ChJs_all.h
		ChJs_body.h
		ChJs_chobject.h
		ChJs_constraint.h
		ChJs_controller.h
		ChJs_Engine.h
		ChJs_force.h
		ChJs_funct.h
		ChJs_geometry.h
		ChJs_impact.h
		ChJs_link.h
		ChJs_marker.h
		ChJs_math.h
		ChJs_optimizer.h
		ChJs_system.h
		ChJs_utils.h
		ChOptvar.h
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_unit_JS_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_MPI FILES 
				${ChronoEngine_unit_JS_SOURCES} 
				${ChronoEngine_unit_JS_HEADERS})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	
	SET (CH_JAVASCRIPTFLAGS "-DCH_JAVASCRIPT")
	
	IF ("${CH_OS}" STREQUAL "WINDOWS")
		SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DXP_WIN")
		SET (CH_JAVASCRIPTINC   "${CH_JAVASRC}/src")
		
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
			SET (CH_JAVASCRIPTLIB "${CH_LIBDIR}/js32.lib")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
			SET (CH_JAVASCRIPTLIB "${CH_LIBDIR}/js64.lib")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC")
			SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DEXPORT_JS_API")
			SET (CH_JAVASCRIPTLIB "${CH_BINDIR}/js.dll")
		ENDIF()
		IF ("${CH_COMPILER}" STREQUAL "COMPILER_GCC_X64")
			SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DEXPORT_JS_API")
			SET (CH_JAVASCRIPTLIB "${CH_BINDIR}/js.dll")
		ENDIF()
	ENDIF()
	IF ("${CH_OS}" STREQUAL "LINUX")
		SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DXP_UNIX")
		SET (CH_JAVASCRIPTINC   "${CH_JAVASRC}")
		SET (CH_JAVASCRIPTLIB   "${CH_LIBDIR}/libjs.a")
		
		MESSAGE( "WARNING!, JS building under LINUX is not yet tested! Check libs paths etc!")
	ENDIF()

	
	SET (CH_CHRONOJSLIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_JS${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOJSSHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_MPI${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOJSLIB  "${CH_CHRONOJSLIBRARYNAME}")
	SET (CH_CHRONOJSLIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}JSDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOJSSHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}MPIDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOJSLIB_DBG  "${CH_CHRONOJSLIBRARYNAME_DBG}")
	
	
	INCLUDE_DIRECTORIES( ${CH_JAVASCRIPTINC} )
	
	# The MPI library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_JS SHARED 
				${ChronoEngine_unit_JS_SOURCES}
				${ChronoEngine_unit_JS_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_JS PROPERTIES 
	                          LINK_FLAGS "" 
	                          COMPILE_DEFINITIONS  CH_API_COMPILE_UNIT_JS
	                          COMPILE_FLAGS       "${CH_JAVASCRIPTFLAGS}"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_JSDBG"
	                          ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_JSDBG")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_JS
		${CH_JAVASCRIPTLIB} 
		optimized 	${CH_CHRONOLIB}
		debug 		${CH_CHRONOLIB_DBG}  )
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_JAVASCRIPTINC      	"${CH_JAVASCRIPTINC}" 		PARENT_SCOPE )
	SET (CH_CHRONOJSLIB  		"${CH_CHRONOJSLIB}" 	 	PARENT_SCOPE )
	SET (CH_CHRONOJSLIB_DBG  	"${CH_CHRONOJSLIB_DBG}" 	PARENT_SCOPE )
		
		
ENDIF(ENABLE_UNIT_JS)