#=============================================================================
# CHRONO::ENGINE   CMake configuration file for OpenCASCADE unit
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_CASCADE     FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine CASCADE unit.")
SET(CH_CASCADEDIR           ""      CACHE PATH   "Where is your OpenCASCADE SDK installed? Ex: 'E:/OpenCASCADE6.3.0'. You must set this path to compile the CASCADE unit.")


# To hide unneeded option stuff from GUI (require refresh by pressing 'configure', anyway)
IF(ENABLE_UNIT_CASCADE)
	MARK_AS_ADVANCED(CLEAR CH_CASCADEDIR)	
ELSE()
	MARK_AS_ADVANCED(FORCE CH_CASCADEDIR)
ENDIF()


IF(ENABLE_UNIT_CASCADE)

	#-----------------------------------------------------------------------------
	#
	# LIST THE FILES THAT MAKE THE OpenCASCADE LIBRARY
	# NOTE: to add a new source to this unit, just add its name
	# here and re-run the CMake.
	#
	
	SET(ChronoEngine_unit_CASCADE_SOURCES 
		ChCascadeMeshTools.cpp
		ChCascadeDoc.cpp
		)
	SET(ChronoEngine_unit_CASCADE_HEADERS
		ChCascadeMeshTools.h
		ChCascadeDoc.h
		ChIrrCascadeMeshTools.h
		)
	#SET_SOURCE_FILES_PROPERTIES(ChronoEngine_unit_CASCADE_HEADERS PROPERTIES  HEADER_FILE_ONLY)
	SOURCE_GROUP(unit_CASCADE FILES 
				${ChronoEngine_unit_CASCADE_SOURCES} 
				${ChronoEngine_unit_CASCADE_HEADERS})
				
	
				
	#-----------------------------------------------------------------------------	
	# In most cases, you do not need to edit the lines below.
	


	SET (CH_CASCADELIBDIR "${CH_CASCADEDIR}/ros/win32/lib/")
	SET (CH_CASCADEINC "${CH_CASCADEDIR}/ros/inc")
	SET (CH_CASCADELIBS 
			${CH_CASCADELIBDIR}BinLPlugin.lib
 			${CH_CASCADELIBDIR}BinPlugin.lib
 			${CH_CASCADELIBDIR}BinTObjPlugin.lib
 			${CH_CASCADELIBDIR}BinXCAFPlugin.lib
 			${CH_CASCADELIBDIR}FWOSPlugin.lib
 			${CH_CASCADELIBDIR}mscmd.lib
 			${CH_CASCADELIBDIR}PTKernel.lib
 			${CH_CASCADELIBDIR}StdLPlugin.lib
 			${CH_CASCADELIBDIR}StdPlugin.lib
 			${CH_CASCADELIBDIR}TKAdvTools.lib
 			${CH_CASCADELIBDIR}TKBin.lib
 			${CH_CASCADELIBDIR}TKBinL.lib
 			${CH_CASCADELIBDIR}TKBinTObj.lib
 			${CH_CASCADELIBDIR}TKBinXCAF.lib
 			${CH_CASCADELIBDIR}TKBO.lib
 			${CH_CASCADELIBDIR}TKBool.lib
 			${CH_CASCADELIBDIR}TKBRep.lib
 			${CH_CASCADELIBDIR}TKCAF.lib
 			${CH_CASCADELIBDIR}TKCDF.lib
 			${CH_CASCADELIBDIR}TKCDLFront.lib
 			${CH_CASCADELIBDIR}TKCPPClient.lib
 			${CH_CASCADELIBDIR}TKCPPExt.lib
 			${CH_CASCADELIBDIR}TKCPPIntExt.lib
 			${CH_CASCADELIBDIR}TKCPPJini.lib
 			${CH_CASCADELIBDIR}TKCSFDBSchema.lib
 			${CH_CASCADELIBDIR}TKDCAF.lib
 			${CH_CASCADELIBDIR}TKDraw.lib
 			${CH_CASCADELIBDIR}TKernel.lib
 			${CH_CASCADELIBDIR}TKFeat.lib
 			${CH_CASCADELIBDIR}TKFillet.lib
 			${CH_CASCADELIBDIR}TKG2d.lib
 			${CH_CASCADELIBDIR}TKG3d.lib
 			${CH_CASCADELIBDIR}TKGeomAlgo.lib
 			${CH_CASCADELIBDIR}TKGeomBase.lib
 			${CH_CASCADELIBDIR}TKHLR.lib
 			${CH_CASCADELIBDIR}TKIDLFront.lib
 			${CH_CASCADELIBDIR}TKIGES.lib
 			${CH_CASCADELIBDIR}TKjcas.lib
 			${CH_CASCADELIBDIR}TKLCAF.lib
 			${CH_CASCADELIBDIR}TKMath.lib
 			${CH_CASCADELIBDIR}TKMesh.lib
 			${CH_CASCADELIBDIR}TKMeshVS.lib
 			${CH_CASCADELIBDIR}TKNIS.lib
 			${CH_CASCADELIBDIR}TKOffset.lib
 			${CH_CASCADELIBDIR}TKOpenGl.lib
 			${CH_CASCADELIBDIR}TKPCAF.lib
 			${CH_CASCADELIBDIR}TKPLCAF.lib
 			${CH_CASCADELIBDIR}TKPrim.lib
 			${CH_CASCADELIBDIR}TKPShape.lib
 			${CH_CASCADELIBDIR}TKService.lib
 			${CH_CASCADELIBDIR}TKShapeSchema.lib
 			${CH_CASCADELIBDIR}TKShHealing.lib
 			${CH_CASCADELIBDIR}TKStdLSchema.lib
 			${CH_CASCADELIBDIR}TKStdSchema.lib
 			${CH_CASCADELIBDIR}TKSTEP.lib
 			${CH_CASCADELIBDIR}TKSTEP209.lib
 			${CH_CASCADELIBDIR}TKSTEPAttr.lib
 			${CH_CASCADELIBDIR}TKSTEPBase.lib
 			${CH_CASCADELIBDIR}TKSTL.lib
 			${CH_CASCADELIBDIR}TKTCPPExt.lib
 			${CH_CASCADELIBDIR}TKTObj.lib
 			${CH_CASCADELIBDIR}TKTObjDRAW.lib
 			${CH_CASCADELIBDIR}TKTopAlgo.lib
 			${CH_CASCADELIBDIR}TKTopTest.lib
 			${CH_CASCADELIBDIR}TKV2d.lib
 			${CH_CASCADELIBDIR}TKV3d.lib
 			${CH_CASCADELIBDIR}TKViewerTest.lib
 			${CH_CASCADELIBDIR}TKVRML.lib
 			${CH_CASCADELIBDIR}TKWOK.lib
 			${CH_CASCADELIBDIR}TKWOKTcl.lib
 			${CH_CASCADELIBDIR}TKXCAF.lib
 			${CH_CASCADELIBDIR}TKXCAFSchema.lib
 			${CH_CASCADELIBDIR}TKXDEDRAW.lib
 			${CH_CASCADELIBDIR}TKXDEIGES.lib
 			${CH_CASCADELIBDIR}TKXDESTEP.lib
 			${CH_CASCADELIBDIR}TKXMesh.lib
 			${CH_CASCADELIBDIR}TKXml.lib
 			${CH_CASCADELIBDIR}TKXmlL.lib
 			${CH_CASCADELIBDIR}TKXmlTObj.lib
 			${CH_CASCADELIBDIR}TKXmlXCAF.lib
 			${CH_CASCADELIBDIR}TKXSBase.lib
 			${CH_CASCADELIBDIR}TKXSDRAW.lib
 			${CH_CASCADELIBDIR}XCAFPlugin.lib
 			${CH_CASCADELIBDIR}XmlLPlugin.lib
 			${CH_CASCADELIBDIR}XmlPlugin.lib
 			${CH_CASCADELIBDIR}XmlTObjPlugin.lib
 			${CH_CASCADELIBDIR}XmlXCAFPlugin.lib
 	)
	
	IF ("${CH_OS}" STREQUAL "LINUX")
		MESSAGE( "WARNING!, OpenCASCADE unit building under LINUX is not yet tested! Check libs paths etc!")
	ENDIF()
	
	
	SET (CH_CHRONOCASCADELIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_CASCADE${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOCASCADESHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_CASCADE${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOCASCADELIB  "${CH_CHRONOCASCADELIBRARYNAME}")
	SET (CH_CHRONOCASCADELIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_CASCADEDBG${CH_LIBNAMEPOSTFIX}")
	SET (CH_CHRONOCASCADESHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_CASCADEDBG${CH_LIBNAMEPOSTFIXSHARED}")
	SET (CH_CHRONOCASCADELIB_DBG  "${CH_CHRONOCASCADELIBRARYNAME_DBG}")
	
	
	INCLUDE_DIRECTORIES( "${CH_CASCADEINC}" )
	
	ADD_DEFINITIONS( "/DWNT" )
	
	
	# The CASCADE library is added to the project,
	# and some custom properties of this target are set.
	
	ADD_LIBRARY(ChronoEngine_CASCADE SHARED 
				${ChronoEngine_unit_CASCADE_SOURCES}
				${ChronoEngine_unit_CASCADE_HEADERS})
	
	SET_TARGET_PROPERTIES(ChronoEngine_CASCADE PROPERTIES 
	                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_CASCADE"
	                          ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CH_LIBDIR}"
	                          ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CH_LIBDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	                          LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}"
	                           RUNTIME_OUTPUT_NAME_DEBUG "ChronoEngine_CASCADEDBG"
	                           LIBRARY_OUTPUT_NAME_DEBUG "ChronoEngine_CASCADEDBG"
	                           ARCHIVE_OUTPUT_NAME_DEBUG "ChronoEngine_CASCADEDBG")
	                          
	TARGET_LINK_LIBRARIES(ChronoEngine_CASCADE
		${CH_CASCADELIBS} 
		optimized 	${CH_CHRONOLIB}
		debug 		${CH_CHRONOLIB_DBG}  )
		
	ADD_DEPENDENCIES (ChronoEngine_CASCADE ChronoEngine)  # better, because not automatic
		
		
	# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
	
	SET (CH_CASCADEINC      		"${CH_CASCADEINC}" 			 PARENT_SCOPE )
	SET (CH_CASCADELIBS      		"${CH_CASCADELIBS}" 		 PARENT_SCOPE )
	SET (CH_CHRONOCASCADELIB  		"${CH_CHRONOCASCADELIB}" 	 PARENT_SCOPE )
	SET (CH_CHRONOCASCADELIB_DBG  	"${CH_CHRONOCASCADELIB_DBG}" PARENT_SCOPE )
		
		
ENDIF(ENABLE_UNIT_CASCADE)
