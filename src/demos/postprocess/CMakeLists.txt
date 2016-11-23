#--------------------------------------------------------------
# Add executables

MESSAGE(STATUS "Demo programs for POSTPROCESS module...")

MESSAGE(STATUS "...add demo_postprocess")
ADD_EXECUTABLE(demo_postprocess   		demo_postprocess.cpp)
SOURCE_GROUP("" FILES  	    demo_postprocess.cpp)
SET_TARGET_PROPERTIES(demo_postprocess PROPERTIES 
	FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_postprocess  ChronoEngine  ChronoEngine_postprocess)
ADD_DEPENDENCIES (demo_postprocess ChronoEngine ChronoEngine_postprocess)

IF(ENABLE_MODULE_IRRLICHT)
	INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )
	MESSAGE(STATUS "...add demo_emit_creation")
	ADD_EXECUTABLE(demo_emit_creation   		demo_emit_creation.cpp)
	SOURCE_GROUP("" FILES  	    demo_emit_creation.cpp)
	SET_TARGET_PROPERTIES(demo_emit_creation PROPERTIES 
		FOLDER demos
		COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_emit_creation  
	ChronoEngine 
	ChronoEngine_postprocess 
	ChronoEngine_irrlicht
	${CH_IRRLICHTLIB}
	)
	ADD_DEPENDENCIES (demo_emit_creation ChronoEngine
	    ChronoEngine_postprocess ChronoEngine_irrlicht)

	MESSAGE(STATUS "...add demo_emitter_asset")
	ADD_EXECUTABLE(demo_emitter_asset   		demo_emitter_asset.cpp)
	SOURCE_GROUP("" FILES  	    demo_emitter_asset.cpp)
	SET_TARGET_PROPERTIES(demo_emitter_asset PROPERTIES 
		FOLDER demos
		COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(demo_emitter_asset  
	ChronoEngine 
	ChronoEngine_postprocess 
	ChronoEngine_irrlicht
	${CH_IRRLICHTLIB}
	)
	ADD_DEPENDENCIES (demo_emitter_asset ChronoEngine
	    ChronoEngine_postprocess ChronoEngine_irrlicht)

ENDIF()

MESSAGE(STATUS "...add demo_timestepping")
ADD_EXECUTABLE(demo_timestepping	demo_timestepping.cpp)
SOURCE_GROUP("" FILES  	demo_timestepping.cpp)
SET_TARGET_PROPERTIES(demo_timestepping PROPERTIES 
	FOLDER demos
	COMPILE_FLAGS "${CH_CXX_FLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_timestepping ChronoEngine)
ADD_DEPENDENCIES (demo_timestepping ChronoEngine ChronoEngine_postprocess)

MESSAGE(STATUS "...add demo_gnuplot")
ADD_EXECUTABLE(demo_gnuplot	demo_gnuplot.cpp)
SOURCE_GROUP("" FILES  	demo_gnuplot.cpp)
SET_TARGET_PROPERTIES(demo_gnuplot PROPERTIES 
	FOLDER demos
	COMPILE_FLAGS "${CH_CXX_FLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_gnuplot ChronoEngine)
ADD_DEPENDENCIES (demo_gnuplot ChronoEngine ChronoEngine_postprocess)


install(TARGETS demo_postprocess DESTINATION ${CH_INSTALL_DEMO})
install(TARGETS demo_timestepping DESTINATION ${CH_INSTALL_DEMO})

