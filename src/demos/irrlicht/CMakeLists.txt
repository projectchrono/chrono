#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} 
				    )


#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_benchmark   	demo_benchmark.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_benchmark.cpp)
SET_TARGET_PROPERTIES(demo_benchmark PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_benchmark
	${CH_IRRLICHTLIB}
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_benchmark ChronoEngine)

 	
ADD_EXECUTABLE(demo_bricks   	    demo_bricks.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_bricks.cpp)
SET_TARGET_PROPERTIES(demo_bricks PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_bricks
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_bricks ChronoEngine)

 	
ADD_EXECUTABLE(demo_cohesion   	    demo_cohesion.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_cohesion.cpp)
SET_TARGET_PROPERTIES(demo_cohesion PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_cohesion
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_cohesion ChronoEngine)


ADD_EXECUTABLE(demo_conveyor   	    demo_conveyor.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_conveyor.cpp)
SET_TARGET_PROPERTIES(demo_conveyor PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_conveyor
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_conveyor ChronoEngine)


ADD_EXECUTABLE(demo_crank   	    demo_crank.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_crank.cpp)
SET_TARGET_PROPERTIES(demo_crank PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_crank
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_crank ChronoEngine)

 	
ADD_EXECUTABLE(demo_decomposition   demo_decomposition.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_decomposition.cpp)
SET_TARGET_PROPERTIES(demo_decomposition PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_decomposition
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_decomposition ChronoEngine)

 	
ADD_EXECUTABLE(demo_forklift    	demo_forklift.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_forklift.cpp)
SET_TARGET_PROPERTIES(demo_forklift PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_forklift
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_forklift ChronoEngine)

 	
ADD_EXECUTABLE(demo_fourbar    		demo_fourbar.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_fourbar.cpp)
SET_TARGET_PROPERTIES(demo_fourbar PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_fourbar
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_fourbar ChronoEngine)

 	
ADD_EXECUTABLE(demo_friction   		demo_friction.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_friction.cpp)
SET_TARGET_PROPERTIES(demo_friction PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_friction
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_friction ChronoEngine)

 	
ADD_EXECUTABLE(demo_gears   		demo_gears.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_gears.cpp)
SET_TARGET_PROPERTIES(demo_gears PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_gears
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_gears ChronoEngine)

 	
ADD_EXECUTABLE(demo_mecanum   		demo_mecanum.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_mecanum.cpp)
SET_TARGET_PROPERTIES(demo_mecanum PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_mecanum
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_mecanum ChronoEngine)

 	
ADD_EXECUTABLE(demo_meshless   		demo_meshless.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_meshless.cpp)
SET_TARGET_PROPERTIES(demo_meshless PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_meshless
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_meshless ChronoEngine)

 	
ADD_EXECUTABLE(demo_pendulum   		demo_pendulum.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_pendulum.cpp)
SET_TARGET_PROPERTIES(demo_pendulum PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_pendulum
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_pendulum ChronoEngine)

 	
ADD_EXECUTABLE(demo_sph   		    demo_sph.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_sph.cpp)
SET_TARGET_PROPERTIES(demo_sph PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_sph
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_sph ChronoEngine)

 	
ADD_EXECUTABLE(demo_suspension   	demo_suspension.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_suspension.cpp)
SET_TARGET_PROPERTIES(demo_suspension PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_suspension
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_suspension ChronoEngine)


ADD_EXECUTABLE(demo_tire   			demo_tire.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_tire.cpp)
SET_TARGET_PROPERTIES(demo_tire PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_tire
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_tire ChronoEngine)

 	
ADD_EXECUTABLE(demo_tracks   		demo_tracks.cpp)
SOURCE_GROUP(demos\\irrlicht FILES  demo_tracks.cpp)
SET_TARGET_PROPERTIES(demo_tracks PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CH_BINDIR}"
	RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CH_BINDIR}")
TARGET_LINK_LIBRARIES(demo_tracks
	${CH_IRRLICHTLIB} 
 	optimized 	${CH_CHRONOLIB} 
 	debug 		${CH_CHRONOLIB_DBG})
ADD_DEPENDENCIES (demo_tracks ChronoEngine)