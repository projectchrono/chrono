# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MKLINC} )
INCLUDE_DIRECTORIES( ${CH_EIGEN_PATH} )
INCLUDE_DIRECTORIES( ${ChronoEngine_UNIT_MKL_INCLUDE})

ADD_EXECUTABLE(demo_LcpMklSolver demo_LcpMklSolver.cpp demo_LcpMklSolver.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_LcpMklSolver PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_LcpMklSolver
 	ChronoEngine
 	ChronoEngine_MKL
	)
ADD_DEPENDENCIES (demo_LcpMklSolver ChronoEngine ChronoEngine_MKL)
INSTALL(TARGETS demo_LcpMklSolver DESTINATION bin)

# ---------------------------------------------------------------------

ADD_EXECUTABLE(demo_sandbox demo_sandbox.cpp demo_sandbox.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_sandbox PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_sandbox
 	ChronoEngine
 	ChronoEngine_MKL
	)
ADD_DEPENDENCIES (demo_sandbox ChronoEngine ChronoEngine_MKL)
INSTALL(TARGETS demo_sandbox DESTINATION bin)

# ---------------------------------------------------------------------

ADD_EXECUTABLE(profiling_sandbox profiling_sandbox.cpp profiling_sandbox.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(profiling_sandbox PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(profiling_sandbox
 	ChronoEngine
 	ChronoEngine_MKL
	"C:/Program Files (x86)/Microsoft Visual Studio 12.0/Team Tools/Performance Tools/x64/PerfSDK/VSPerf.lib"
	)
INCLUDE_DIRECTORIES("C:/Program Files (x86)/Microsoft Visual Studio 12.0/Team Tools/Performance Tools/x64/PerfSDK")
ADD_DEPENDENCIES (profiling_sandbox ChronoEngine ChronoEngine_MKL)
INSTALL(TARGETS profiling_sandbox DESTINATION bin)

