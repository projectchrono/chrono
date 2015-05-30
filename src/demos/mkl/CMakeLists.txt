#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_MKLINC} )
				    
#--------------------------------------------------------------
# Add executables

SET(CH_EIGEN_PATH "D:/SourceTreeWS/libraries/eigen"     CACHE PATH   "Where is your Eigen library main folder? (should contain ./Engine)")

INCLUDE_DIRECTORIES( ${CH_EIGEN_PATH} )
INCLUDE_DIRECTORIES( ${CH_MKLROOT}/include )

ADD_EXECUTABLE(demo_sandbox demo_sandbox.cpp demo_sandbox.h SparseMatrixAddon.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_sandbox PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_sandbox
	${CH_MKLLIB}
 	ChronoEngine
 	# ChronoEngine_MKL
	)
# ADD_DEPENDENCIES (demo_sandbox ChronoEngine)

# --------------------

ADD_EXECUTABLE(demo_pardiso demo_pardiso.cpp demo_pardiso.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_pardiso PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_pardiso
	${CH_MKLLIB}
 	# ChronoEngine
 	# ChronoEngine_MKL
	)
# ADD_DEPENDENCIES (demo_dgemm_example ChronoEngine ChronoEngine_MKL)

# ----------------------

ADD_EXECUTABLE(demo_LcpMklSolver demo_LcpMklSolver.cpp demo_LcpMklSolver.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_LcpMklSolver PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_LcpMklSolver
	${CH_MKLLIB}
 	# ChronoEngine
 	# ChronoEngine_MKL
	)
# ADD_DEPENDENCIES (demo_dgemm_example ChronoEngine ChronoEngine_MKL)


# install(TARGETS demo_dgemm_example DESTINATION bin)
# install(TARGETS demo_pardiso DESTINATION bin)