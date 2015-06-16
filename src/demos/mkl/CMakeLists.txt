#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_MKLINC} )
INCLUDE_DIRECTORIES( ${CH_EIGEN_PATH} )
<<<<<<< HEAD
INCLUDE_DIRECTORIES( ${CH_MKLROOT}/include )
# INCLUDE_DIRECTORIES ( "${CMAKE_SOURCE_DIR}/core")
				    
#--------------------------------------------------------------
# Add executables

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
# ADD_DEPENDENCIES (demo_pardiso ChronoEngine ChronoEngine_MKL)

# ----------------------
=======
INCLUDE_DIRECTORIES( ${CH_MKLROOT}/include 	)			
INCLUDE_DIRECTORIES( ${ChronoEngine_UNIT_MKL_INCLUDE})
>>>>>>> unit_MKL_throughLinkedList

ADD_EXECUTABLE(demo_LcpMklSolver demo_LcpMklSolver.cpp demo_LcpMklSolver.h)
# SOURCE_GROUP(demos//mkl FILES demo_pardiso.cpp demo_pardiso.h)
SET_TARGET_PROPERTIES(demo_LcpMklSolver PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE} ${CH_MKL_LINK_FLAGS}"
	)
TARGET_LINK_LIBRARIES(demo_LcpMklSolver
<<<<<<< HEAD
	${CH_MKLLIB}
=======
>>>>>>> unit_MKL_throughLinkedList
 	ChronoEngine
 	ChronoEngine_MKL
	)
ADD_DEPENDENCIES (demo_LcpMklSolver ChronoEngine ChronoEngine_MKL)


<<<<<<< HEAD
# install(TARGETS demo_dgemm_example DESTINATION bin)
# install(TARGETS demo_pardiso DESTINATION bin)
=======
install(TARGETS demo_LcpMklSolver DESTINATION bin)
>>>>>>> unit_MKL_throughLinkedList
