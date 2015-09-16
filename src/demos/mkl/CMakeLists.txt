# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MKL_INCLUDES} )

MESSAGE(STATUS "Demo programs for MKL module...")

# ---------------------------------------------------------------------

MESSAGE(STATUS "...add demo_sandbox")
ADD_EXECUTABLE(demo_sandbox demo_sandbox.cpp)
SOURCE_GROUP("" FILES demo_pardiso.cpp)
SET_TARGET_PROPERTIES(demo_sandbox PROPERTIES 
                      FOLDER demos
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_sandbox ChronoEngine ChronoEngine_mkl)
ADD_DEPENDENCIES(demo_sandbox ChronoEngine ChronoEngine_mkl)
INSTALL(TARGETS demo_sandbox DESTINATION bin)


# ---------------------------------------------------------------------

MESSAGE(STATUS "...add demo_unit_test_ChCSR3Matrix")
ADD_EXECUTABLE(demo_unit_test_ChCSR3Matrix demo_unit_test_ChCSR3Matrix.cpp)
SOURCE_GROUP("" FILES demo_unit_test_ChCSR3Matrix.cpp)
SET_TARGET_PROPERTIES(demo_unit_test_ChCSR3Matrix PROPERTIES 
                      FOLDER demos
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_unit_test_ChCSR3Matrix ChronoEngine ChronoEngine_mkl)
ADD_DEPENDENCIES(demo_unit_test_ChCSR3Matrix ChronoEngine ChronoEngine_mkl)
INSTALL(TARGETS demo_unit_test_ChCSR3Matrix DESTINATION bin)


# ---------------------------------------------------------------------

#MESSAGE(STATUS "...add profiling_sandbox")
#ADD_EXECUTABLE(profiling_sandbox profiling_sandbox.cpp profiling_sandbox.h)
#SOURCE_GROUP("" FILES demo_pardiso.cpp demo_pardiso.h)
#SET_TARGET_PROPERTIES(profiling_sandbox PROPERTIES 
#	FOLDER demos
#	LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(profiling_sandbox
# 	ChronoEngine
# 	ChronoEngine_mkl
#	"C:/Program Files (x86)/Microsoft Visual Studio 12.0/Team Tools/Performance Tools/x64/PerfSDK/VSPerf.lib")
#INCLUDE_DIRECTORIES("C:/Program Files (x86)/Microsoft Visual Studio 12.0/Team Tools/Performance Tools/x64/PerfSDK")
#ADD_DEPENDENCIES (profiling_sandbox ChronoEngine ChronoEngine_mkl)
#INSTALL(TARGETS profiling_sandbox DESTINATION bin)

