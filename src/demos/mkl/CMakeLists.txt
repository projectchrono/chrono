# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MKLINC} )

MESSAGE(STATUS "Demo programs for MKL module...")

# ---------------------------------------------------------------------

MESSAGE(STATUS "...add demo_MklEngine")
ADD_EXECUTABLE(demo_MklEngine demo_MklEngine.cpp)
SOURCE_GROUP("" FILES demo_MklEngine.cpp)
SET_TARGET_PROPERTIES(demo_MklEngine PROPERTIES 
                      FOLDER demos
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_MklEngine ChronoEngine ChronoEngine_mkl)
ADD_DEPENDENCIES(demo_MklEngine ChronoEngine ChronoEngine_mkl)
INSTALL(TARGETS demo_MklEngine DESTINATION bin)



