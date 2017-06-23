# Additional include paths
INCLUDE_DIRECTORIES( ${CH_MUMPS_INCLUDES} )
 
MESSAGE(STATUS "Demo programs for Mumps module...")

# ---------------------------------------------------------------------

MESSAGE(STATUS "...add demo_MUMPS_MumpsEngine")
ADD_EXECUTABLE(demo_MUMPS_MumpsEngine demo_MUMPS_MumpsEngine)
SOURCE_GROUP("" FILES demo_MUMPS_MumpsEngine)
SET_TARGET_PROPERTIES(demo_MUMPS_MumpsEngine PROPERTIES 
                      FOLDER demos
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_MUMPS_MumpsEngine ChronoEngine ChronoEngine_mumps)
ADD_DEPENDENCIES(demo_MUMPS_MumpsEngine ChronoEngine ChronoEngine_mumps)
INSTALL(TARGETS demo_MUMPS_MumpsEngine DESTINATION bin)




