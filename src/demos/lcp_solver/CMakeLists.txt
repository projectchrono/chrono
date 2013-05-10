#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_LCPsolver  	demo_LCPsolver.cpp)
SOURCE_GROUP(demos\\lcp_solver FILES  		demo_LCPsolver.cpp)
SET_TARGET_PROPERTIES(demo_LCPsolver PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_LCPsolver 
 	ChronoEngine)
ADD_DEPENDENCIES (demo_LCPsolver ChronoEngine)

install(TARGETS demo_LCPsolver DESTINATION bin)