#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_FEMbasic  	demo_FEMbasic.cpp)
SOURCE_GROUP(demos\\fem FILES  		demo_FEMbasic.cpp)
SET_TARGET_PROPERTIES(demo_FEMbasic PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_FEMbasic 
 	ChronoEngine)
ADD_DEPENDENCIES (demo_FEMbasic ChronoEngine)

install(TARGETS demo_FEMbasic DESTINATION bin)