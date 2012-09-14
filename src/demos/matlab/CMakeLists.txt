#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_MATLABINC} 
				    )

				    
#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_matlab   		demo_matlab.cpp)
SOURCE_GROUP(demos\\matlab FILES  	demo_matlab.cpp)
SET_TARGET_PROPERTIES(demo_matlab PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_matlab
	${CH_MATLABLIB}
 	ChronoEngine)
ADD_DEPENDENCIES (demo_matlab ChronoEngine)


ADD_EXECUTABLE(demo_chfunctions_plot  		demo_chfunctions_plot.cpp)
SOURCE_GROUP(demos\\matlab FILES  	demo_chfunctions_plot.cpp)
SET_TARGET_PROPERTIES(demo_chfunctions_plot PROPERTIES 
	FOLDER demos
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_chfunctions_plot
	${CH_MATLABLIB}
 	ChronoEngine)
ADD_DEPENDENCIES (demo_chfunctions_plot ChronoEngine)


install(TARGETS demo_matlab DESTINATION bin)
install(TARGETS demo_chfunctions_plot DESTINATION bin)