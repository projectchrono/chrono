#--------------------------------------------------------------
# Add executables


ADD_EXECUTABLE(demo_socket   		demo_socket.cpp)
SOURCE_GROUP(demos\\cosimulation FILES  demo_socket.cpp)
SET_TARGET_PROPERTIES(demo_socket PROPERTIES 
	FOLDER cosimulation
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_socket ChronoEngine ChronoEngine_COSIMULATION)
ADD_DEPENDENCIES (demo_socket ChronoEngine ChronoEngine_COSIMULATION)




install(TARGETS demo_socket DESTINATION bin)

