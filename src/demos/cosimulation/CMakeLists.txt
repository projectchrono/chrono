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


ADD_EXECUTABLE(demo_cosimulation   		demo_cosimulation.cpp)
SOURCE_GROUP(demos\\cosimulation FILES  demo_cosimulation.cpp)
SET_TARGET_PROPERTIES(demo_cosimulation PROPERTIES 
	FOLDER cosimulation
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_cosimulation ChronoEngine ChronoEngine_COSIMULATION)
ADD_DEPENDENCIES (demo_cosimulation ChronoEngine ChronoEngine_COSIMULATION)

install(TARGETS demo_cosimulation DESTINATION bin)


ADD_EXECUTABLE(demo_cosim_hydraulics   		demo_cosim_hydraulics.cpp)
SOURCE_GROUP(demos\\cosimulation FILES  demo_cosim_hydraulics.cpp)
SET_TARGET_PROPERTIES(demo_cosim_hydraulics PROPERTIES 
	FOLDER cosimulation
	LINK_FLAGS "${CH_LINKERFLAG_EXE}"
	)
TARGET_LINK_LIBRARIES(demo_cosim_hydraulics ChronoEngine ChronoEngine_COSIMULATION)
ADD_DEPENDENCIES (demo_cosim_hydraulics ChronoEngine ChronoEngine_COSIMULATION)

install(TARGETS demo_cosim_hydraulics DESTINATION bin)
