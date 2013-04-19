#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( 	${CH_CUDA_INC} 
			${CH_CUDA_SDK_INC} 
			)
		 	
SET(UNIT_GPU_INCLUDE_LIBS  ${CUDA_FRAMEWORK} ChronoEngine ChronoEngine_GPU ChronoEngine_OPENGL ChronoEngine_POSTPROCESS) 

#--------------------------------------------------------------
#ADD_EXECUTABLE(test_gravity_gpu	gravity.cpp)
#SET_TARGET_PROPERTIES(test_gravity_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_gravity_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_gravity_gpu ChronoEngine ChronoEngine_GPU)
#ADD_TEST(test_gravity_gpu ${PROJECT_BINARY_DIR}/bin/test_gravity_gpu)

#ADD_EXECUTABLE(test_revolute_gpu	revolute.cpp)
#SET_TARGET_PROPERTIES(test_revolute_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_revolute_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_revolute_gpu ChronoEngine ChronoEngine_GPU)
#ADD_TEST(test_revolute_gpu ${PROJECT_BINARY_DIR}/bin/test_revolute_gpu)

ADD_EXECUTABLE(test_solver_gpu	solver.cpp)
SET_TARGET_PROPERTIES(test_solver_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_solver_gpu ${UNIT_GPU_INCLUDE_LIBS})
ADD_DEPENDENCIES (test_solver_gpu ChronoEngine ChronoEngine_GPU)

ADD_EXECUTABLE(test_collision_gpu	collision.cpp)
SET_TARGET_PROPERTIES(test_collision_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(test_collision_gpu ${UNIT_GPU_INCLUDE_LIBS})
ADD_DEPENDENCIES (test_collision_gpu ChronoEngine ChronoEngine_GPU)
		 	
#ADD_EXECUTABLE(test_box_contact_gpu	box_contact.cpp)
#SET_TARGET_PROPERTIES(test_box_contact_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_box_contact_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_box_contact_gpu ChronoEngine ChronoEngine_GPU)
		 	
#ADD_EXECUTABLE(test_box_gpu	box.cpp)
#SET_TARGET_PROPERTIES(test_box_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_box_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_box_gpu ChronoEngine ChronoEngine_GPU)
	
#ADD_EXECUTABLE(test_bilateral_gpu	bilateral.cpp)
#SET_TARGET_PROPERTIES(test_bilateral_gpu PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_bilateral_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_bilateral_gpu ChronoEngine ChronoEngine_GPU)

#ADD_EXECUTABLE(test_revolute_gpu	revolute.cpp)
#SET_TARGET_PROPERTIES(test_revolute_gpu PROPERTIES PROPERTIES LINK_FLAGS "${CH_LINKERFLAG_EXE}")
#TARGET_LINK_LIBRARIES(test_revolute_gpu ${UNIT_GPU_INCLUDE_LIBS})
#ADD_DEPENDENCIES (test_revolute_gpu ChronoEngine ChronoEngine_GPU)


