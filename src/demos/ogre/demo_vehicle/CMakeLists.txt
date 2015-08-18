SET(MAIN_FILES
	main.cpp
	VESuspensionDemo.cpp
	VESuspensionDemo.h
	VEVehicle.cpp
	VEVehicle.h
	)
	
SOURCE_GROUP("" FILES ${MAIN_FILES})

ADD_EXECUTABLE(demo_vehicle ${MAIN_FILES})

include_directories("../")
	
TARGET_LINK_LIBRARIES(demo_vehicle ${LIBRARIES} ChOgre)

ADD_DEPENDENCIES(demo_vehicle ChOgre)

set_target_properties( demo_vehicle
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/debug"
)

set_target_properties( demo_vehicle
    PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/release"
)
