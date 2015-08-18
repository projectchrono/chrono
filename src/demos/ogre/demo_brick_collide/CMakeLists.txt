SET(MAIN_FILES
	demo_brick_collide.cpp
	)
	
SOURCE_GROUP("" FILES ${MAIN_FILES})

ADD_EXECUTABLE(demo_brick_collide ${MAIN_FILES})

include_directories("../")
	
TARGET_LINK_LIBRARIES(demo_brick_collide ${LIBRARIES} ChOgre)

ADD_DEPENDENCIES(demo_brick_collide ChOgre)

set_target_properties( demo_brick_collide
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/debug"
)

set_target_properties( demo_brick_collide
    PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/release"
)
