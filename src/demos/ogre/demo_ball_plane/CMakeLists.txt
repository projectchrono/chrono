SET(MAIN_FILES
	demo_ball_plane_main.cpp
	)
	
SOURCE_GROUP("" FILES ${MAIN_FILES})

ADD_EXECUTABLE(demo_ball_plane ${MAIN_FILES})

include_directories("../")
	
TARGET_LINK_LIBRARIES(demo_ball_plane ${LIBRARIES} ChOgre)

ADD_DEPENDENCIES(demo_ball_plane ChOgre)

set_target_properties( demo_ball_plane
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/debug"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/bin/debug"
)

set_target_properties( demo_ball_plane
    PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/release"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/release"
)
