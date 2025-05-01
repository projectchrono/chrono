SET(LIBRARIES Chrono_core Chrono_ros)

SET(TESTS
  utest_ROS_custom_handler_with_manager
  utest_ROS_custom_handler
  utest_ROS_simple_publisher_subscriber
)

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

MESSAGE(STATUS "Add unit test programs for ROS module")

FOREACH(PROGRAM ${TESTS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} gtest_main)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    ADD_TEST(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
ENDFOREACH(PROGRAM)
