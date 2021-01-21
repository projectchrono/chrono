message(STATUS "==== Chrono models ====")

add_subdirectory(robosimian)
add_subdirectory(viper_rover)

if(ENABLE_MODULE_VEHICLE)
  	add_subdirectory(vehicle)
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)

