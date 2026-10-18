option(CH_ENABLE_MODULE_ROBOT_MODELS "Build the Chrono robot models library" TRUE)
set(CH_ENABLE_MODULE_ROBOT_MODELS "${CH_ENABLE_MODULE_ROBOT_MODELS}" PARENT_SCOPE)  

if(CH_ENABLE_MODULE_VEHICLE)
  option(CH_ENABLE_MODULE_VEHICLE_MODELS "Build the Chrono vehicle models library" TRUE)
  set(CH_ENABLE_MODULE_VEHICLE_MODELS "${CH_ENABLE_MODULE_VEHICLE_MODELS}" PARENT_SCOPE)
else()
  set(CH_ENABLE_MODULE_VEHICLE_MODELS FALSE CACHE BOOL "Build the Chrono vehicle models library" FORCE)
endif()

if(CH_ENABLE_MODULE_ROBOT_MODELS OR CH_ENABLE_MODULE_VEHICLE_MODELS)
  message(STATUS "\n==== Chrono models ====\n")
endif()

if(CH_ENABLE_MODULE_ROBOT_MODELS)
  add_subdirectory(robot)
endif()

if(CH_ENABLE_MODULE_VEHICLE_MODELS)
  add_subdirectory(vehicle)
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)
