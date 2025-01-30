set(PROGRAM demo_VEH_Cosim_WheelRig_CustomTerrain)

add_executable(${PROGRAM} ${PROGRAM}.cpp)
source_group("" FILES ${PROGRAM}.cpp)

target_link_libraries(${PROGRAM} ChronoEngine_vehicle_cosim)
if(ENABLE_MODULE_IRRLICHT)
  target_link_libraries(${PROGRAM} ChronoEngine_irrlicht)
endif()