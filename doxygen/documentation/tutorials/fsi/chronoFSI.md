Chrono FSI module tutorials {#tutorial_table_of_content_chrono_fsi}
===========================

The Chrono distribution includes several demos for simulation of fluid-solid interaction problems with the [FSI module](group__fsi.html), using both the SPH and TDPF fluid solvers.

### FSI demos with the Chrono::FSI-SPH module

- [demo_FSI-SPH_Compressibility](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_Compressibility.cpp)
  This demo shows how to measure the amount of compressibility (density error) of an SPH solver using a static tank model.

- [demo_FSI-SPH_ObjectDrop](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_ObjectDrop.cpp)
  Object dropped in rectangular water tank.

- [demo_FSI-SPH_CylindricalTank](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_CylindricalTank.cpp)
  Demonstration of FSI-SPH problems in cylindrical coordinates

- [demo_FSI-SPH_DamBreak](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_DamBreak.cpp)
  Classical dam break problem modeled with the SPH method.

- [demo_FSI-SPH_Flexible_Cable](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_Flexible_Cable.cpp) 
  This demo shows interaction of fluid with a flexible beam modeled with ANCF cable elements.

- [demo_FSI-SPH_Flexible_Plate](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_Flexible_Plate.cpp)
  This demo shows interaction of fluid with a flexible plate modeled with ANCF shell elements.

- [demo_FSI-SPH_AngleRepose](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_AngleRepose.cpp)
  Angle of repose test with granular material modeled as a continuum using the CRM method.

- [demo_FSI-APH_Cratering](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_Cratering.cpp)
  Interaction of a falling sphere with a granular material bed. This demo can be used to validate against experimental/analytical results available in the [literature](https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff).

- [demo_FSI_WaveTank](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/sph/demo_FSI-SPH_WaveTank.cpp)
  Generation of waves in a wave tank using the FSI-SPH module.

Additional demos using the Chrono::FSI-SPH module involve vehicle and rovers over deformable CRM terrain. Some examples include:

- [demo_VEH_CRMTerrain_WheeledVehicle](https://github.com/projectchrono/chrono/blob/main/src/demos/vehicle/terrain/demo_VEH_CRMTerrain_WheeledVehicle.cpp)
  Interaction of a wheeled vehicle with CRM terrain modeled.

- [demo_VEH_CRMTerrain_TrackedVehicle](https://github.com/projectchrono/chrono/blob/main/src/demos/vehicle/terrain/demo_VEH_CRMTerrain_TrackedVehicle.cpp)
  Interaction of a tracked vehicle with CRM terrain modeled.

- [demo_ROBOT_Viper_CRM](https://github.com/projectchrono/chrono/blob/main/src/demos/robot/viper/demo_ROBOT_Viper_CRM.cpp)
  Interaction of the Viper Moon rover with deformable CRM terrain.

### FSI demos with the Chrono::FSI-TDPF module

- [demo_FSI-TDPF_sphere_decay](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/tdpf/demo_FSI-TDPF_sphere_decay.cpp)
  Decaying sphere bounce.

- [demo_FSI-TDPF_rm3_reg_waves](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/tdpf/demo_FSI-TDPF_rm3_reg_waves.cpp)
  Sphere floating in regular wave conditions.

- [demo_FSI-TDPF_sphere_irreg_waves](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/tdpf/demo_FSI-TDPF_sphere_irreg_waves.cpp)
  Sphere floating in irregular wave conditions.

- [demo_FSI-TDPF_oswec_reg_waves](https://github.com/projectchrono/chrono/blob/main/src/demos/fsi/tdpf/demo_FSI-TDPF_oswec_reg_waves.cpp)
  Flap-type wave energy converter.