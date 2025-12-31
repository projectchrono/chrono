Chrono FSI module tutorials {#tutorial_table_of_content_chrono_fsi}
===========================

The Chrono distribution includes several demos for simulation of fluid-solid interaction problems with the [FSI module](group__fsi.html).

- Compressibility (demo_FSI_Compressibility.cpp): This demo shows how to measure the amount of compressibility (density error) of an SPH solver using a static tank model.

- Dam Break (demo_FSI_DamBreak.cpp): Classical dam break problem modeled with SPH method.

- Cable with ANCF Beams (demo_FSI_Flexible_Cable.cpp): This demo shows interaction of fluid with a flexible plate modeled via ANCF cable elements. The fluid model is similar to the dam break problem.

- Plate with ANCF Shells (demo_FSI_Flexible_Plate.cpp): This demo shows interaction of fluid with a flexible plate modeled via ANCF shell elements. The fluid model is similar to the dam break problem.

- Angle of Repose (demo_FSI_AngleRepose.cpp): This demo demonstrates an angle of repose test with granular material modeled as a continuum using WCSPH method.

- Sphere Cratering (demo_FSI_Cratering.cpp): This demo shows the interaction of a falling sphere with a granular material bed. This demo can be used to validate against experimental/analytical results available in the [literature](https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff).

- Wave tank (demo_FSI_WaveTank.cpp): This demo shows the generation of waves in a wave tank using the FSI module.

Some demos with rigid body systems such as rovers, vehicles, etc are also available:

- VIPER on GRC1 Terrain Bed (demo_FSI_Viper_SPH.cpp): This demo shows the interaction of a VIPER rover with a GRC1 terrain bed modelled using the FSI module.

- Wheeled Vehicle on Granular Material Terrain (demo_VEH_CRMTerrain_WheeledVehicle.cpp): This demo shows the interaction of a wheeled vehicle with terrain modeled using the FSI module.