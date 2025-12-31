<p align="center">
<img src="https://github.com/projectchrono/chrono/blob/main/contrib/logo/Chrono_alpha.png" width="200">
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
<img src="https://github.com/projectchrono/fmu-forge/blob/main/fmu-forge.png" width="150">
</p>

<br><br>

# Chrono::FMI examples

To illustrate the export and import capabilities of fmu-forge, we provide the following FMU examples:

<details>
<summary> Co-simulation </summary>
  
- Hydraulic actuator (FMI-2.0): [actuatorFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu2_actuator)
- Double pendulum crane (FMI-2.0): [craneFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu2_crane)
- Hydraulic actuator (FMI-3.0): [actuatorFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu3_actuator)
- Double pendulum crane (FMI-3.0): [craneFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu3_crane)
  
</details>

<details>
<summary> Model exchange </summary>
  
- Van der Pol equation (FMI-2.0): [vdpFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/modex/fmu2_vdp)
- Van der Pol equation (FMI-3.0): [vdpFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/modex/fmu3_vdp)
- Hydraulic actuator (FMI-2.0): [actuatorFMU](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/modex/fmu2_actuator)
  
</details>

For additional examples of Chrono-based FMUs, see the Chrono::Vehicle [FMI demos](https://github.com/projectchrono/chrono/tree/main/src/demos/vehicle/fmi).

## FMU export compatibility information

During export, any FMU generated with fmu-forge can be automatically tested with the `fmusim` validation tool (see [Reference-FMUs](https://github.com/modelica/Reference-FMUs)).<br>
This option, available for both FMI 2.0 and FMI 3.0, can be enabled by setting the `FMU_TESTING` CMake variable to `ON`. See for example [CMakeLists.txt](https://github.com/projectchrono/fmu-forge/blob/01cda9654dc48adbd310267de1915a11f369e250/examples/fmi2/cosimulation/CMakeLists.txt#L10) for the FMI 2.0 co-simulation examples.<br>
All Chrono example FMUs provided here have been validated with `fmusim`. 

The two FMI 2.0 CS FMUs for a [double-pendulum crane](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu2_crane) and a [hydraulic actuator](https://github.com/projectchrono/chrono/tree/main/src/demos/fmi/cosim/fmu2_actuator) have been imported and co-simulated using [Simulink](https://mathworks.com/products/simulink.html).<br>
See the Simulink [model](https://github.com/projectchrono/chrono/tree/main/data/fmi). 

## FMU import compatibility information

The following example driver programs illustrate the FMU import:

- CS FMI-2.0 [demo_FMI2_hydraulic_crane_cosim](https://github.com/projectchrono/chrono/blob/main/src/demos/fmi/cosim/demo_FMI2_hydraulic_crane_cosim.cpp)
- CS FMI-3.0: [demo_Cosimulation_fmi3](https://github.com/projectchrono/fmu-forge/blob/main/examples/fmi3/cosimulation/demo_Cosimulation_fmi3.cpp)
  
- ME FMI-2.0 and FMI-3.0: [demo_FMI_VdP_modex](https://github.com/projectchrono/chrono/blob/main/src/demos/fmi/modex/demo_FMI_VdP_modex.cpp)
- ME FMI-2.0 and FMI-3.0: [demo_FMI_hydraulic_crane_modex](https://github.com/projectchrono/chrono/blob/main/src/demos/fmi/modex/demo_FMI_hydraulic_crane_modex.cpp)
  
See also the demos for Chrono::Vehicle [FMI support]()
