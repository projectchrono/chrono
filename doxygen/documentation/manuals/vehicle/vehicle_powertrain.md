Powertrain models {#vehicle_powertrain}
=======================================

\tableofcontents

The powertrain system is connected to the driver system (receiving throttle inputs) and to the driveline sub-system of the vehicle system (receiving the driveshaft angular speed and sending the driveshaft torque).

<img src="http://www.projectchrono.org/assets/manual/vehicle/powertrain_data_flow.png" width="600" />

Chrono::Vehicle is designed to permit linking to a third-party powertrain model. Interfacing to such an external powertrain model requires two thin interface classes, derived from [ChEngine](@ref chrono::vehicle::ChEngine) and [ChTransmission](@ref chrono::vehicle::ChTransmission).

Chrono::Vehicle provides several templates for the powertrain system, described below.


## Shafts-based powertrain model {#vehicle_powertrain_shafts}

This powertrain model is based on various Chrono 1-D shaft elements ([ChShaft](@ref chrono::ChShaft)) and specialized shaft connection elements. Such elements are used to model:
- the engine ([ChShaftsThermalEngine](@ref chrono::ChShaftsThermalEngine)), specified through speed-torque curves
- the torque converter ([ChShaftsTorqueConverter](@ref chrono::ChShaftsTorqueConverter)), specified through curves for the capacity factor as function of the speed ratio and the torque ratio as function of the speed ratio
- the gear box (manual or automatic, with an arbitrary number of forward gears and a reverse gear)

The motor block is attached to the chassis through a special Chrono constraint ([ChShaftBodyRotation](@ref chrono::ChShaftBodyRotation)) thus permitting simulating motor torque effects on the chassis. 

See [ChEngineShafts](@ref chrono::vehicle::ChEngineShafts),  [ChAutomaticTransmissionShafts](@ref chrono::vehicle::ChAutomaticTransmissionShafts), and [ChManualTransmissionShafts](@ref chrono::vehicle::ChManualTransmissionShafts)

The image below shows the shafts-based powertrain model connected to a shafts-based wheeled vehicle model.

<img src="http://www.projectchrono.org/assets/manual/vehicle/shafts_powertrain.png" width="800" />

Sample JSON files with the specification of a shafts-based powertrain, illustrating the various template parameters, are provided below:
\include "data/vehicle/hmmwv/powertrain/HMMWV_EngineShafts.json"
\include "data/vehicle/hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json"


The curves encoded in the above JSON file are shown below.

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/ShaftsPowertrain_engine_curves.png" width="500" />
<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/ShaftsPowertrain_TC_curves.png" width="500" />


## Engine map model {#vehicle_powertrain_map}

This template for a simple powertrain model is based on speed-torque engine maps. The model has no torque converter and can have either a manual or an automatic transmission. It accepts a single reverse gear and any number of forward gears. In automatic mode, gear shifting is done based on specified ideal shift points.

See [ChEngineSimpleMap](@ref chrono::vehicle::ChEngineSimpleMap) and [ChAutomaticTransmissionSimpleMap](@ref chrono::vehicle::ChAutomaticTransmissionSimpleMap).

Sample JSON files with the specification of a map powertrain, illustrating the various template parameters, are provided below.
\include "data/vehicle/uaz/powertrain/UAZBUS_EngineSimpleMap.json"
\include "data/vehicle/uaz/powertrain/UAZBUS_AutomaticTransmissionSimpleMap.json"

The curves encoded in the above JSON file are shown below.

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/SimpleMapPowertrain_engine_curves.png" width="500" />
<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/SimpleMapPowertrain_shift_bands.png" width="500" />


## Simple powertrain model {#vehicle_powertrain_simple}

This model uses a trivial speed-torque dependency, has no torque converter and no transmission box.

See [ChEngineSimple](@ref chrono::vehicle::ChEngineSimple) and [EngineSimple](@ref chrono::vehicle::EngineSimple).

A sample JSON file with the specification of a CVT-like powertrain, illustrating the various template parameters, is provided below.
\include "data/vehicle/hmmwv/powertrain/HMMWV_EngineSimple.json"
