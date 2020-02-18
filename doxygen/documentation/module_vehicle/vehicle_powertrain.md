Powertrain models {#vehicle_powertrain}
=======================================

\tableofcontents

The powertrain system is connected to the driver system (receiving throttle inputs) and to the driveline sub-system of the vehicle system (receiving the driveshaft angular speed and sending the driveshaft torque).

<img src="http://www.projectchrono.org/assets/manual/vehicle/powertrain_data_flow.png" width="600" />

Chrono::Vehicle is designed to permit linking to a third-party powertrain model. Interfacing to such an external powertrain model requires a thin interface class, derived from [ChPowertrain](@ref chrono::vehicle::ChPowertrain).

Chrono::Vehicle provides several templates for the powertrain system, descibed below.


## Shafts-based powertrain model {#vehicle_powertrain_shafts}

This powertrain model is based on various Chrono 1-D shaft elements ([ChShaft](@ref chrono::ChShaft)) and specialized shaft connection elements. Such elements are used to model:
- the engine ([ChShaftsThermalEngine](@ref chrono::ChShaftsThermalEngine)), specified through speed-torque curves
- the torque converter ([ChShaftsTorqueConverter](@ref chrono::ChShaftsTorqueConverter)), specified through curves for the capacity factor as function of the speed ratio and the torque ratio as function of the speed ratio
- the gear box (manual or automatic, with an arbitrary number of forward gears and a reverse gear)

The motor block is attached to the chassis through a special Chrono constraint ([ChShaftsBody](@ref chrono::ChShaftsBody)) thus permitting simulating motor torque effects on the chassis. 

See [ChShaftsPowertrain](@ref chrono::vehicle::ChShaftsPowertrain) and [ShaftsPowertrain](@ref chrono::vehicle::ShaftsPowertrain).

The image below shows the shafts-based powertrain model connected to a shafts-based wheeled vehicle model.

<img src="http://www.projectchrono.org/assets/manual/vehicle/shafts_powertrain.png" width="800" />

A sample JSON file with the specification of a shafts-based powertrain, illustrating the various template parameters, is provided below.
\include "../../data/vehicle/hmmwv/powertrain/HMMWV_ShaftsPowertrain.json"

The curves encoded in the above JSON file are shown below.

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/ShaftsPowertrain_engine_curves.png" width="500" />
<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/ShaftsPowertrain_TC_curves.png" width="500" />

## Simple powertrain model {#vehicle_powertrain_simple}

This model uses a trivial speed-torque dependency, has no torque converter and no transmission box.

See [ChSimplePowertrain](@ref chrono::vehicle::ChSimplePowertrain) and [SimplePowertrain](@ref chrono::vehicle::SimplePowertrain).

A sample JSON file with the specification of a simple powertrain, illustrating the various template parameters, is provided below.
\include "../../data/vehicle/hmmwv/powertrain/HMMWV_SimplePowertrain.json"


## Engine map powertrain model {#vehicle_powertrain_map}

This template for a simple powertrain model is based on speed-torque engine maps. The model has no torque converter and can have either a manual or an automatic transmission. It accepts a single reverse gear and any number of forward gears. In automatic mode, gear shifting is done based on specified ideal shift points.

See [ChSimpleMapPowertrain](@ref chrono::vehicle::ChSimpleMapPowertrain) and [SimpleMapPowertrain](@ref chrono::vehicle::SimpleMapPowertrain).

A sample JSON file with the specification of a map powertrain, illustrating the various template parameters, is provided below.
\include "../../data/vehicle/uaz/powertrain/UAZBUS_SimpleMapPowertrain.json"

The curves encoded in the above JSON file are shown below.

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/SimpleMapPowertrain_engine_curves.png" width="500" />
<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/SimpleMapPowertrain_shift_bands.png" width="500" />


## Simple CVT-like powertrain subsystem {#vehicle_powertrain_cvt}

This simple powertrain model template uses a hyperbolical speed-torque curve like a CVT gearbox, has no torque converter and no transmission box.

See [ChSimpleCVTPowertrain](@ref chrono::vehicle::ChSimpleCVTPowertrain) and [SimpleCVTPowertrain](@ref chrono::vehicle::SimpleCVTPowertrain).

A sample JSON file with the specification of a CVT-like powertrain, illustrating the various template parameters, is provided below.
\include "../../data/vehicle/hmmwv/powertrain/HMMWV_SimpleCVTPowertrain.json"
