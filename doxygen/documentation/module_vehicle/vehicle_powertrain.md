Powertrain models {#vehicle_powertrain}
=======================================

The powertrain system is connected to the driver system (receiving throttle inputs) and to the driveline sub-system of the vehicle system (receiving the driveshaft angular speed and sending the driveshaft torque).

<img src="http://www.projectchrono.org/assets/manual/vehicle/powertrain_data_flow.png" width="600" />

Chrono::Vehicle is designed to permit linking to a third-party powertrain model. Interfacing to such an external powertrain model requires a thin interface class, derived from [ChPowertrain](@ref chrono::vehicle::ChPowertrain).

Chrono::Vehicle provides several templates for the powertrain system, descibed below.


## Shafts-based powertrain model {#vehicle_powertrain_shafts}

This powertrain model leverages various 1-D modeling elements in Chrono, in particular ...

The image below shows the shafts-based powertrain model connected to a shafts-based wheeled vehicle model.

<img src="http://www.projectchrono.org/assets/manual/vehicle/shafts_powertrain.png" width="800" />


## Simple powertrain model {#vehicle_powertrain_simple}


## Engine map powertrain model {#vehicle_powertrain_map}


## Simple CVT-like powertrain subsystem {#vehicle_powertrain_cvt}
