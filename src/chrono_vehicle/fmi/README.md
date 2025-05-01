Chrono::Vehicle FMUs
====================

### Vehicle drivers

- FMU_PathFollower Driver
  An FMU encapsulating a combined path-follower lateral controller and a cruise control longitudinal controller.

### Powertrain

- FMU_Powertrain
  An FMU encapsulating an engine and transmission models, specified through JSON files.

### Wheeled vehicles

- FMU_WheeledVehicle
  An FMU encapsulating a Chrono::Vehicle vehicle model with no powertrain and no tires, specified through a set of JSON files.

- FMU_WheeledVehiclePtrain
  An FMU encapsulating a Chrono::Vehicle vehicle model with powertrain but no tires, specified through a set of JSON files.

- FMU_ForceElementTire
  An FMU encapsulating a Chrono::Vehicle handling tire model, specified through a JSON files.

