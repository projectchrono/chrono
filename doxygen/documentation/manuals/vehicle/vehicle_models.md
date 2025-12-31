Vehicle models {#vehicle_models}
================================

\tableofcontents

## Wheeled vehicle models {#vehicle_models_wheeled}

### HMMWV {#vehicle_models_hmmwv}

HMMWV stands for the High Mobility Multipurpose Wheeled Vehicle family. The Chrono example shows the M966 troup carrier. Like the real vehicle it has a double wishbone suspension on both axles and all wheels can be driven. It has a three-gear automatic gearbox with torque converter and a realistic engine subsystem. It takes a pitman-arm steering. The model can use all kinds of tires Chrono has to offer.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/HMMWV.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/HMMWV_suspension.png" width="600" />

### FEDA {#vehicle_models_feda}

The FED ALPHA is a concept vehicle developed by the engineering firm Ricardo in collaboration with the U.S. military.  

The Chrono::Vehicle FEDA is a 4WD model with double wishbone front and rear suspension, an anti-roll bar, and Pitman-arm steering mechanism.  Currently, only a Pacejka 2002 and rigid tire models are implemented.

### Sedan {#vehicle_models_sedan}

The Sedan examples shows a generic average passenger car. It uses a double wishbone suspension on the front and a multilink suspension on the rear axle. The steering system is of type rack-and-pinion.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/Sedan.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/Sedan_suspension.png" width="600" />

### Citybus {#vehicle_models_citybus}

As the name suggests the Citybus is a typical bus you can find in every city of the world. Like all heavy vehicles it uses leaf-sprung solid axles. The Chrono leafspring axles are based on a functional approach. For this reason no leafblades can be seen. The front wheels are steered by a rotary arm, in this combination it is also called toebar steering. This model also shows the use of double wheels.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/CityBus.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/CityBus_suspension.png" width="600" />

### FMTV {#vehicle_models_fmtv}

The Family of Medium Tactical Vehicles (FMTV) is a series of trucks currently manufactured by the Oshkosh Corporation.

The Chrono vehcle models library provides models for two variants of these vehicles:
- the LMTV (Light Medium Tactical Vehicle), a 2.5-ton, 4x4 truck 
- the MTV (Medium Tactical Vehicle), a 5-ton, 6x4 truck

Both vehicle models use leaf-spring rear axles and a leaf-spring with toe-bar steerable front axle and include a model of the torsional compliance between the cabin and cargo.  The MTV model has a balancer beam rear bogie.

### MAN {#vehicle_models_man}

The MAN (Maschinenfabrik Augsburg Nürnberg) Kat 1 truck family has been designed for tactical offroad use. All family members have coil-sprung solid axle systems guided by links. This design allow a very high wheel travel. The vehicle frame is box-shaped so the model can use a rigid chassis approach, since in the real vehicle there is nearly no frame torsion. All steering axles have a bellcrank/rotary arm mechanism. All wheels are driven.

Originally designed for german Bundeswehr, it can be found today in several armies in the world, for example in the US army as M1001 prime mover or as M1002 recovery vehicle.

The 5t (load capacity) truck is the smallest family member.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_5t.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_5t_suspension.png" width="600" />

The 7t truck is the one with the most different configurations for carrying special equipment or as tipper. In Chrono it has just a load bed.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_7t.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_7t_suspension.png" width="600" />

The 10t truck was the very first model to go in service in the late 70s. Most of them are used as transport trucks, some are equipped with a load crane. 

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_10t.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/MAN_10t_suspension.png" width="600" />

### UAZ {#vehicle_models_uaz}

UAZ stands for Ulyanovski Avtomobilny Zavod. The UAZ 452 Bus/Van/Lorry has been popular in Russia since 1965. It has an all-wheel-drive and two leaf-spring axles. The front axle has a toebar steering mechanism.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/UAZBUS.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/UAZBUS_suspension.png" width="600" />

The UAZ 469 jeep-like vehicle has been used since 1965 in the Soviet Army. It shares many components (axle, engine) with the UAZ 452. The steering system is also of toebar/rotary arm type, but in a slightly different configuration.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/UAZ469.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/UAZ469_suspension.png" width="600" />

### Gator {#vehicle_models_gator}

The Gator is a model of a John Deere electric utility vehicle instrumented for autonomous driving.  The Gator model uses a single-wishbone suspension in the front and a rigid axle in the rear.  It has a rack-and-pinion steering mechanism and brakes only on the rear wheels.  The electric motor is connected through a differential to the rear wheels.

### KRAZ {#vehicle_models_kraz}

This model, based on the Ukrainian Kraz 64431, is a semi-trailer truck. The tractor has a toebar leaf-spring front suspension, steered through a rotary arm mechanism and two leaf-spring rear suspensions. The tractor is in a 6x4 configuration with the two rear axles driven. The trailer, connected to the tractor through a hitch, has three axles all of leaf-spring type.

### RC-car {#vehicle_models_rccar}

This is a model of a small remote-controlled 4WD wheeled vehicle.

### Generic wheeld vehicle {#vehicle_models_generic}

The generic wheeld vehicle model is a sandbox for testing various templates and settings. It does not represent any particular vehicle and not all concrete subsystems provided for this model are necessarily consistent with each other.

## Tracked vehicle models {#vehicle_models_tracked}

### M113 {#vehicle_models_m113}

The M113 is a light tank with aluminium alloy hull and a running gear of Christie type. The first M113 went in service 1960. It is used in many armies of the world and a lot of configurations have been derived from the basic vehicle. The Chrono M113 model shows a troup carrier (2+11 occupants).

The running gear consists of ten road wheels, two sprockets and two idler wheels. The model can be configured with a single pin track or a double pin track. A third alternative - a rubber band track - is under development. The track tension can be set by means of the special tensioning suspension of the idler wheels. The road wheels are suspended by a bogie/torsion bar system. The drivetrain is modelled as three gear automatic gear box with torque converter in combination of a combustion engine.

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/M113.png" width="600" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/models/M113_suspension.png" width="600" />

### Marder {#vehicle_models_marder}

The Marder ("marten" in German) is a tracked infantry fighting vehicle used by the German Bundeswehr since 1969. It has a running gear with 12 road wheels, sprocket, idler and 3 support rollers. The first two and the last two road wheels on every side are damped by telescopic dampers. It is driven by a 444 kW Diesel engine, torque converter with lockup and 4 gear automatic gearbox. It carries up to nine soldiers (commander, gunner, driver and six infantrymen). 

Our model is based only on public data available online and information found in literature. Although the original vehicle employs double-pin tracks, the current Chrono model only implements a single-pin track.
