// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definitions of wheeled vehicle models specified through JSON files
//
// =============================================================================

#pragma once

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Specification of a vehicle model from JSON files
// Available models:
//    HMMWV       - High Mobility Multipurpose Wheeled Vehicle
//    Sedan       - Generic sedan vehicle
//    Audi        - Audia A4
//    VW microbus - VW T2 microbus
//    UAZ         - UAZ minibus
//    G500        - Mercedes-Benz G500
//    CityBus     - passenger bus
//    MAN         - MAN 10t truck
//    MTV         - MTV truck
//    ACV         - articulated chassis vehicle (skid steer)

class WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string TireJSON(unsigned int axle) const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual double CameraDistance() const = 0;

    static std::vector<std::pair<std::shared_ptr<WheeledVehicleJSON>, std::string>> List();
};

class HMMWV_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual std::string VehicleJSON() const override {
        return "hmmwv/vehicle/HMMWV_Vehicle.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_replica.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_mapShock.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_bushings.json";
        ////return "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
    }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "hmmwv/tire/HMMWV_RigidTire.json";
        ////return "hmmwv/tire/HMMWV_FialaTire.json";
        return "hmmwv/tire/HMMWV_TMeasyTire.json";
        ////return "hmmwv/tire/HMMWV_TMsimpleTire.json";
        ////return "hmmwv/tire/HMMWV_Pac89Tire.json";
        ////return "hmmwv/tire/HMMWV_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override {
        return "hmmwv/powertrain/HMMWV_EngineShafts.json";
        ////return "hmmwv/powertrain/HMMWV_EngineSimpleMap.json";
        ////return "hmmwv/powertrain/HMMWV_EngineSimple.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json";
        ////return "hmmwv/powertrain/HMMWV_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class Sedan_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "Sedan"; }
    virtual std::string VehicleJSON() const override { return "sedan/vehicle/Sedan_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "sedan/tire/Sedan_RigidTire.json";
        ////return "sedan/tire/Sedan_TMeasyTire.json";
        return "sedan/tire/Sedan_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override {
        ////return "sedan/powertrain/Sedan_EngineSimpleMap.json";
        return "sedan/powertrain/Sedan_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        ////return "sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json";
        return "sedan/powertrain/Sedan_ManualTransmissionShafts.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class Audi_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "Audi"; }
    virtual std::string VehicleJSON() const override { return "audi/json/audi_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "audi/json/audi_TMeasyTire.json";
        return "audi/json/audi_Pac02Tire.json";
        ////return "audi/json/audi_RigidTire.json.json";
    }
    virtual std::string EngineJSON() const override { return "audi/json/audi_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "audi/json/audi_AutomaticTransmissionSimpleMap.json";
        ////return "audi/json/audi_ManualTransmissionShafts.json";
    }
    virtual double CameraDistance() const override { return 6.0; }
};

class Polaris_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "Polaris"; }
    virtual std::string VehicleJSON() const override { return "Polaris/Polaris.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        return "Polaris/Polaris_TMeasyTire.json";
        ////return "Polaris/Polaris_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "Polaris/Polaris_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 7.0; }
};

class UAZ_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "UAZ"; }
    virtual std::string VehicleJSON() const override {
        ////return "uaz/vehicle/UAZBUS_Vehicle.json";
        return "uaz/vehicle/UAZ469_Vehicle.json";
        ////return "uaz/vehicle/UAZBUS_VehicleT.json";
        ////return "uaz/vehicle/UAZBUS_SAEVehicle.json";
    }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "uaz/tire/UAZBUS_TMeasyTireFront.json";
        return "uaz/tire/UAZBUS_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "uaz/powertrain/UAZBUS_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "uaz/powertrain/UAZBUS_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 8.0; }
};

class G500_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "G500"; }
    virtual std::string VehicleJSON() const override { return "gclass/vehicle/G500_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override { return "gclass/tire/G500_TMeasyTire.json"; }
    virtual std::string EngineJSON() const override { return "gclass/powertrain/G500_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "gclass/powertrain/G500_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 10.0; }
};

class VW_Microbus_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "VW_Microbus"; }
    virtual std::string VehicleJSON() const override { return "VW_microbus/json/van_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "VW_microbus/json/van_TMsimpleTireFull.json";
        ////return "VW_microbus/json/van_TMsimpleTire.json";
        ////return "VW_microbus/json/van_TMeasyTireFull.json";
        ////return "VW_microbus/json/van_TMeasyTire.json";
        return "VW_microbus/json/van_Pac02Tire_extTIR.json";
        ////return "VW_microbus/json/van_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "VW_microbus/json/van_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "VW_microbus/json/van_AutomaticTransmissionSimpleMap.json";
    }
    virtual double CameraDistance() const override { return 7.0; }
};

class CityBus_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "CityBus"; }
    virtual std::string VehicleJSON() const override { return "citybus/vehicle/CityBus_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        ////return "citybus/tire/CityBus_RigidTire.json";
        ////return "citybus/tire/CityBus_TMeasyTire.json";
        return "citybus/tire/CityBus_Pac02Tire.json";
    }
    virtual std::string EngineJSON() const override { return "citybus/powertrain/CityBus_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "citybus/powertrain/CityBus_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 16.0; }
};

class Duro_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "Duro"; }
    virtual std::string VehicleJSON() const override { return "duro/vehicle/duro_vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override { return "duro/tires/duro_tmeasy_tire.json"; }
    virtual std::string EngineJSON() const override { return "duro/powertrain/duro_engine_simple_map.json"; }
    virtual std::string TransmissionJSON() const override {
        return "duro/powertrain/duro_automatic_transmission_simple_map.json";
    }

    virtual double CameraDistance() const override { return 10.0; }
};

class MAN_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "MAN"; }
    virtual std::string VehicleJSON() const override {
        ////return "MAN_Kat1/vehicle/MAN_5t_Vehicle_4WD.json";
        ////return "MAN_Kat1/vehicle/MAN_7t_Vehicle_6WD.json";
        return "MAN_Kat1/vehicle/MAN_10t_Vehicle_8WD.json";
    }
    virtual std::string TireJSON(unsigned int axle) const override { return "MAN_Kat1/tire/MAN_5t_TMeasyTire.json"; }
    virtual std::string EngineJSON() const override { return "MAN_Kat1/powertrain/MAN_7t_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "MAN_Kat1/powertrain/MAN_7t_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 15.0; }
};

class MTV_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "MTV"; }
    virtual std::string VehicleJSON() const override { return "mtv/vehicle/MTV_Vehicle_WalkingBeam.json"; }
    virtual std::string TireJSON(unsigned int axle) const override { return "mtv/tire/FMTV_TMeasyTire.json"; }
    virtual std::string EngineJSON() const override { return "mtv/powertrain/FMTV_EngineShafts.json"; }
    virtual std::string TransmissionJSON() const override {
        return "mtv/powertrain/FMTV_AutomaticTransmissionShafts.json";
    }

    virtual double CameraDistance() const override { return 12.0; }
};

class Gator_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "Gator"; }
    virtual std::string VehicleJSON() const override { return "gator/json/Gator_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override {
        return (axle == 0) ? "gator/json/Gator_TMeasyTireFront.json" : "gator/json/Gator_TMeasyTireRear.json";
    }
    virtual std::string EngineJSON() const override { return "gator/json/Gator_EngineSimple.json"; }
    virtual std::string TransmissionJSON() const override {
        return "gator/json/Gator_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 8.0; }
};

class ACV_Model : public WheeledVehicleJSON {
  public:
    virtual std::string ModelName() const override { return "ACV"; }
    virtual std::string VehicleJSON() const override { return "articulated_chassis/ACV_Vehicle.json"; }
    virtual std::string TireJSON(unsigned int axle) const override { return "articulated_chassis/ACV_RigidTire.json"; }
    virtual std::string EngineJSON() const override { return "articulated_chassis/ACV_EngineSimpleMap.json"; }
    virtual std::string TransmissionJSON() const override {
        return "articulated_chassis/ACV_AutomaticTransmissionSimpleMap.json";
    }

    virtual double CameraDistance() const override { return 8.0; }
};

// =============================================================================
// Specification of a trailer model from JSON files
// Available models:
//    Ultra_Tow 40in x 48 in

class Trailer_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string TrailerJSON() const = 0;
    virtual std::string TireJSON() const = 0;
};

class UT_Model : public Trailer_Model {
  public:
    virtual std::string ModelName() const override { return "Ultra-Tow"; }
    virtual std::string TrailerJSON() const override { return "ultra_tow/UT_Trailer.json"; }
    virtual std::string TireJSON() const override {
        ////return "ultra_tow/UT_RigidTire.json";
        return "ultra_tow/UT_TMeasyTire.json";
    }
};

// =============================================================================

std::vector<std::pair<std::shared_ptr<WheeledVehicleJSON>, std::string>> WheeledVehicleJSON::List() {
    std::vector<std::pair<std::shared_ptr<WheeledVehicleJSON>, std::string>> models = {
        {chrono_types::make_shared<HMMWV_Model>(), "HMMWV"},
        {chrono_types::make_shared<Audi_Model>(), "Audi"},
        {chrono_types::make_shared<Polaris_Model>(), "Polaris"},
        {chrono_types::make_shared<VW_Microbus_Model>(), "VW bus"},
        {chrono_types::make_shared<UAZ_Model>(), "UAZ"},
        {chrono_types::make_shared<G500_Model>(), "Mercedes G500"},
        {chrono_types::make_shared<Sedan_Model>(), "Sedan"},
        {chrono_types::make_shared<CityBus_Model>(), "City bus"},
        {chrono_types::make_shared<MAN_Model>(), "MAN"},
        {chrono_types::make_shared<MTV_Model>(), "MTV"},
        {chrono_types::make_shared<Duro_Model>(), "Duro"},
        {chrono_types::make_shared<Gator_Model>(), "Gator"},
        {chrono_types::make_shared<ACV_Model>(), "Skid steer"}};

    return models;
}