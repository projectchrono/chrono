// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Wrappers around existing models in the Chrono::Vehicle model library
//
// =============================================================================

#pragma once

#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono_models/vehicle/citybus/CityBus.h"
#include "chrono_models/vehicle/duro/Duro.h"
#include "chrono_models/vehicle/feda/FEDA.h"
#include "chrono_models/vehicle/gclass/G500.h"
#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_models/vehicle/kraz/Kraz.h"
#include "chrono_models/vehicle/man/MAN_5t.h"
#include "chrono_models/vehicle/man/MAN_7t.h"
#include "chrono_models/vehicle/man/MAN_10t.h"
#include "chrono_models/vehicle/mtv/LMTV.h"
#include "chrono_models/vehicle/mtv/MTV.h"
#include "chrono_models/vehicle/mrole/mrole.h"
#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/uaz/UAZBUS.h"
#include "chrono_models/vehicle/uaz/UAZBUS_SAE.h"
#include "chrono_models/vehicle/unimog/U401.h"

using namespace chrono;
using namespace chrono::vehicle;

using namespace chrono::vehicle::artcar;
using namespace chrono::vehicle::citybus;
using namespace chrono::vehicle::duro;
using namespace chrono::vehicle::feda;
using namespace chrono::vehicle::fmtv;
using namespace chrono::vehicle::gclass;
using namespace chrono::vehicle::gator;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::vehicle::kraz;
using namespace chrono::vehicle::man;
using namespace chrono::vehicle::mrole;
using namespace chrono::vehicle::sedan;
using namespace chrono::vehicle::uaz;
using namespace chrono::vehicle::unimog;

// =============================================================================
// Specification of a vehicle model from the Chrono::Vehicle model library

class WheeledVehicleModel {
  public:
    virtual std::string ModelName() const = 0;
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) = 0;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) = 0;
    virtual ChWheeledVehicle& GetVehicle() = 0;
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) = 0;
    virtual void Advance(double step) = 0;
    virtual ChVector3d TrackPoint() const = 0;
    virtual double CameraDistance() const = 0;
    virtual double CameraHeight() const = 0;

    static std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> List();
};

class ARTcar_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "ARTCAR"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        car->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 0.2); }
    virtual double CameraDistance() const override { return 2.0; }
    virtual double CameraHeight() const override { return 0.3; }

  private:
    void Construct();
    ARTcar* car;
};

class Citybus_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "CITYBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return bus->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        bus->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { bus->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 16.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    CityBus* bus;
};

class Duro_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "DURO"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return duro->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        duro->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { duro->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    Duro* duro;
};

class FEDA_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "FEDA"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return feda->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        feda->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { feda->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    FEDA* feda;
};

class G500_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "G500"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return gclass->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        gclass->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { gclass->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    G500* gclass;
};

class Gator_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "GATOR"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return gator->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        gator->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { gator->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 7.0; }
    virtual double CameraHeight() const override { return 0.1; }

  private:
    void Construct();
    Gator* gator;
};

class HMMWV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return hmmwv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        hmmwv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { hmmwv->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    HMMWV_Full* hmmwv;
};

class HMMWV9_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "HMMWV9"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return hmmwv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        hmmwv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { hmmwv->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    HMMWV_Reduced* hmmwv;
};

class KRAZ_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "KRAZ"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return kraz->GetTractor(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        kraz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { kraz->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 25.0; }
    virtual double CameraHeight() const override { return 1.5; }

  private:
    void Construct();
    Kraz* kraz;
};

class LMTV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "KRAZ"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return lmtv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        lmtv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { lmtv->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    LMTV* lmtv;
};

class MTV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "KRAZ"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return mtv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        mtv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { mtv->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    MTV* mtv;
};

class MAN5_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN5t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    MAN_5t* man;
};

class MAN7_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN7t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    MAN_7t* man;
};

class MAN10_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN10t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    MAN_10t* man;
};

class MROLE_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MROLE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return mrole->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        mrole->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { mrole->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    mrole_Full* mrole;
};

class Sedan_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "SEDAN"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return sedan->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        sedan->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { sedan->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct();
    Sedan* sedan;
};

class UAZBUS_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UAZBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return uaz->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        uaz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { uaz->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    UAZBUS* uaz;
};

class UAZBUSSAE_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UAZBUSSAE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return uaz->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        uaz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { uaz->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    UAZBUS_SAE* uaz;
};

class U401_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UNIMOG"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos) override;
    virtual ChWheeledVehicle& GetVehicle() override { return u401->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        u401->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { u401->Advance(step); }
    virtual ChVector3d TrackPoint() const { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct();
    U401* u401;
};

// =============================================================================

void ARTcar_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    car = new ARTcar(system);
    car->SetInitPosition(init_pos);
    Construct();
}

void ARTcar_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    car = new ARTcar();
    car->SetContactMethod(contact_method);
    car->SetInitPosition(init_pos);
    Construct();
}

void ARTcar_Model::Construct() {
    car->SetChassisCollisionType(CollisionType::NONE);
    car->SetChassisFixed(false);
    car->SetTireType(TireModelType::TMEASY);
    car->Initialize();

    car->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::NONE);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Citybus_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    bus = new CityBus(system);
    bus->SetInitPosition(init_pos);
    Construct();
}

void Citybus_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    bus = new CityBus();
    bus->SetContactMethod(contact_method);
    bus->SetInitPosition(init_pos);
    Construct();
}

void Citybus_Model::Construct() {
    bus->SetChassisFixed(false);
    bus->SetTireType(TireModelType::PAC02);
    bus->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    bus->Initialize();

    bus->SetChassisVisualizationType(VisualizationType::MESH);
    bus->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    bus->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    bus->SetWheelVisualizationType(VisualizationType::MESH);
    bus->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Duro_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    duro = new Duro(system);
    duro->SetInitPosition(init_pos);
    Construct();
}

void Duro_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    duro = new Duro();
    duro->SetContactMethod(contact_method);
    duro->SetInitPosition(init_pos);
    Construct();
}

void Duro_Model::Construct() {
    duro->SetChassisFixed(false);
    duro->SetEngineType(EngineModelType::SHAFTS);
    duro->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    duro->SetTireType(TireModelType::TMEASY);
    duro->SetBrakeType(BrakeType::SHAFTS);
    duro->SetInitFwdVel(0.0);
    duro->Initialize();

    duro->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    duro->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    duro->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    duro->SetWheelVisualizationType(VisualizationType::NONE);
    duro->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void FEDA_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    feda = new FEDA(system);
    feda->SetInitPosition(init_pos);
    Construct();
}

void FEDA_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    feda = new FEDA();
    feda->SetContactMethod(contact_method);
    feda->SetInitPosition(init_pos);
    Construct();
}

void FEDA_Model::Construct() {
    feda->SetChassisFixed(false);
    feda->SetEngineType(EngineModelType::SIMPLE_MAP);
    feda->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    feda->SetTireType(TireModelType::PAC02);
    feda->SetBrakeType(BrakeType::SHAFTS);
    feda->SetAerodynamicDrag(0.6, 3.8, 1.2041);
    feda->Initialize();

    feda->SetChassisVisualizationType(VisualizationType::MESH);
    feda->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    feda->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    feda->SetWheelVisualizationType(VisualizationType::MESH);
    feda->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void G500_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    gclass = new G500(system);
    gclass->SetInitPosition(init_pos);
    Construct();
}

void G500_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    gclass = new G500();
    gclass->SetContactMethod(contact_method);
    gclass->SetInitPosition(init_pos);
    Construct();
}

void G500_Model::Construct() {
    gclass->SetChassisCollisionType(CollisionType::NONE);
    gclass->SetChassisFixed(false);
    gclass->SetTireType(TireModelType::TMEASY);
    gclass->SetAerodynamicDrag(0.5, 5.0, 1.2);
    gclass->Initialize();

    gclass->SetChassisVisualizationType(VisualizationType::MESH);
    gclass->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    gclass->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    gclass->SetWheelVisualizationType(VisualizationType::MESH);
    gclass->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Gator_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    gator = new Gator(system);
    gator->SetInitPosition(init_pos);
    Construct();
}

void Gator_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    gator = new Gator();
    gator->SetContactMethod(contact_method);
    gator->SetInitPosition(init_pos);
    Construct();
}

void Gator_Model::Construct() {
    gator->SetChassisCollisionType(CollisionType::NONE);
    gator->SetChassisFixed(false);
    gator->SetTireType(TireModelType::TMEASY);
    gator->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    gator->SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator->Initialize();

    gator->SetChassisVisualizationType(VisualizationType::MESH);
    gator->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    gator->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    gator->SetWheelVisualizationType(VisualizationType::MESH);
    gator->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    hmmwv = new HMMWV_Full(system);
    hmmwv->SetInitPosition(init_pos);
    Construct();
}

void HMMWV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    hmmwv = new HMMWV_Full();
    hmmwv->SetContactMethod(contact_method);
    hmmwv->SetInitPosition(init_pos);
    Construct();
}

void HMMWV_Model::Construct() {
    hmmwv->SetChassisCollisionType(CollisionType::NONE);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetEngineType(EngineModelType::SHAFTS);
    hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->UseTierodBodies(true);
    hmmwv->SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    hmmwv->SetBrakeType(BrakeType::SHAFTS);
    hmmwv->SetTireType(TireModelType::PAC02);
    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV9_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    hmmwv = new HMMWV_Reduced(system);
    hmmwv->SetInitPosition(init_pos);
    Construct();
}

void HMMWV9_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    hmmwv = new HMMWV_Reduced();
    hmmwv->SetContactMethod(contact_method);
    hmmwv->SetInitPosition(init_pos);
    Construct();
}

void HMMWV9_Model::Construct() {
    hmmwv->SetChassisFixed(false);
    hmmwv->SetChassisCollisionType(CollisionType::NONE);
    hmmwv->SetEngineType(EngineModelType::SHAFTS);
    hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->SetTireType(TireModelType::PAC89);
    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void KRAZ_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    kraz = new Kraz(system);
    kraz->SetInitPosition(init_pos);
    Construct();
}

void KRAZ_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    kraz = new Kraz();
    kraz->SetContactMethod(contact_method);
    kraz->SetInitPosition(init_pos);
    Construct();
}

void KRAZ_Model::Construct() {
    kraz->SetChassisFixed(false);
    kraz->SetInitFwdVel(0.0);
    kraz->Initialize();

    kraz->SetChassisVisualizationType(VisualizationType::MESH, VisualizationType::PRIMITIVES);
    kraz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    kraz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
    kraz->SetWheelVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
    kraz->SetTireVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void LMTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    lmtv = new LMTV(system);
    lmtv->SetInitPosition(init_pos);
    Construct();
}

void LMTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    lmtv = new LMTV();
    lmtv->SetContactMethod(contact_method);
    lmtv->SetInitPosition(init_pos);
    Construct();
}

void LMTV_Model::Construct() {
    lmtv->SetChassisFixed(false);
    lmtv->SetTireType(TireModelType::TMEASY);
    lmtv->Initialize();

    lmtv->SetChassisVisualizationType(VisualizationType::MESH);
    lmtv->SetChassisRearVisualizationType(VisualizationType::MESH);
    lmtv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    lmtv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    lmtv->SetWheelVisualizationType(VisualizationType::MESH);
    lmtv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    mtv = new MTV(system);
    mtv->SetInitPosition(init_pos);
    Construct();
}

void MTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    mtv = new MTV();
    mtv->SetContactMethod(contact_method);
    mtv->SetInitPosition(init_pos);
    Construct();
}

void MTV_Model::Construct() {
    mtv->SetChassisFixed(false);
    mtv->UseWalkingBeamRearSuspension(false);
    mtv->SetTireType(TireModelType::TMEASY);
    mtv->Initialize();

    mtv->SetChassisVisualizationType(VisualizationType::MESH);
    mtv->SetChassisRearVisualizationType(VisualizationType::MESH);
    mtv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mtv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mtv->SetWheelVisualizationType(VisualizationType::MESH);
    mtv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN5_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    man = new MAN_5t(system);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN5_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    man = new MAN_5t();
    man->SetContactMethod(contact_method);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN5_Model::Construct() {
    man->SetChassisCollisionType(CollisionType::NONE);
    man->SetChassisFixed(false);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(VisualizationType::MESH);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN7_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    man = new MAN_7t(system);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN7_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    man = new MAN_7t();
    man->SetContactMethod(contact_method);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN7_Model::Construct() {
    man->SetChassisCollisionType(CollisionType::NONE);
    man->SetChassisFixed(false);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(VisualizationType::MESH);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN10_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    man = new MAN_10t(system);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN10_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    man = new MAN_10t();
    man->SetContactMethod(contact_method);
    man->SetInitPosition(init_pos);
    Construct();
}

void MAN10_Model::Construct() {
    man->SetChassisCollisionType(CollisionType::NONE);
    man->SetChassisFixed(false);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(VisualizationType::MESH);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MROLE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    mrole = new mrole_Full(system);
    mrole->SetInitPosition(init_pos);
    Construct();
}

void MROLE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    mrole = new mrole_Full();
    mrole->SetContactMethod(contact_method);
    mrole->SetInitPosition(init_pos);
    Construct();
}

void MROLE_Model::Construct() {
    mrole->SetChassisCollisionType(CollisionType::NONE);
    mrole->SetChassisFixed(false);
    mrole->SetEngineType(EngineModelType::SHAFTS);
    mrole->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    mrole->SetDriveType(DrivelineTypeWV::AWD6);
    mrole->SetBrakeType(BrakeType::SHAFTS);
    mrole->SetTireType(TireModelType::TMEASY);
    mrole->SelectRoadOperation();
    mrole->Initialize();

    mrole->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    mrole->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mrole->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mrole->SetWheelVisualizationType(VisualizationType::MESH);
    mrole->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Sedan_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    sedan = new Sedan(system);
    sedan->SetInitPosition(init_pos);
    Construct();
}

void Sedan_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    sedan = new Sedan();
    sedan->SetContactMethod(contact_method);
    sedan->SetInitPosition(init_pos);
    Construct();
}

void Sedan_Model::Construct() {
    sedan->SetChassisCollisionType(CollisionType::NONE);
    sedan->SetChassisFixed(false);
    sedan->SetTireType(TireModelType::PAC02);
    sedan->SetBrakeType(BrakeType::SHAFTS);
    sedan->Initialize();

    sedan->SetChassisVisualizationType(VisualizationType::MESH);
    sedan->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    sedan->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    sedan->SetWheelVisualizationType(VisualizationType::MESH);
    sedan->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUS_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    uaz = new UAZBUS(system);
    uaz->SetInitPosition(init_pos);
    Construct();
}

void UAZBUS_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    uaz = new UAZBUS();
    uaz->SetContactMethod(contact_method);
    uaz->SetInitPosition(init_pos);
    Construct();
}

void UAZBUS_Model::Construct() {
    uaz->SetChassisFixed(false);
    uaz->SetTireType(TireModelType::PAC02);
    uaz->SetInitFwdVel(0.0);
    uaz->Initialize();

    uaz->SetChassisVisualizationType(VisualizationType::MESH);
    uaz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetWheelVisualizationType(VisualizationType::MESH);
    uaz->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUSSAE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    uaz = new UAZBUS_SAE(system);
    uaz->SetInitPosition(init_pos);
    Construct();
}

void UAZBUSSAE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    uaz = new UAZBUS_SAE();
    uaz->SetContactMethod(contact_method);
    uaz->SetInitPosition(init_pos);
    Construct();
}

void UAZBUSSAE_Model::Construct() {
    uaz->SetChassisFixed(false);
    uaz->SetTireType(TireModelType::TMEASY);
    uaz->SetInitFwdVel(0.0);
    uaz->Initialize();

    uaz->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetWheelVisualizationType(VisualizationType::MESH);
    uaz->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void U401_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos) {
    u401 = new U401(system);
    u401->SetInitPosition(init_pos);
    Construct();
}

void U401_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos) {
    u401 = new U401();
    u401->SetContactMethod(contact_method);
    u401->SetInitPosition(init_pos);
    Construct();
}

void U401_Model::Construct() {
    u401->SetChassisFixed(false);
    u401->SetTireType(TireModelType::TMEASY);
    u401->SetInitFwdVel(0.0);
    u401->Initialize();

    u401->SetChassisVisualizationType(VisualizationType::MESH);
    u401->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    u401->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    u401->SetWheelVisualizationType(VisualizationType::MESH);
    u401->SetTireVisualizationType(VisualizationType::MESH);
}

// =============================================================================

std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> WheeledVehicleModel::List() {
    std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> models = {
        {chrono_types::make_shared<ARTcar_Model>(), "ARTCAR"},         //
        {chrono_types::make_shared<Citybus_Model>(), "CITYBUS"},       //
        {chrono_types::make_shared<Duro_Model>(), "DURO"},             //
        {chrono_types::make_shared<FEDA_Model>(), "FEDA"},             //
        {chrono_types::make_shared<G500_Model>(), "G500"},             //
        {chrono_types::make_shared<Gator_Model>(), "GATOR"},           //
        {chrono_types::make_shared<HMMWV_Model>(), "HMMWV"},           //
        {chrono_types::make_shared<HMMWV9_Model>(), "HMMWV9"},         //
        {chrono_types::make_shared<KRAZ_Model>(), "KRAZ"},             //
        {chrono_types::make_shared<LMTV_Model>(), "LMTV"},             //
        {chrono_types::make_shared<MAN5_Model>(), "MAN5t"},            //
        {chrono_types::make_shared<MAN7_Model>(), "MAN7t"},            //
        {chrono_types::make_shared<MAN10_Model>(), "MAN10t"},          //
        {chrono_types::make_shared<MTV_Model>(), "MTV"},               //
        {chrono_types::make_shared<MROLE_Model>(), "MROLE"},           //
        {chrono_types::make_shared<Sedan_Model>(), "SEDAN"},           //
        {chrono_types::make_shared<UAZBUS_Model>(), "UAZBUS"},         //
        {chrono_types::make_shared<UAZBUSSAE_Model>(), "UAZBUS_SAE"},  //
        {chrono_types::make_shared<U401_Model>(), "UNIMOG"}            //
    };

    return models;
}
