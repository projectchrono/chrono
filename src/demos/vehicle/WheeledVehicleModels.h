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

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/artcar/ARTcar.h"
#include "chrono_models/vehicle/jeep/Cherokee.h"
#include "chrono_models/vehicle/bmw/BMW_E90.h"
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
using namespace chrono::vehicle::jeep;
using namespace chrono::vehicle::bmw;
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
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis = true) = 0;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis = true) = 0;
    virtual ChWheeledVehicle& GetVehicle() = 0;
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) = 0;
    virtual void Advance(double step) = 0;
    virtual ChVector3d TrackPoint() const = 0;
    virtual double CameraDistance() const = 0;
    virtual double CameraHeight() const = 0;

    void SetChassisCollisionType(CollisionType type) { chassis_collision_type = type; }
    void SetCollisionSystemType(ChCollisionSystem::Type type) { collision_system_type = type; }

    static std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> List();

  protected:
    WheeledVehicleModel()
        : collision_system_type(ChCollisionSystem::Type::BULLET), chassis_collision_type(CollisionType::NONE) {}

    ChCollisionSystem::Type collision_system_type;
    CollisionType chassis_collision_type;
};

class ARTcar_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "ARTCAR"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        car->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 0.2); }
    virtual double CameraDistance() const override { return 2.0; }
    virtual double CameraHeight() const override { return 0.3; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    ARTcar* car;
};

class Citybus_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "CITYBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return bus->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        bus->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { bus->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 16.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    CityBus* bus;
};

class Duro_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "DURO"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return duro->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        duro->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { duro->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Duro* duro;
};

class FEDA_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "FEDA"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return feda->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        feda->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { feda->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    FEDA* feda;
};

class G500_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "G500"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return gclass->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        gclass->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { gclass->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    G500* gclass;
};

class Gator_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "GATOR"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return gator->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        gator->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { gator->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 7.0; }
    virtual double CameraHeight() const override { return 0.1; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Gator* gator;
};

class HMMWV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return hmmwv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        hmmwv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { hmmwv->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    HMMWV_Full* hmmwv;
};

class HMMWV9_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "HMMWV9"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return hmmwv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        hmmwv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { hmmwv->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    HMMWV_Reduced* hmmwv;
};

class KRAZ_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "KRAZ"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return kraz->GetTractor(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        kraz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { kraz->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 25.0; }
    virtual double CameraHeight() const override { return 1.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Kraz* kraz;
};

class LMTV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "LMTV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return lmtv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        lmtv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { lmtv->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    LMTV* lmtv;
};

class MTV_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MTV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return mtv->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        mtv->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { mtv->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MTV* mtv;
};

class MAN5_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN5t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_5t* man;
};

class MAN7_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN7t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_7t* man;
};

class MAN10_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MAN10t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return man->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        man->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { man->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_10t* man;
};

class MROLE_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "MROLE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return mrole->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        mrole->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { mrole->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    mrole_Full* mrole;
};

class Sedan_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "SEDAN"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return sedan->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        sedan->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { sedan->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Sedan* sedan;
};

class BMW_E90_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "BMW_E90"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return bmw->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        bmw->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { bmw->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    BMW_E90* bmw;
};

class Cherokee_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "Cherokee"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return cherokee->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        cherokee->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { cherokee->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Cherokee* cherokee;
};

class UAZBUS_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UAZBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return uaz->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        uaz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { uaz->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    UAZBUS* uaz;
};

class UAZBUSSAE_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UAZBUSSAE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return uaz->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        uaz->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { uaz->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    UAZBUS_SAE* uaz;
};

class U401_Model : public WheeledVehicleModel {
  public:
    virtual std::string ModelName() const override { return "UNIMOG"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return u401->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override {
        u401->Synchronize(time, driver_inputs, terrain);
    }
    virtual void Advance(double step) override { u401->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    U401* u401;
};

// =============================================================================

void ARTcar_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new ARTcar(system);
    car->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void ARTcar_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new ARTcar();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    car->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void ARTcar_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetTireType(TireModelType::TMEASY);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::NONE);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Citybus_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    bus = new CityBus(system);
    bus->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Citybus_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    bus = new CityBus();
    bus->SetCollisionSystemType(collision_system_type);
    bus->SetContactMethod(contact_method);
    bus->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Citybus_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    bus->SetChassisCollisionType(chassis_collision_type);
    bus->SetChassisFixed(false);
    bus->SetTireType(TireModelType::PAC02);
    bus->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    bus->Initialize();

    bus->SetChassisVisualizationType(chassis_vis);
    bus->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    bus->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    bus->SetWheelVisualizationType(VisualizationType::MESH);
    bus->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Cherokee_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    cherokee = new Cherokee(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Cherokee_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    cherokee = new Cherokee();
    cherokee->SetCollisionSystemType(collision_system_type);
    cherokee->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Cherokee_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    cherokee->SetChassisCollisionType(chassis_collision_type);
    cherokee->SetChassisFixed(false);
    cherokee->SetInitPosition(init_pos);
    cherokee->SetTireType(TireModelType::TMEASY);
    cherokee->SetBrakeType(BrakeType::SHAFTS);
    cherokee->Initialize();

    cherokee->SetChassisVisualizationType(chassis_vis);
    cherokee->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    cherokee->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    cherokee->SetWheelVisualizationType(VisualizationType::MESH);
    cherokee->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Duro_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    duro = new Duro(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Duro_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    duro = new Duro();
    duro->SetCollisionSystemType(collision_system_type);
    duro->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Duro_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    duro->SetChassisCollisionType(chassis_collision_type);
    duro->SetChassisFixed(false);
    duro->SetInitPosition(init_pos);
    duro->SetEngineType(EngineModelType::SHAFTS);
    duro->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    duro->SetTireType(TireModelType::TMEASY);
    duro->SetBrakeType(BrakeType::SHAFTS);
    duro->SetInitFwdVel(0.0);
    duro->Initialize();

    duro->SetChassisVisualizationType(chassis_vis);
    duro->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    duro->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    duro->SetWheelVisualizationType(VisualizationType::NONE);
    duro->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void FEDA_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    feda = new FEDA(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void FEDA_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    feda = new FEDA();
    feda->SetCollisionSystemType(collision_system_type);
    feda->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void FEDA_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    feda->SetChassisCollisionType(chassis_collision_type);
    feda->SetChassisFixed(false);
    feda->SetInitPosition(init_pos);
    feda->SetEngineType(EngineModelType::SIMPLE_MAP);
    feda->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    feda->SetTireType(TireModelType::PAC02);
    feda->SetBrakeType(BrakeType::SHAFTS);
    feda->SetAerodynamicDrag(0.6, 3.8, 1.2041);
    feda->Initialize();

    feda->SetChassisVisualizationType(chassis_vis);
    feda->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    feda->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    feda->SetWheelVisualizationType(VisualizationType::MESH);
    feda->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void G500_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    gclass = new G500(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void G500_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    gclass = new G500();
    gclass->SetCollisionSystemType(collision_system_type);
    gclass->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void G500_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    gclass->SetChassisCollisionType(chassis_collision_type);
    gclass->SetChassisFixed(false);
    gclass->SetInitPosition(init_pos);
    gclass->SetTireType(TireModelType::TMEASY);
    gclass->SetAerodynamicDrag(0.5, 5.0, 1.2);
    gclass->Initialize();

    gclass->SetChassisVisualizationType(chassis_vis);
    gclass->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    gclass->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    gclass->SetWheelVisualizationType(VisualizationType::MESH);
    gclass->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Gator_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    gator = new Gator(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Gator_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    gator = new Gator();
    gator->SetCollisionSystemType(collision_system_type);
    gator->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Gator_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    gator->SetChassisCollisionType(chassis_collision_type);
    gator->SetChassisFixed(false);
    gator->SetInitPosition(init_pos);
    gator->SetTireType(TireModelType::TMEASY);
    gator->SetDrivelineType(DrivelineTypeWV::SIMPLE);
    gator->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    gator->SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator->Initialize();

    gator->SetChassisVisualizationType(chassis_vis);
    gator->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    gator->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    gator->SetWheelVisualizationType(VisualizationType::MESH);
    gator->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    hmmwv = new HMMWV_Full(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    hmmwv = new HMMWV_Full();
    hmmwv->SetCollisionSystemType(collision_system_type);
    hmmwv->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    hmmwv->SetChassisCollisionType(chassis_collision_type);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(init_pos);
    hmmwv->SetEngineType(EngineModelType::SIMPLE_MAP);
    hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_CVT);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->UseTierodBodies(true);
    hmmwv->SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    hmmwv->SetBrakeType(BrakeType::SHAFTS);
    hmmwv->SetTireType(TireModelType::TMEASY);
    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(chassis_vis);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV9_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    hmmwv = new HMMWV_Reduced(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV9_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    hmmwv = new HMMWV_Reduced();
    hmmwv->SetCollisionSystemType(collision_system_type);
    hmmwv->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV9_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(init_pos);
    hmmwv->SetChassisCollisionType(chassis_collision_type);
    hmmwv->SetEngineType(EngineModelType::SHAFTS);
    hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    hmmwv->SetTireType(TireModelType::PAC89);
    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(chassis_vis);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv->SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void KRAZ_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    kraz = new Kraz(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void KRAZ_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    kraz = new Kraz();
    kraz->SetCollisionSystemType(collision_system_type);
    kraz->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void KRAZ_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    kraz->SetChassisCollisionType(chassis_collision_type);
    kraz->SetChassisFixed(false);
    kraz->SetInitPosition(init_pos);
    kraz->SetInitFwdVel(0.0);
    kraz->Initialize();

    kraz->SetChassisVisualizationType(chassis_vis, VisualizationType::PRIMITIVES);
    kraz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    kraz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
    kraz->SetWheelVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
    kraz->SetTireVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void LMTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    lmtv = new LMTV(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void LMTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    lmtv = new LMTV();
    lmtv->SetCollisionSystemType(collision_system_type);
    lmtv->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void LMTV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    lmtv->SetChassisCollisionType(chassis_collision_type);
    lmtv->SetChassisFixed(false);
    lmtv->SetInitPosition(init_pos);
    lmtv->SetEngineType(EngineModelType::SIMPLE_MAP);
    lmtv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    lmtv->SetTireType(TireModelType::TMEASY);
    lmtv->Initialize();

    lmtv->SetChassisVisualizationType(chassis_vis);
    lmtv->SetChassisRearVisualizationType(chassis_vis);
    lmtv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    lmtv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    lmtv->SetWheelVisualizationType(VisualizationType::MESH);
    lmtv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    mtv = new MTV(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    mtv = new MTV();
    mtv->SetCollisionSystemType(collision_system_type);
    mtv->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MTV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    mtv->SetChassisCollisionType(chassis_collision_type);
    mtv->SetChassisFixed(false);
    mtv->SetInitPosition(init_pos);
    mtv->UseWalkingBeamRearSuspension(false);
    mtv->SetEngineType(EngineModelType::SIMPLE_MAP);
    mtv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    mtv->SetTireType(TireModelType::TMEASY);
    mtv->Initialize();

    mtv->SetChassisVisualizationType(chassis_vis);
    mtv->SetChassisRearVisualizationType(chassis_vis);
    mtv->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    mtv->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mtv->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mtv->SetWheelVisualizationType(VisualizationType::MESH);
    mtv->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN5_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_5t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN5_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_5t();
    man->SetCollisionSystemType(collision_system_type);
    man->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN5_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    man->SetChassisCollisionType(chassis_collision_type);
    man->SetChassisFixed(false);
    man->SetInitPosition(init_pos);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(chassis_vis);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN7_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_7t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN7_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_7t();
    man->SetCollisionSystemType(collision_system_type);
    man->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN7_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    man->SetChassisCollisionType(chassis_collision_type);
    man->SetChassisFixed(false);
    man->SetInitPosition(init_pos);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(chassis_vis);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN10_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_10t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN10_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    man = new MAN_10t();
    man->SetCollisionSystemType(collision_system_type);
    man->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN10_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    man->SetChassisCollisionType(chassis_collision_type);
    man->SetChassisFixed(false);
    man->SetInitPosition(init_pos);
    man->SetEngineType(EngineModelType::SIMPLE);
    man->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    man->SetTireType(TireModelType::TMSIMPLE);
    man->SetBrakeType(BrakeType::SHAFTS);
    man->Initialize();

    man->SetChassisVisualizationType(chassis_vis);
    man->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    man->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    man->SetWheelVisualizationType(VisualizationType::MESH);
    man->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MROLE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    mrole = new mrole_Full(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MROLE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    mrole = new mrole_Full();
    mrole->SetCollisionSystemType(collision_system_type);
    mrole->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MROLE_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    mrole->SetChassisCollisionType(chassis_collision_type);
    mrole->SetChassisFixed(false);
    mrole->SetInitPosition(init_pos);
    mrole->SetEngineType(EngineModelType::SHAFTS);
    mrole->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    mrole->SetDriveType(DrivelineTypeWV::AWD6);
    mrole->SetBrakeType(BrakeType::SHAFTS);
    mrole->SetTireType(TireModelType::TMEASY);
    mrole->SelectRoadOperation();
    mrole->Initialize();

    mrole->SetChassisVisualizationType(chassis_vis);
    mrole->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    mrole->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    mrole->SetWheelVisualizationType(VisualizationType::MESH);
    mrole->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void BMW_E90_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    bmw = new BMW_E90(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void BMW_E90_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    bmw = new BMW_E90();
    bmw->SetCollisionSystemType(collision_system_type);
    bmw->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void BMW_E90_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    bmw->SetChassisCollisionType(chassis_collision_type);
    bmw->SetChassisFixed(false);
    bmw->SetInitPosition(init_pos);
    bmw->SetTireType(TireModelType::TMEASY);
    bmw->SetBrakeType(BrakeType::SHAFTS);
    bmw->Initialize();

    bmw->SetChassisVisualizationType(chassis_vis);
    bmw->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    bmw->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    bmw->SetWheelVisualizationType(VisualizationType::MESH);
    bmw->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Sedan_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    sedan = new Sedan(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Sedan_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    sedan = new Sedan();
    sedan->SetCollisionSystemType(collision_system_type);
    sedan->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Sedan_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    sedan->SetChassisCollisionType(chassis_collision_type);
    sedan->SetChassisFixed(false);
    sedan->SetInitPosition(init_pos);
    sedan->SetTireType(TireModelType::PAC02);
    sedan->SetBrakeType(BrakeType::SHAFTS);
    sedan->Initialize();

    sedan->SetChassisVisualizationType(chassis_vis);
    sedan->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    sedan->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    sedan->SetWheelVisualizationType(VisualizationType::MESH);
    sedan->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUS_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    uaz = new UAZBUS(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUS_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    uaz = new UAZBUS();
    uaz->SetCollisionSystemType(collision_system_type);
    uaz->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUS_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    uaz->SetChassisCollisionType(chassis_collision_type);
    uaz->SetChassisFixed(false);
    uaz->SetInitPosition(init_pos);
    uaz->SetTireType(TireModelType::PAC02);
    uaz->SetInitFwdVel(0.0);
    uaz->Initialize();

    uaz->SetChassisVisualizationType(chassis_vis);
    uaz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetWheelVisualizationType(VisualizationType::MESH);
    uaz->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUSSAE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    uaz = new UAZBUS_SAE(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUSSAE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    uaz = new UAZBUS_SAE();
    uaz->SetCollisionSystemType(collision_system_type);
    uaz->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUSSAE_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    uaz->SetChassisCollisionType(chassis_collision_type);
    uaz->SetChassisFixed(false);
    uaz->SetInitPosition(init_pos);
    uaz->SetTireType(TireModelType::TMEASY);
    uaz->SetInitFwdVel(0.0);
    uaz->Initialize();

    uaz->SetChassisVisualizationType(chassis_vis);
    uaz->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    uaz->SetWheelVisualizationType(VisualizationType::MESH);
    uaz->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void U401_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    u401 = new U401(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void U401_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    u401 = new U401();
    u401->SetCollisionSystemType(collision_system_type);
    u401->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void U401_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    u401->SetChassisCollisionType(chassis_collision_type);
    u401->SetChassisFixed(false);
    u401->SetInitPosition(init_pos);
    u401->SetTireType(TireModelType::TMEASY);
    u401->SetBrakeType(BrakeType::SHAFTS);
    u401->SetInitFwdVel(0.0);
    u401->Initialize();

    u401->SetChassisVisualizationType(chassis_vis);
    u401->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    u401->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    u401->SetWheelVisualizationType(VisualizationType::MESH);
    u401->SetTireVisualizationType(VisualizationType::MESH);
}

// =============================================================================

std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> WheeledVehicleModel::List() {
    std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> models = {
        {chrono_types::make_shared<ARTcar_Model>(), "ARTCAR"},         //
        {chrono_types::make_shared<Cherokee_Model>(), "Cherokee"},     //
        {chrono_types::make_shared<BMW_E90_Model>(), "BMW_E90"},       //
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
