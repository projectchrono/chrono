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

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================
// Specification of a vehicle model from the Chrono::Vehicle model library

class WheeledVehicleModel {
  public:
    virtual ~WheeledVehicleModel() {}

    virtual std::string ModelName() const = 0;
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis = true) = 0;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis = true) = 0;
    virtual ChWheeledVehicle& GetVehicle() = 0;
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) = 0;
    virtual void Advance(double step) = 0;
    virtual ChVector3d TrackPoint() const = 0;
    virtual double CameraDistance() const = 0;
    virtual double CameraHeight() const = 0;

    void SetCollisionSystemType(ChCollisionSystem::Type type) { collision_system_type = type; }
    void SetChassisCollisionType(CollisionType type) { chassis_collision_type = type; }
    void SetTireCollisionType(ChTire::CollisionType type) { tire_collision_type = type; }

    static std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> List();

  protected:
    WheeledVehicleModel() : collision_system_type(ChCollisionSystem::Type::BULLET), chassis_collision_type(CollisionType::NONE) {}

    ChCollisionSystem::Type collision_system_type;
    CollisionType chassis_collision_type;
    ChTire::CollisionType tire_collision_type;
};

class Citybus_Model : public WheeledVehicleModel {
  public:
    Citybus_Model() : car(nullptr) {}
    ~Citybus_Model() { delete car; }

    virtual std::string ModelName() const override { return "CITYBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 16.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    CityBus* car;
};

class Duro_Model : public WheeledVehicleModel {
  public:
    Duro_Model() : car(nullptr) {}
    ~Duro_Model() { delete car; }

    virtual std::string ModelName() const override { return "DURO"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Duro* car;
};

class FEDA_Model : public WheeledVehicleModel {
  public:
    FEDA_Model() : car(nullptr) {}
    ~FEDA_Model() { delete car; }

    virtual std::string ModelName() const override { return "FEDA"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 0.75); }
    virtual double CameraDistance() const override { return 10.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    FEDA* car;
};

class G500_Model : public WheeledVehicleModel {
  public:
    G500_Model() : car(nullptr) {}
    ~G500_Model() { delete car; }

    virtual std::string ModelName() const override { return "G500"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    G500* car;
};

class Gator_Model : public WheeledVehicleModel {
  public:
    Gator_Model() : car(nullptr) {}
    ~Gator_Model() { delete car; }

    virtual std::string ModelName() const override { return "GATOR"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 7.0; }
    virtual double CameraHeight() const override { return 0.1; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Gator* car;
};

class HMMWV_Model : public WheeledVehicleModel {
  public:
    HMMWV_Model() : car(nullptr) {}
    ~HMMWV_Model() { delete car; }

    virtual std::string ModelName() const override { return "HMMWV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    HMMWV_Full* car;
};

class HMMWV9_Model : public WheeledVehicleModel {
  public:
    HMMWV9_Model() : car(nullptr) {}
    ~HMMWV9_Model() { delete car; }

    virtual std::string ModelName() const override { return "HMMWV9"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    HMMWV_Reduced* car;
};

class KRAZ_Model : public WheeledVehicleModel {
  public:
    KRAZ_Model() : car(nullptr) {}
    ~KRAZ_Model() { delete car; }

    virtual std::string ModelName() const override { return "KRAZ"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetTractor(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 25.0; }
    virtual double CameraHeight() const override { return 1.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Kraz* car;
};

class LMTV_Model : public WheeledVehicleModel {
  public:
    LMTV_Model() : car(nullptr) {}
    ~LMTV_Model() { delete car; }

    virtual std::string ModelName() const override { return "LMTV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    LMTV* car;
};

class MTV_Model : public WheeledVehicleModel {
  public:
    MTV_Model() : car(nullptr) {}
    ~MTV_Model() { delete car; }

    virtual std::string ModelName() const override { return "MTV"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 12.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MTV* car;
};

class MAN5_Model : public WheeledVehicleModel {
  public:
    MAN5_Model() : car(nullptr) {}
    ~MAN5_Model() { delete car; }

    virtual std::string ModelName() const override { return "MAN5t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_5t* car;
};

class MAN7_Model : public WheeledVehicleModel {
  public:
    MAN7_Model() : car(nullptr) {}
    ~MAN7_Model() { delete car; }

    virtual std::string ModelName() const override { return "MAN7t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_7t* car;
};

class MAN10_Model : public WheeledVehicleModel {
  public:
    MAN10_Model() : car(nullptr) {}
    ~MAN10_Model() { delete car; }

    virtual std::string ModelName() const override { return "MAN10t"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    MAN_10t* car;
};

class MROLE_Model : public WheeledVehicleModel {
  public:
    MROLE_Model() : car(nullptr) {}
    ~MROLE_Model() { delete car; }

    virtual std::string ModelName() const override { return "MROLE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 14.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    mrole_Full* car;
};

class Sedan_Model : public WheeledVehicleModel {
  public:
    Sedan_Model() : car(nullptr) {}
    ~Sedan_Model() { delete car; }

    virtual std::string ModelName() const override { return "SEDAN"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Sedan* car;
};

class BMW_E90_Model : public WheeledVehicleModel {
  public:
    BMW_E90_Model() : car(nullptr) {}
    ~BMW_E90_Model() { delete car; }

    virtual std::string ModelName() const override { return "BMW_E90"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    BMW_E90* car;
};

class Cherokee_Model : public WheeledVehicleModel {
  public:
    Cherokee_Model() : car(nullptr) {}
    ~Cherokee_Model() { delete car; }

    virtual std::string ModelName() const override { return "Cherokee"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.2; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    Cherokee* car;
};

class UAZBUS_Model : public WheeledVehicleModel {
  public:
    UAZBUS_Model() : car(nullptr) {}
    ~UAZBUS_Model() { delete car; }

    virtual std::string ModelName() const override { return "UAZBUS"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    UAZBUS* car;
};

class UAZBUSSAE_Model : public WheeledVehicleModel {
  public:
    UAZBUSSAE_Model() : car(nullptr) {}
    ~UAZBUSSAE_Model() { delete car; }

    virtual std::string ModelName() const override { return "UAZBUSSAE"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    UAZBUS_SAE* car;
};

class U401_Model : public WheeledVehicleModel {
  public:
    U401_Model() : car(nullptr) {}
    ~U401_Model() { delete car; }

    virtual std::string ModelName() const override { return "UNIMOG"; }
    virtual void Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual void Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) override;
    virtual ChWheeledVehicle& GetVehicle() override { return car->GetVehicle(); }
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) override { car->Synchronize(time, driver_inputs, terrain); }
    virtual void Advance(double step) override { car->Advance(step); }
    virtual ChVector3d TrackPoint() const override { return ChVector3d(0, 0, 1.75); }
    virtual double CameraDistance() const override { return 8.0; }
    virtual double CameraHeight() const override { return 0.5; }

  private:
    void Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis);
    U401* car;
};

// =============================================================================

void Citybus_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new CityBus(system);
    car->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Citybus_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new CityBus();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    car->SetInitPosition(init_pos);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Citybus_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetTireType(TireModelType::PAC02);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Cherokee_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Cherokee(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Cherokee_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Cherokee();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Cherokee_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Duro_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Duro(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Duro_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Duro();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Duro_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SHAFTS);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->SetInitFwdVel(0.0);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::NONE);
    car->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void FEDA_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new FEDA(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void FEDA_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new FEDA();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void FEDA_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE_MAP);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::PAC02);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->SetAerodynamicDrag(0.6, 3.8, 1.2041);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void G500_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new G500(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void G500_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new G500();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void G500_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetAerodynamicDrag(0.5, 5.0, 1.2);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Gator_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Gator(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Gator_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Gator();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Gator_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetDrivelineType(DrivelineTypeWV::SIMPLE);
    car->SetBrakeType(chrono::vehicle::BrakeType::SHAFTS);
    car->SetAerodynamicDrag(0.5, 5.0, 1.2);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new HMMWV_Full(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new HMMWV_Full();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE_MAP);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_CVT);
    car->SetDriveType(DrivelineTypeWV::AWD);
    car->UseTierodBodies(true);
    car->SetSteeringType(SteeringTypeWV::PITMAN_ARM);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void HMMWV9_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new HMMWV_Reduced(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV9_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new HMMWV_Reduced();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void HMMWV9_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetEngineType(EngineModelType::SHAFTS);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    car->SetDriveType(DrivelineTypeWV::AWD);
    car->SetTireType(TireModelType::PAC89);
    car->SetTireCollisionType(tire_collision_type);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::NONE);
    car->SetTireVisualizationType(VisualizationType::PRIMITIVES);
}

// -----------------------------------------------------------------------------

void KRAZ_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Kraz(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void KRAZ_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Kraz();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void KRAZ_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetInitFwdVel(0.0);
    car->SetTireCollisionType(tire_collision_type);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis, VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH, VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void LMTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new LMTV(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void LMTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new LMTV();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void LMTV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE_MAP);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetChassisRearVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MTV_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MTV(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MTV_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MTV();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MTV_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->UseWalkingBeamRearSuspension(false);
    car->SetEngineType(EngineModelType::SIMPLE_MAP);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetChassisRearVisualizationType(chassis_vis);
    car->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN5_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_5t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN5_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_5t();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN5_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::TMSIMPLE);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN7_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_7t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN7_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_7t();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN7_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::TMSIMPLE);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MAN10_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_10t(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN10_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new MAN_10t();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MAN10_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SIMPLE);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    car->SetTireType(TireModelType::TMSIMPLE);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void MROLE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new mrole_Full(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MROLE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new mrole_Full();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void MROLE_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetEngineType(EngineModelType::SHAFTS);
    car->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    car->SetDriveType(DrivelineTypeWV::AWD6);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SelectRoadOperation();
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void BMW_E90_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new BMW_E90(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void BMW_E90_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new BMW_E90();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void BMW_E90_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void Sedan_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Sedan(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Sedan_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new Sedan();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void Sedan_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::PAC02);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUS_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new UAZBUS(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUS_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new UAZBUS();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUS_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::PAC02);
    car->SetTireCollisionType(tire_collision_type);
    car->SetInitFwdVel(0.0);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void UAZBUSSAE_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new UAZBUS_SAE(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUSSAE_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new UAZBUS_SAE();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void UAZBUSSAE_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetInitFwdVel(0.0);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// -----------------------------------------------------------------------------

void U401_Model::Create(ChSystem* system, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new U401(system);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void U401_Model::Create(ChContactMethod contact_method, const ChCoordsys<>& init_pos, bool chassis_vis) {
    car = new U401();
    car->SetCollisionSystemType(collision_system_type);
    car->SetContactMethod(contact_method);
    Construct(init_pos, chassis_vis ? VisualizationType::MESH : VisualizationType::NONE);
}

void U401_Model::Construct(const ChCoordsys<>& init_pos, VisualizationType chassis_vis) {
    car->SetChassisCollisionType(chassis_collision_type);
    car->SetChassisFixed(false);
    car->SetInitPosition(init_pos);
    car->SetTireType(TireModelType::TMEASY);
    car->SetTireCollisionType(tire_collision_type);
    car->SetBrakeType(BrakeType::SHAFTS);
    car->SetInitFwdVel(0.0);
    car->Initialize();

    car->SetChassisVisualizationType(chassis_vis);
    car->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    car->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    car->SetWheelVisualizationType(VisualizationType::MESH);
    car->SetTireVisualizationType(VisualizationType::MESH);
}

// =============================================================================

std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> WheeledVehicleModel::List() {
    std::vector<std::pair<std::shared_ptr<WheeledVehicleModel>, std::string>> models = {
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
