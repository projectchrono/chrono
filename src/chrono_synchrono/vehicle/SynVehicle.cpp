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
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Class that wraps a ChVehicle. The underlying vehicle dynamics remain
// untouched; however, additional functionality is added to allow for SynChrono
// to pass messages and remain synchronized across ranks.
//
// Currently, both wheeled and tracked vehicles are wrapped. See
// SynWheeledVehicle and SynTrackedVehicle for their additional methods.
//
// A SynVehicle will always on the system unless a vehicle is passed in through
// the SynCustom__Vehicle classes. This means that either the system is
// passed on instantiation or is created by this class. If this was not the case
// the underlying ChVehicle may instantiate its own. This can cause issues with
// destruction and nullptrs associated with the systems.
//
// =============================================================================

#include "chrono_synchrono/vehicle/SynVehicle.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

using namespace rapidjson;

namespace chrono {
namespace synchrono {

SynVehicle::SynVehicle(bool is_zombie) : m_is_zombie(is_zombie), m_owns_vehicle(false), m_system(nullptr) {}

SynVehicle::SynVehicle(const std::string& filename, ChContactMethod contact_method)
    : m_is_zombie(false), m_owns_vehicle(true) {
    m_system = (contact_method == ChContactMethod::NSC) ? static_cast<ChSystem*>(new ChSystemNSC)
                                                        : static_cast<ChSystem*>(new ChSystemSMC);

    m_system->Set_G_acc(-9.81 * VECT_Z);

    // Integration and Solver settings
    switch (contact_method) {
        case ChContactMethod::NSC:
            m_system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
            break;
        default:
            break;
    }

    m_system->SetSolverMaxIterations(150);
    m_system->SetMaxPenetrationRecoverySpeed(4.0);
}

SynVehicle::SynVehicle(const std::string& filename, ChSystem* system)
    : m_system(system), m_is_zombie(false), m_owns_vehicle(true) {}

SynVehicle::SynVehicle(const std::string& filename) : m_is_zombie(true), m_owns_vehicle(false) {}

SynVehicle::~SynVehicle() {
    // Will delete the system if the vehicle was created by this object
    // If it wasn't, it must have been passed in and the vehicle is then
    // responsible for deleting the system
    if (m_owns_vehicle && m_system)
        delete m_system;
}

// ---------------------------------------------------------------------------

void SynVehicle::Advance(double step) {
    GetVehicle().Advance(step);

    // Will usually own the system, so the system also needs to be advanced
    // If this object does not own the system, this method should be overridden.
    if (m_owns_vehicle)
        m_system->DoStepDynamics(step);
}

// ---------------------------------------------------------------------------

Document SynVehicle::ParseVehicleFileJSON(const std::string& filename) {
    // Open and parse the input file
    auto d = ReadFileJSON(filename);
    if (d.IsNull())
        throw ChException("Vehicle file not read properly in ParseVehicleFileJSON.");

    // Read top-level data
    assert(d.HasMember("Name"));
    assert(d.HasMember("Type"));

    std::string name = d["Name"].GetString();
    std::string type = d["Type"].GetString();
    assert(type.compare("Vehicle") == 0);

    // ----------------------------
    // Validations of the JSON file
    // ----------------------------

    assert(d.HasMember("Vehicle"));
    assert(d.HasMember("Powertrain"));
    assert(d.HasMember("Zombie"));

    assert(d["Vehicle"].HasMember("Chassis Visualization Type"));

    assert(d["Zombie"].HasMember("Chassis Visualization File"));

    return d;
}

// ---------------------------------------------------------------------------

std::shared_ptr<ChTriangleMeshShape> SynVehicle::CreateMeshZombieComponent(const std::string& filename) {
    auto mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    mesh->LoadWavefrontMesh(vehicle::GetDataFile(filename), false, false);

    auto trimesh = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh->SetMesh(mesh);
    trimesh->SetStatic(true);
    trimesh->SetName(filename);

    return trimesh;
}

void SynVehicle::CreateChassisZombieBody(const std::string& filename, ChSystem* system) {
    auto trimesh = CreateMeshZombieComponent(filename);

    m_zombie_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_zombie_body->AddAsset(trimesh);
    m_zombie_body->SetCollide(false);
    m_zombie_body->SetBodyFixed(true);
    m_zombie_body->SetFrame_COG_to_REF(ChFrame<>({0, 0, -0.2}, {1, 0, 0, 0}));
    system->Add(m_zombie_body);
}

}  // namespace synchrono
}  // namespace chrono
