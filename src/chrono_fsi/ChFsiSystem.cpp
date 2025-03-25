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
// Author: Radu Serban
// =============================================================================
//
// Base class for an FSI system, independent of the fluid solver
//
// =============================================================================

#include <cmath>
#include <stdexcept>
#include <thread>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/ChFsiSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChFsiSystem::ChFsiSystem(ChSystem& sysMBS, ChFsiFluidSystem& sysCFD)
    : m_sysMBS(sysMBS),
      m_sysCFD(sysCFD),
      m_fsi_interface(nullptr),
      m_is_initialized(false),
      m_verbose(true),
      m_step_MBD(-1),
      m_step_CFD(-1),
      m_time(0),
      m_RTF(-1),
      m_ratio_MBD(-1) {}

ChFsiSystem::~ChFsiSystem() {}

ChFsiFluidSystem& ChFsiSystem::GetFluidSystem() const {
    return m_sysCFD;
}

ChSystem& ChFsiSystem::GetMultibodySystem() const {
    return m_sysMBS;
}

ChFsiInterface& ChFsiSystem::GetFsiInterface() const {
    return *m_fsi_interface;
}

void ChFsiSystem::SetVerbose(bool verbose) {
    ChAssertAlways(m_fsi_interface);
    m_fsi_interface->SetVerbose(verbose);
    m_verbose = verbose;
}

void ChFsiSystem::SetStepsizeMBD(double step) {
    m_step_MBD = step;
}

void ChFsiSystem::SetStepSizeCFD(double step) {
    m_step_CFD = step;
}

void ChFsiSystem::SetGravitationalAcceleration(const ChVector3d& gravity) {
    m_sysCFD.SetGravitationalAcceleration(gravity);
    m_sysMBS.SetGravitationalAcceleration(gravity);
}

size_t ChFsiSystem::AddFsiBody(std::shared_ptr<ChBody> body) {
    unsigned int index = m_fsi_interface->GetNumBodies();
    auto& fsi_body = m_fsi_interface->AddFsiBody(body);
    m_sysCFD.OnAddFsiBody(index, fsi_body);
    return index;
}

void ChFsiSystem::AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh) {
    bool has_contact1D = false;
    bool has_contact2D = false;

    // Search for contact surfaces associated with the FEA mesh
    for (const auto& surface : mesh->GetContactSurfaces()) {
        if (auto surface_segs = std::dynamic_pointer_cast<fea::ChContactSurfaceSegmentSet>(surface)) {
            if (surface_segs->GetNumSegments() > 0)
                AddFsiMesh1D(surface_segs);
            has_contact1D = true;
        } else if (auto surface_mesh = std::dynamic_pointer_cast<fea::ChContactSurfaceMesh>(surface)) {
            if (surface_mesh->GetNumTriangles() > 0)
                AddFsiMesh2D(surface_mesh);
            has_contact2D = true;
        }
    }

    // If no 1D surface contact found, create one with a default contact material and add segments from cable and beam
    // elements in the FEA mesh. Do not add the new contact surface to the mesh.
    if (!has_contact1D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_segs = chrono_types::make_shared<fea::ChContactSurfaceSegmentSet>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        surface_segs->AddAllSegments(*mesh, 0);
        if (surface_segs->GetNumSegments() > 0)
            AddFsiMesh1D(surface_segs);
    }

    // If no 2D surface contact found, create one with a default contact material and extract the boundary faces of the
    // FEA mesh. Do not add the new contact surface to the mesh.
    if (!has_contact2D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_mesh = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        surface_mesh->AddFacesFromBoundary(*mesh, 0, true, false, false);  // do not include cable and beam elements
        if (surface_mesh->GetNumTriangles() > 0)
            AddFsiMesh2D(surface_mesh);
    }
}

void ChFsiSystem::AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface) {
    unsigned int index = m_fsi_interface->GetNumMeshes1D();
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh1D(surface);
    m_sysCFD.OnAddFsiMesh1D(index, fsi_mesh);
}

void ChFsiSystem::AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface) {
    unsigned int index = m_fsi_interface->GetNumMeshes2D();
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh2D(surface);
    m_sysCFD.OnAddFsiMesh2D(index, fsi_mesh);
}

void ChFsiSystem::Initialize() {
    if (!m_fsi_interface) {
        cout << "ERROR: No FSI interface was created." << endl;
        throw std::runtime_error("No FSI interface was created.");
    }

    if (m_step_CFD < 0)
        m_step_CFD = m_sysCFD.GetStepSize();

    if (m_step_CFD < 0) {
        cout << "ERROR: Integration step size for fluid dynamics not set." << endl;
        throw std::runtime_error("Integration step size for fluid dynamics not set.");
    }

    if (m_step_MBD < 0) {
        m_step_MBD = m_step_CFD;
    }

    // Initialize the FSI interface and extract initial solid states
    m_fsi_interface->Initialize();

    std::vector<FsiBodyState> body_states;
    std::vector<FsiMeshState> mesh1D_states;
    std::vector<FsiMeshState> mesh2D_states;
    m_fsi_interface->AllocateStateVectors(body_states, mesh1D_states, mesh2D_states);
    m_fsi_interface->StoreSolidStates(body_states, mesh1D_states, mesh2D_states);

    // Initialize fluid system with initial solid states
    m_sysCFD.SetStepSize(m_step_CFD);
    m_sysCFD.Initialize(m_fsi_interface->GetNumBodies(),                                        //
                        m_fsi_interface->GetNumNodes1D(), m_fsi_interface->GetNumElements1D(),  //
                        m_fsi_interface->GetNumNodes2D(), m_fsi_interface->GetNumElements2D(),  //
                        body_states, mesh1D_states, mesh2D_states);                             //

    // Mark system as initialized
    m_is_initialized = true;
}

const ChVector3d& ChFsiSystem::GetFsiBodyForce(size_t i) const {
    return m_fsi_interface->GetFsiBodyForce(i);
}

const ChVector3d& ChFsiSystem::GetFsiBodyTorque(size_t i) const {
    return m_fsi_interface->GetFsiBodyTorque(i);
}

void ChFsiSystem::AdvanceCFD(double step, double threshold) {
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_step_CFD, step - t);
        if (h <= threshold)
            break;
        m_sysCFD.DoStepDynamics(h);
        t += h;
    }
    m_timer_CFD = m_sysCFD.GetTimerStep();
}

void ChFsiSystem::AdvanceMBS(double step, double threshold) {
    double t = 0;
    if (m_MBD_callback) {
        m_MBD_callback->Advance(step, threshold);
    } else {
        while (t < step) {
            double h = std::min<>(m_step_MBD, step - t);
            if (h <= threshold)
                break;
            m_sysMBS.DoStepDynamics(h);
            t += h;
        }
    }
    m_timer_MBD = m_sysMBS.GetTimerStep();
}

void ChFsiSystem::DoStepDynamics(double step) {
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    double factor = 1e-6;
    double threshold_CFD = factor * m_step_CFD;
    double threshold_MBD = factor * m_step_MBD;

    m_timer_step.reset();
    m_timer_FSI.reset();

    m_timer_step.start();

    // Data exchange between phases:
    //   1. [CFD -> MBS] Apply fluid forces and torques on FSI solids
    //   2. [MBS -> CFD] Load new solid phase states
    m_timer_FSI.start();
    m_sysCFD.OnExchangeSolidForces();
    m_fsi_interface->ExchangeSolidForces();
    m_fsi_interface->ExchangeSolidStates();
    m_sysCFD.OnExchangeSolidStates();
    m_timer_FSI.stop();

    // Advance dynamics of the two phases.
    //   1. Advance the dynamics of the multibody system in a concurrent thread (does not block execution)
    //   2. Advance the dynamics of the fluid system (in the main thread)
    //   3. Wait for the MBS thread to finish execution.
    std::thread th(&ChFsiSystem::AdvanceMBS, this, step, threshold_MBD);
    AdvanceCFD(step, threshold_CFD);
    th.join();

    m_timer_step.stop();

    // Calculate RTF and MBD/CFD timer ratio
    m_RTF = m_timer_step() / step;
    m_ratio_MBD = m_timer_MBD / m_timer_CFD;

    // Update simulation time
    m_time += step;
}

}  // end namespace fsi
}  // end namespace chrono
