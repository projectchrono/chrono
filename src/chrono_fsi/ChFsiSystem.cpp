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

////#define DEBUG_LOG

#include <cmath>
#include <stdexcept>
#include <thread>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtils.h"
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

ChFsiSystem::ChFsiSystem(ChSystem* sysMBS, ChFsiFluidSystem* sysCFD)
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

void ChFsiSystem::AttachFluidSystem(ChFsiFluidSystem* sys) {
    m_sysCFD = sys;
    m_sysCFD->SetVerbose(m_verbose);
}

void ChFsiSystem::AttachMultibodySystem(ChSystem* sys) {
    m_sysMBS = sys;
    m_fsi_interface->AttachMultibodySystem(sys);
}

ChFsiFluidSystem& ChFsiSystem::GetFluidSystem() const {
    assert(m_sysCFD != nullptr);
    return *m_sysCFD;
}

ChSystem& ChFsiSystem::GetMultibodySystem() const {
    assert(m_sysMBS != nullptr);
    return *m_sysMBS;
}

ChFsiInterface& ChFsiSystem::GetFsiInterface() const {
    return *m_fsi_interface;
}

double ChFsiSystem::GetRtfCFD() const {
    if (!m_sysCFD)
        return 0;
    return m_sysCFD->GetRtf();
}

double ChFsiSystem::GetRtfMBD() const {
    if (!m_sysMBS)
        return 0;
    return m_sysMBS->GetRTF();
}

void ChFsiSystem::SetVerbose(bool verbose) {
    ChAssertAlways(m_fsi_interface);
    if (m_sysCFD)
        m_sysCFD->SetVerbose(verbose);
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
    if (m_sysCFD)
        m_sysCFD->SetGravitationalAcceleration(gravity);
    if (m_sysMBS)
        m_sysMBS->SetGravitationalAcceleration(gravity);
}

void ChFsiSystem::UseNodeDirections(NodeDirectionsMode mode) {
    ChAssertAlways(m_fsi_interface);
    ChDebugLog("uses direction data? " << (mode != NodeDirectionsMode::NONE));
    m_fsi_interface->UseNodeDirections(mode);
    if (m_sysCFD)
        m_sysCFD->UseNodeDirections(mode);
}

std::shared_ptr<FsiBody> ChFsiSystem::AddFsiBody(std::shared_ptr<ChBody> body,
                                                 std::shared_ptr<ChBodyGeometry> geometry,
                                                 bool check_embedded) {
    ChAssertAlways(m_fsi_interface);
    return m_fsi_interface->AddFsiBody(body, geometry, check_embedded);
}

std::shared_ptr<FsiMesh1D> ChFsiSystem::AddFsiMesh1D(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded) {
    ChAssertAlways(m_fsi_interface);

    // Search for contact surfaces associated with the FEA mesh
    for (const auto& surface : mesh->GetContactSurfaces()) {
        if (auto surface_segs = std::dynamic_pointer_cast<fea::ChContactSurfaceSegmentSet>(surface)) {
            if (surface_segs->GetNumSegments() > 0)
                return m_fsi_interface->AddFsiMesh1D(surface_segs, check_embedded);
        }
    }

    // If no 1D surface contact found, create one with a default contact material and add segments from cable and beam
    // elements in the FEA mesh. Do not add the new contact surface to the mesh.
    ChContactMaterialData contact_material_data;
    auto surface_segs = chrono_types::make_shared<fea::ChContactSurfaceSegmentSet>(
        contact_material_data.CreateMaterial(ChContactMethod::SMC));
    surface_segs->AddAllSegments(*mesh, 0);
    if (surface_segs->GetNumSegments() > 0)
        return m_fsi_interface->AddFsiMesh1D(surface_segs, check_embedded);

    // The FEA mesh contains no 1D elements (cable or beam)
    return nullptr;
}

std::shared_ptr<FsiMesh2D> ChFsiSystem::AddFsiMesh2D(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded) {
    ChAssertAlways(m_fsi_interface);

    // Search for contact surfaces associated with the FEA mesh
    for (const auto& surface : mesh->GetContactSurfaces()) {
        if (auto surface_mesh = std::dynamic_pointer_cast<fea::ChContactSurfaceMesh>(surface)) {
            if (surface_mesh->GetNumTriangles() > 0)
                return m_fsi_interface->AddFsiMesh2D(surface_mesh, check_embedded);
        }
    }

    // If no 2D surface contact found, create one with a default contact material and extract the boundary faces of the
    // FEA mesh. Do not add the new contact surface to the mesh.
    ChContactMaterialData contact_material_data;
    auto surface_mesh = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
        contact_material_data.CreateMaterial(ChContactMethod::SMC));
    surface_mesh->AddFacesFromBoundary(*mesh, 0, true, false, false);  // do not include cable and beam elements
    if (surface_mesh->GetNumTriangles() > 0)
        return m_fsi_interface->AddFsiMesh2D(surface_mesh, check_embedded);

    // The FEA mesh contains no 2D elements (shell or solid)
    return nullptr;
}

void ChFsiSystem::Initialize() {
    ChAssertAlways(m_sysCFD);
    ChAssertAlways(m_sysMBS);
    ChAssertAlways(m_fsi_interface);

    if (m_step_CFD < 0)
        m_step_CFD = m_sysCFD->GetStepSize();

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
    m_sysCFD->SetStepSize(m_step_CFD);
    m_sysCFD->Initialize(body_states, mesh1D_states, mesh2D_states);

    // Mark systems as initialized
    m_sysCFD->m_is_initialized = true;
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
    constexpr double min_step = 1e-6;
    while (t < step) {
        // In case variable time step is not used - this will just return the step size

        double h = std::max<>(m_sysCFD->GetVariableStepSize(), min_step);
        h = std::min<>(h, step - t);

        // double h_new = m_sysCFD.GetVariableStepSize();
        // double h = std::min<>(m_step_CFD, step - t);
        if (h <= threshold)
            break;
        m_sysCFD->DoStepDynamics(h);
        t += h;
    }
    m_timer_CFD = m_sysCFD->GetTimerStep();
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
            m_sysMBS->DoStepDynamics(h);
            t += h;
        }
    }
    m_timer_MBD = m_sysMBS->GetTimerStep();
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

    // Advance dynamics of the two phases.
    //   1. Advance the dynamics of the multibody system in a concurrent thread (does not block execution)
    //   2. Advance the dynamics of the fluid system (in the main thread)
    //   3. Wait for the MBS thread to finish execution.
    m_timer_step.start();
    std::thread th(&ChFsiSystem::AdvanceMBS, this, step, threshold_MBD);
    AdvanceCFD(step, threshold_CFD);
    th.join();
    m_timer_step.stop();

    // Data exchange between phases:
    //   1. [CFD -> MBS] Apply fluid forces and torques on FSI solids
    //   2. [MBS -> CFD] Load new solid phase states
    m_timer_FSI.start();
    m_sysCFD->OnExchangeSolidForces();
    m_fsi_interface->ExchangeSolidForces();
    m_fsi_interface->ExchangeSolidStates();
    m_sysCFD->OnExchangeSolidStates();
    m_timer_FSI.stop();

    // Calculate RTF and MBD/CFD timer ratio
    m_RTF = m_timer_step() / step;
    m_ratio_MBD = m_timer_MBD / m_timer_CFD;

    // Update simulation time
    m_time += step;
}

}  // end namespace fsi
}  // end namespace chrono
