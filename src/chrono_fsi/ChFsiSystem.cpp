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

ChFsiSystem::ChFsiSystem(ChSystem* sysMBS)
    : m_sysMBS(sysMBS),
      m_is_initialized(false),
      m_verbose(true),
      m_step_MBD(-1),
      m_step_CFD(-1),
      m_time(0),
      m_RTF(0),
      m_ratio_MBS(0),
      m_write_mode(OutputMode::NONE) {}

ChFsiSystem::~ChFsiSystem() {}

void ChFsiSystem::AttachSystem(ChSystem* sysMBS) {
    m_sysMBS = sysMBS;
}

void ChFsiSystem::SetVerbose(bool verbose) {
    m_verbose = verbose;
    m_fsi_interface->m_verbose = verbose;
}

void ChFsiSystem::SetStepsizeMBD(double step) {
    m_step_MBD = step;
}

void ChFsiSystem::SetStepSizeCFD(double step) {
    m_step_CFD = step;
}

size_t ChFsiSystem::AddFsiBody(std::shared_ptr<ChBody> body) {
    size_t index = m_fsi_interface->GetNumBodies();

    auto& fsi_body = m_fsi_interface->AddFsiBody(body);
    OnAddFsiBody(fsi_body);

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
    // elements in the FEA mesh
    if (!has_contact1D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_segs = chrono_types::make_shared<fea::ChContactSurfaceSegmentSet>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        mesh->AddContactSurface(surface_segs);
        surface_segs->AddAllSegments(0);
        if (surface_segs->GetNumSegments() > 0)
            AddFsiMesh1D(surface_segs);
    }

    // If no 2D surface contact found, create one with a default contact material and extract the boundary faces of the
    // FEA mesh
    if (!has_contact2D) {
        ChContactMaterialData contact_material_data;  // default contact material
        auto surface_mesh = chrono_types::make_shared<fea::ChContactSurfaceMesh>(
            contact_material_data.CreateMaterial(ChContactMethod::SMC));
        mesh->AddContactSurface(surface_mesh);
        surface_mesh->AddFacesFromBoundary(0, true, false, false);  // do not include cable and beam elements
        if (surface_mesh->GetNumTriangles() > 0)
            AddFsiMesh2D(surface_mesh);
    }
}

void ChFsiSystem::AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface) {
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh1D(surface);
    OnAddFsiMesh1D(fsi_mesh);
}

void ChFsiSystem::AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface) {
    auto& fsi_mesh = m_fsi_interface->AddFsiMesh2D(surface);
    OnAddFsiMesh2D(fsi_mesh);
}

void ChFsiSystem::Initialize() {
    if (!m_fsi_interface) {
        cout << "ERROR: No FSI interface was created." << endl;
        throw std::runtime_error("No FSI interface was created.");
    }

    if (m_step_CFD < 0) {
        cout << "ERROR: Integration step size for fluid dynamics not set." << endl;
        throw std::runtime_error("Integration step size for fluid dynamics not set.");
    }

    if (m_step_MBD < 0) {
        m_step_MBD = m_step_CFD;
    }

    // Mark system as initialized
    m_is_initialized = true;
}

const ChVector3d& ChFsiSystem::GetFsiBodyForce(size_t i) const {
    return m_fsi_interface->m_fsi_bodies[i].fsi_force;
}

const ChVector3d& ChFsiSystem::GetFsiBodyTorque(size_t i) const {
    return m_fsi_interface->m_fsi_bodies[i].fsi_torque;
}

void ChFsiSystem::DoStepDynamics(double step) {
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    double factor = 1e-6;
    double threshold_CFD = factor * m_step_CFD;
    double threshold_MBD = factor * m_step_MBD;

    double timeCFD = m_time;
    double timeMBD = m_time;

    m_timer_step.reset();
    m_timer_CFD.reset();
    m_timer_MBS.reset();
    m_timer_FSI.reset();

    m_timer_step.start();

    // Advance the dynamics of the fluid system
    m_timer_CFD.start();
    {
        double t = 0;
        while (t < step) {
            double h = std::min<>(m_step_CFD, step - t);
            if (h <= threshold_CFD)
                break;
            AdvanceFluidDynamics(timeCFD, h);
            t += h;
            timeCFD += h;
        }
    }
    m_timer_CFD.stop();

    // Apply fluid forces and torques on FSI solids
    m_timer_FSI.start();
    OnApplySolidForces();
    m_fsi_interface->ApplyBodyForces();
    m_fsi_interface->ApplyMeshForces();
    m_timer_FSI.stop();

    // Advance the dynamics of the multibody system
    m_timer_MBS.start();
    {
        double t = 0;
        while (t < step) {
            double h = std::min<>(m_step_MBD, step - t);
            if (h <= threshold_MBD)
                break;
            m_sysMBS->DoStepDynamics(h);
            t += h;
            timeMBD += h;
        }
    }
    m_timer_MBS.stop();

    m_time += step;

    // Load new solid phase states
    m_timer_FSI.start();
    m_fsi_interface->LoadBodyStates();
    m_fsi_interface->LoadMeshStates();
    OnLoadSolidStates();
    m_timer_FSI.stop();

    m_timer_step.stop();

    m_RTF = m_timer_step() / step;
    if (m_sysMBS)
        m_ratio_MBS = m_timer_MBS() / m_timer_step();
}

}  // end namespace fsi
}  // end namespace chrono
