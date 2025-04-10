// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Continuum representation (SPH-based) deformable terrain model.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#define DEBUG_LOG

#include "chrono/utils/ChUtils.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

namespace chrono {

using namespace chrono::fsi;
using namespace chrono::fsi::sph;

namespace vehicle {

// -----------------------------------------------------------------------------

class VehicleAdvance : public fsi::ChFsiSystem::MBDCallback {
  public:
    VehicleAdvance(ChVehicle* vehicle) : m_vehicle(vehicle) {}
    virtual void Advance(double step, double threshold) override { m_vehicle->Advance(step); }
    ChVehicle* m_vehicle;
};

// -----------------------------------------------------------------------------

CRMTerrain::CRMTerrain(ChSystem& sys, double spacing)
    : ChFsiProblemCartesian(sys, spacing),
      m_moving_patch(false),
      m_moved(false),
      m_buffer(0),
      m_Ishift(0) {}

void CRMTerrain::SetActiveDomain(const ChVector3d& box_dim) {
    GetFluidSystemSPH().SetActiveDomain(box_dim);
}

void CRMTerrain::SetActiveDomainDelay(double delay) {
    GetFluidSystemSPH().SetActiveDomainDelay(delay);
}

void CRMTerrain::RegisterVehicle(ChVehicle* vehicle) {
    auto vehicle_advance_cb = chrono_types::make_shared<VehicleAdvance>(vehicle);
    m_sysFSI.RegisterMBDCallback(vehicle_advance_cb);
}

void CRMTerrain::Advance(double step) {
    DoStepDynamics(step);
}

// -----------------------------------------------------------------------------

void CRMTerrain::ConstructMovingPatch(const ChVector3d& box_size,
                                      std::shared_ptr<ChBody> body,
                                      double buffer_distance,
                                      double shift_distance) {
    m_sentinel = body;
    m_moving_patch = true;
    m_buffer = buffer_distance;

    m_Irear = 0;
    m_Ifront = (int)std::round(box_size.x() / m_spacing);
    m_Ishift = (int)std::round(shift_distance / m_spacing);
    int IsizeY = (int)std::round(box_size.y() / m_spacing);
    int IsizeZ = (int)std::round(box_size.z() / m_spacing);

    // Initialize bounding boxes (Y and Z directions)
    m_rearAABB = ChAABB(ChVector3d(0, -m_spacing / 2, -m_spacing / 2),                                            //
                        ChVector3d(0, +m_spacing / 2 + m_spacing * IsizeY, m_spacing / 2 + m_spacing * IsizeZ));  //
    m_frontAABB = ChAABB(ChVector3d(0, 0, 0),                                                                     //
                         ChVector3d(0, +m_spacing * IsizeY, m_spacing * IsizeZ));                                 //
    m_IfrontAABB = ChIntAABB(ChVector3i(0, 0, 0),                                                                 //
                             ChVector3i(0, +IsizeY, IsizeZ));                                                     //

    // Set X extents of bounding boxes
    UpdateAABB();

    std::cout << "Moving patch initialization" << std::endl;
    std::cout << "Rear boundary:  " << m_Irear << std::endl;
    std::cout << "Front boundary: " << m_Ifront << std::endl;
    std::cout << "Shift:          " << m_Ishift << std::endl;
    std::cout << "Spacing:        " << m_spacing << std::endl;

    std::cout << "RearAABB: " << m_rearAABB.min << " " << m_rearAABB.max << std::endl;

    // Create the SPH particles and boundary BCE markers (no top)
    Construct(box_size, ChVector3d(box_size.x() / 2, box_size.y() / 2, 0), BoxSide::ALL & ~BoxSide::Z_POS);
}

void CRMTerrain::Synchronize(double time) {
    m_moved = false;
    if (!m_moving_patch)
        return;

    // Check distance from monitored body to front boundary.
    double dist = m_spacing * m_Ifront - m_sentinel->GetFrameRefToAbs().GetPos().x();
    if (dist >= m_buffer)
        return;

    ChDebugLog("Move patch (" << dist << " < " << m_buffer << ")   shift: " << m_Ishift << "x" << m_spacing);

    // Move patch (operate directly on device data)
    BCEShift(ChVector3d(m_spacing * m_Ishift, 0, 0));
    ////SPHMoveAABB2AABB(m_rearAABB, m_frontAABB);
    SPHMoveAABB2AABB(m_rearAABB, m_IfrontAABB);

    m_Irear += m_Ishift;
    m_Ifront += m_Ishift;

    UpdateAABB();

    m_moved = true;
}

void CRMTerrain::UpdateAABB() {
    double rear = m_spacing * m_Irear;
    double front = m_spacing * m_Ifront;
    double shift = m_spacing * m_Ishift;

    m_rearAABB.min.x() = rear - m_spacing / 2;
    m_rearAABB.max.x() = rear + shift - m_spacing / 2;

    m_frontAABB.min.x() = front;
    m_frontAABB.max.x() = front + shift;

    m_IfrontAABB.min.x() = m_Ifront;
    m_IfrontAABB.max.x() = m_Ifront + m_Ishift;
}

}  // end namespace vehicle
}  // end namespace chrono
