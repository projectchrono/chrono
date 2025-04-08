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

////#define DEBUG_LOG

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
      m_buffer_dist(0),
      m_shift_dist(0) {}

void CRMTerrain::SetActiveDomain(const ChVector3d& half_dim) {
    GetFluidSystemSPH().SetActiveDomain(half_dim);
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
    m_buffer_dist = buffer_distance;
    m_shift_dist = shift_distance;
    m_rear = 0;
    m_front = box_size.x();

    m_rearAABB = ChAABB(ChVector3d(-m_spacing + m_rear, -m_spacing - box_size.y(), -m_spacing),                    //
                        ChVector3d(m_rear + m_shift_dist, +m_spacing + box_size.y(), +m_spacing + box_size.z()));  //
    m_frontAABB = ChAABB(ChVector3d(m_front, -box_size.y(), 0),                                                    //
                         ChVector3d(m_front + m_shift_dist, +box_size.y(), box_size.z()));                         //

    // m_rear_cells:  current location of inner-most rear BCE X layer
    // m_front_cells: current location of inner-most front BCE X layer 
    // m_shift_cells: number of relocated SPH X layers 
    m_rear_cells = 0;
    m_front_cells = std::round(m_front / m_spacing) + 1;
    m_shift_cells = std::round(m_shift_dist / m_spacing);

    // Create the SPH particles and boundary BCE markers (no top)
    Construct(box_size, ChVector3d(box_size.x() / 2, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);
}

void CRMTerrain::Synchronize(double time) {
    m_moved = false;
    if (!m_moving_patch)
        return;

    // Check distance from monitored body to front boundary.
    double dist = m_front - m_sentinel->GetFrameRefToAbs().GetPos().x();
    if (dist >= m_buffer_dist)
        return;

    ChDebugLog("Move patch (" << dist << " < " << m_buffer_dist << ")   shift: " << m_shift_dist);

    // Move patch (operate directly on device data)
    BCEShift(ChVector3d(m_shift_dist, 0, 0));
    SPHMoveAABB(m_rearAABB, m_frontAABB);

    m_rear += m_shift_dist;
    m_front += m_shift_dist;
    m_rearAABB.min.x() = -m_spacing + m_rear;
    m_rearAABB.max.x() = m_rear + m_shift_dist;
    m_frontAABB.min.x() = m_front;
    m_frontAABB.max.x() = m_front + m_shift_dist;

    m_moved = true;
}

}  // end namespace vehicle
}  // end namespace chrono
