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

using std::cout;
using std::cerr;
using std::endl;

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

    // Initialize bounding boxes (in Y and Z directions only)
    int IsizeY = (int)std::round(box_size.y() / m_spacing);  // number of grid cells in Y direction
    int IsizeZ = (int)std::round(box_size.z() / m_spacing);  // number of grid cells in Z direction
    m_rearAABB = ChAABB(ChVector3d(0, -m_spacing / 2, -m_spacing / 2),                                            //
                        ChVector3d(0, +m_spacing / 2 + m_spacing * IsizeY, m_spacing / 2 + m_spacing * IsizeZ));  //
    m_frontAABB = ChAABB(ChVector3d(0, 0, 0),                                                                     //
                         ChVector3d(0, +m_spacing * IsizeY, m_spacing * IsizeZ));                                 //
    m_IfrontAABB = ChIntAABB(ChVector3i(0, 0, 0),                                                                 //
                             ChVector3i(0, +IsizeY, IsizeZ));                                                     //

    // Set the shift X length in number of grid cells
    m_Ishift = (int)std::round(shift_distance / m_spacing);

    // Set moving patch parameters
    m_Irear = 0;                                           // X grid coordinate of rear-most SPH particles
    m_Ifront = (int)std::round(box_size.x() / m_spacing);  // X grid coordinate of front-most SPH particles
    UpdateAABBs();                                         // source and destination AABBs

    // Create the SPH particles and boundary BCE markers (no top)
    Construct(box_size, ChVector3d(box_size.x() / 2, box_size.y() / 2, 0), BoxSide::ALL & ~BoxSide::Z_POS);

    // Create a particle relocator
    CreateParticleRelocator();
}

//
//     BCE           src AABB                                    dest AABB           BCE
//   _______    _____________________                       ___________________     _______
//  |       |  |                     |                     |                   |   |       |
//  |       |  |                     |                     |                   |   |       |
//                                        
//  x   x   x   o   o   o   o   o   o     ......   o   o   x   x   x   
//                          X   X   X                      O   O   O   O   O   O   X   X   X 
//                                        
//          |                       ^                      |                       ^  
//          |                       |                      |                       |
//           -----------------------                        ----------------------- 
//                   BCE shift                                      BCE shift
//
void CRMTerrain::Synchronize(double time) {
    m_moved = false;
    if (!m_moving_patch)
        return;

    // Check distance from monitored body to front boundary.
    double dist = m_spacing * m_Ifront - m_sentinel->GetFrameRefToAbs().GetPos().x();
    if (dist >= m_buffer)
        return;

    // Current computationl domain
    ChAABB domainAABB = m_sysSPH.GetComputationalDomain();

    if (m_verbose) {
        cout << "Move patch (" << dist << " < " << m_buffer << ") at t = " << time << endl;
        cout << "  Rear boundary:  " << m_Irear << endl;
        cout << "  Front boundary: " << m_Ifront << endl;
        cout << "  Shift:          " << m_Ishift << endl;
        cout << "  Spacing:        " << m_spacing << endl;
        cout << "  Src. AABB  x: " << m_rearAABB.min.x() << "  " << m_rearAABB.max.x() << endl;
        cout << "             y: " << m_rearAABB.min.y() << "  " << m_rearAABB.max.y() << endl;
        cout << "             z: " << m_rearAABB.min.z() << "  " << m_rearAABB.max.z() << endl;
        cout << "  Dest. AABB x: " << m_IfrontAABB.min.x() << "  " << m_IfrontAABB.max.x() << endl;
        cout << "             y: " << m_IfrontAABB.min.y() << "  " << m_IfrontAABB.max.y() << endl;
        cout << "             z: " << m_IfrontAABB.min.z() << "  " << m_IfrontAABB.max.z() << endl;

        ChVector3i dim = m_IfrontAABB.max - m_IfrontAABB.min;
        cout << "  Num dest slots: " << (dim.x() + 1) * (dim.y() + 1) * (dim.z() + 1) << endl;

        cout << "  Current computational domain: " << domainAABB.min << "  " << domainAABB.max << endl;
    }

    // Move patch (operate directly on device data)
    BCEShift(ChVector3d(m_spacing * (m_Ishift + 1), 0, 0));
    SPHMoveAABB2AABB(m_rearAABB, m_IfrontAABB);

    // Update moving patch parameters for next shift
    m_Irear += m_Ishift;   // X grid coordinate of rear-most SPH particles
    m_Ifront += m_Ishift;  // X grid coordinate of front-most SPH particles
    UpdateAABBs();         // source and destination AABBs

    // Update computational domain
    domainAABB.min.x() += m_spacing * (m_Ishift + 1);
    domainAABB.max.x() += m_spacing * (m_Ishift + 1);
    m_sysSPH.SetComputationalDomain(domainAABB);

    if (m_verbose) {
        cout << "  Updated computational domain: " << domainAABB.min << "  " << domainAABB.max << endl;
    }

    // Force proximity search at the beginning of next step
    ForceProximitySearch();

    m_moved = true;
}

void CRMTerrain::UpdateAABBs() {
    double rear = m_spacing * m_Irear;
    double front = m_spacing * m_Ifront;
    double shift = m_spacing * m_Ishift;

    m_rearAABB.min.x() = rear - m_spacing / 2;
    m_rearAABB.max.x() = rear + shift + m_spacing / 2;

    m_frontAABB.min.x() = front;
    m_frontAABB.max.x() = front + shift;

    m_IfrontAABB.min.x() = m_Ifront + 1;
    m_IfrontAABB.max.x() = m_Ifront + 1 + m_Ishift;
}

}  // end namespace vehicle
}  // end namespace chrono
