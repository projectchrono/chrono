// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include "chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

class ChAAPlaneCB : public ChSystem::CustomCollisionCallback {
  public:
    ChAAPlaneCB(ChSystemDistributed* sys,  ///< Main system pointer
              ChBody* body,              ///< Associate body
              int const_coord,           ///< Coordinate which is kept constant (x=0, y=1, z=2)
              double const_coord_val,    ///< Value of constant coordinate
              ChVector<> inward_normal,   ///< Inward normal
              double min_1,              ///< Min value along lowest non-const axis
              double max_1,              ///< Max value along lowest non-const axis
              double min_2,              ///< Min value along highest non-const axis
              double max_2               ///< Max value along highest non-const axis
              )
        : m_sys(sys),
          m_body(body),
          m_const_coord(const_coord),
          m_const_coord_val(const_coord_val),
          m_inward_normal(inward_normal) {
        switch (m_const_coord) {
            case 0:
                m_nonconst_coords[0] = 1;
                m_nonconst_coords[1] = 2;
                break;
            case 1:
                m_nonconst_coords[0] = 0;
                m_nonconst_coords[1] = 2;
                break;
            case 2:
                m_nonconst_coords[0] = 0;
                m_nonconst_coords[1] = 1;
                break;
            default:
				break;
                // TODO yell
        }

        m_nonconst_min[0] = min_1;
        m_nonconst_min[1] = min_2;
        m_nonconst_max[0] = max_1;
        m_nonconst_max[1] = max_2;
    }

    void SetPos(double new_const_coord) { m_const_coord_val = new_const_coord; }

    // Main method called by the system at each collision detection phase
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    // Checks for collision between the plane and a sphere
    void CheckSphereProfile(std::shared_ptr<ChBody> sphere);
    ChSystemDistributed* m_sys;
    ChBody* m_body;

    int m_const_coord;
    double m_const_coord_val;
    int m_nonconst_coords[2];
    double m_nonconst_max[2];
    double m_nonconst_min[2];
    ChVector<> m_inward_normal;
};

void ChAAPlaneCB::OnCustomCollision(ChSystem* sys) {
    // Loop over all bodies in the system
    for (int i = 0; i < m_sys->data_manager->body_list->size(); i++) {
        auto sphere = (*m_sys->data_manager->body_list)[i];
        // TODO switch on shape type to be added later
		// Avoid colliding with other planes
		if ((std::dynamic_pointer_cast<collision::ChCollisionModelParallel>(sphere->GetCollisionModel()))->GetNObjects() > 0)
        	CheckSphereProfile(sphere);
    }
}

// Check for collision between a sphere shape and the plane
// Add a new contact to the associated system if there is contact
void ChAAPlaneCB::CheckSphereProfile(std::shared_ptr<ChBody> sphere) {
    // Mini broad-phase
    ChVector<> centerS(sphere->GetPos());
	auto pmodel = std::dynamic_pointer_cast<collision::ChCollisionModelParallel>(sphere->GetCollisionModel());
    double radius = pmodel->mData[0].B[0];
    if (centerS[m_nonconst_coords[0]] + radius < m_nonconst_min[0] ||
        centerS[m_nonconst_coords[0]] - radius > m_nonconst_max[0] ||
        centerS[m_nonconst_coords[1]] + radius < m_nonconst_min[1] ||
        centerS[m_nonconst_coords[1]] - radius > m_nonconst_max[1] ||
        std::abs(centerS[m_const_coord] - m_const_coord_val) > radius)
        return;

    ChVector<> normal = m_inward_normal / m_inward_normal.Length();

    ChVector<> plane_point = ChVector<>(0);
    plane_point[m_const_coord] = m_const_coord_val;
    plane_point[m_nonconst_coords[0]] = m_nonconst_min[0];
    plane_point[m_nonconst_coords[1]] = m_nonconst_min[1];

    ChVector<> delta = normal.Dot(centerS - plane_point);

    // Contact point on the plane
    ChVector<> vpA = ChVector<>(0);
    vpA[m_const_coord] = m_const_coord_val;
    vpA[m_nonconst_coords[0]] = centerS[m_nonconst_coords[0]];
    vpA[m_nonconst_coords[1]] = centerS[m_nonconst_coords[1]];

    // Contact point on sphere
    ChVector<> vpB = centerS - radius * normal;

    // Fill in contact information and add the contact to the system.
    // Express all vectors in the global frame
    collision::ChCollisionInfo contact;
    contact.modelA = m_body->GetCollisionModel().get();
    contact.modelB = sphere->GetCollisionModel().get();
    contact.vN = normal;
    contact.vpA = vpA;
    contact.vpB = vpB;
    contact.distance = delta.Length() - radius;

	m_sys->data_manager->host_data.erad_rigid_rigid.push_back(std::abs(delta.Length() - radius));
	
    m_sys->GetContactContainer()->AddContact(contact);  // NOTE: Not thread-safe
}

}  // end namespace chrono