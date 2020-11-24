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
// Authors: Simone Benatti
// =============================================================================
//
// Structures for links, contacts, body properties in PBD systems and their lists
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/PBD/ChPBDutils.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"

namespace chrono {

// Not efficient, but also colled only once at the beginning
template<typename Base, typename T>
inline bool instanceof(const T *ptr) {
	return dynamic_cast<const Base*>(ptr) != nullptr;
}
// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPBD)

ChLinkPBD::ChLinkPBD(std::shared_ptr<ChLink> alink) {
	link = alink;
	// Get info from instances of ChLinkMateGeneric
	if (instanceof<ChLinkMateGeneric>(link.get())) {
		ChLinkMateGeneric* mate = dynamic_cast<ChLinkMateGeneric*>(link.get());
		f1 = mate->GetFrame1();
		f2 = mate->GetFrame2();
		mask[0] = mate->IsConstrainedX();
		mask[1] = mate->IsConstrainedY();
		mask[2] = mate->IsConstrainedZ();
		mask[3] = mate->IsConstrainedRx();
		mask[4] = mate->IsConstrainedRy();
		mask[5] = mate->IsConstrainedRz();
	}
	else if (instanceof<ChLinkLock>(link.get())) {
		ChLinkLock* lock = dynamic_cast<ChLinkLock*>(link.get());
		f1 = lock->GetMarker1.GetCoord();
		f2 = lock->GetMarker2.GetCoord();
		ChLinkMask& p_mask = lock->GetMask();
		ChLinkMaskLF* m_lf = dynamic_cast<ChLinkMaskLF*>(&p_mask);
		mask[0] = m_lf->Constr_X().GetMode() == CONSTRAINT_LOCK;
		mask[1] = m_lf->Constr_Y().GetMode() == CONSTRAINT_LOCK;
		mask[2] = m_lf->Constr_Z().GetMode() == CONSTRAINT_LOCK;
		//mask[3] = m_lf->Constr_E0().GetMode() == CONSTRAINT_LOCK;
		mask[3] = m_lf->Constr_E1().GetMode() == CONSTRAINT_LOCK;
		mask[4] = m_lf->Constr_E2().GetMode() == CONSTRAINT_LOCK;
		mask[5] = m_lf->Constr_E3().GetMode() == CONSTRAINT_LOCK;
	}
	/*
	if (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) {
		p_dof = NONE;
	}
	else if (int(mask[0]) + int(mask[1]) + int(mask[2]) == 3) {
		p_dof = ALL;
	}
	else p_dof = PARTIAL;

	if (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) {
		r_dof = NONE;
	}
	else if (int(mask[3]) + int(mask[4]) + int(mask[5]) == 3) {
		r_dof = ALL;
	}
	else r_dof = PARTIAL;
	*/
	p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
	p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
	r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
}

void ChLinkPBD::SolvePositions() {
	if (!p_free) {

	}
}
/*
void ChSystemNSC::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystemNSC>();

    // serialize parent class
    ChSystem::ArchiveOUT(marchive);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void ChSystemNSC::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSystemNSC>();

    // deserialize parent class
    ChSystem::ArchiveIN(marchive);

    // stream in all member data:
}
*/
}  // end namespace chrono
