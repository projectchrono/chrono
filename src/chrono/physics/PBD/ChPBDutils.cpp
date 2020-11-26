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


namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPBD)
ChLinkPBD::ChLinkPBD() : p_dir(ChVector<double>(0, 0, 0)), r_dir(ChVector<double>(0, 0, 0)), p_free(false), r_free(false) {}

ChLinkPBDLock::ChLinkPBDLock(ChLinkLock* alink) : ChLinkPBD() {
	link = alink;
	f1 = ChFrame<>(link->GetMarker1()->GetCoord());
	f2 = ChFrame<>(link->GetMarker2()->GetCoord());
	ChLinkMask& p_mask = link->GetMask();
	ChLinkMaskLF* m_lf = dynamic_cast<ChLinkMaskLF*>(&p_mask);
	mask[0] = m_lf->Constr_X().GetMode() == CONSTRAINT_LOCK;
	mask[1] = m_lf->Constr_Y().GetMode() == CONSTRAINT_LOCK;
	mask[2] = m_lf->Constr_Z().GetMode() == CONSTRAINT_LOCK;
	//mask[3] = m_lf->Constr_E0().GetMode() == CONSTRAINT_LOCK;
	mask[3] = m_lf->Constr_E1().GetMode() == CONSTRAINT_LOCK;
	mask[4] = m_lf->Constr_E2().GetMode() == CONSTRAINT_LOCK;
	mask[5] = m_lf->Constr_E3().GetMode() == CONSTRAINT_LOCK;
	p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
	p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
	r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
}



ChLinkPBDMate::ChLinkPBDMate(ChLinkMateGeneric* alink) : ChLinkPBD() {
	MGlink = alink;
	f1 = ChFrame<double>(MGlink->GetFrame1());
	f2 = ChFrame<double>(MGlink->GetFrame2());
	mask[0] = MGlink->IsConstrainedX();
	mask[1] = MGlink->IsConstrainedY();
	mask[2] = MGlink->IsConstrainedZ();
	mask[3] = MGlink->IsConstrainedRx();
	mask[4] = MGlink->IsConstrainedRy();
	mask[5] = MGlink->IsConstrainedRz();
	p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
	p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
	r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
}

/*ChLinkPBD::ChLinkPBD(const ChLinkPBD& other) :  p_dir(ChVector<double>(0, 0, 0)), r_dir(ChVector<double>(0, 0, 0)), f1(ChFrame<double>(VNULL)), f2(ChFrame<double>(VNULL)), p_free(false), r_free(false) {
	ChLinkPBD(other.link);
}*/

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
