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
// This inherits from a NSC ContactContainer but it makes the lists accessible
//
// =============================================================================

#ifndef CH_CONTACTCONTAINER_PBD_H
#define CH_CONTACTCONTAINER_PBD_H

#include <list>

#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChContactNSC.h"
#include "chrono/physics/ChContactNSCrolling.h"
#include "chrono/physics/ChContactable.h"

namespace chrono {

/// Class representing a container of many non-smooth contacts.
/// Implemented using linked lists of ChContactNSC objects (that is, contacts between two ChContactable objects, with 3
/// reactions). It might also contain ChContactNSCrolling objects (extended versions of ChContactNSC, with 6 reactions,
/// that account also for rolling and spinning resistance), but also for '6dof vs 6dof' contactables.
class ChApi ChContactContainerPBD : public ChContactContainerNSC {

  public:
	ChContactContainerPBD() : ChContactContainerNSC() {};
    ChContactContainerPBD(const ChContactContainerPBD& other) : ChContactContainerNSC() {};

    virtual ~ChContactContainerPBD() {
		RemoveAllContacts();
	}


    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerPBD* Clone() const override { return new ChContactContainerPBD(*this); }

    /// Report the number of added contacts.
  private:
	// PBD solver must access contact tuples
	std::list<ChContactNSC_6_6*>& Get_6_6_clist() {
		return contactlist_6_6;
    }
	std::list<ChContactNSC_6_3*>& Get_6_3_clist() {
		return contactlist_6_3;
	}
	std::list<ChContactNSC_3_3*>& Get_3_3_clist() {
		return contactlist_3_3;
	}
	std::list<ChContactNSC_333_3*>& Get_333_3_clist() {
		return contactlist_333_3;
	}
	std::list<ChContactNSC_333_6*>& Get_333_6_clist() {
		return contactlist_333_6;
	}
	std::list<ChContactNSC_333_333*>& Get_333_333_clist() {
		return contactlist_333_333;
	}
	std::list<ChContactNSC_666_3*>& Get_666_3_clist() {
		return contactlist_666_3;
	}
	std::list<ChContactNSC_666_6*>& Get_666_6_clist() {
		return contactlist_666_6;
	}
	std::list<ChContactNSC_666_333*>& Get_666_333_clist() {
		return contactlist_666_333;
	}
	std::list<ChContactNSC_666_666*>& Get_666_666_clist() {
		return contactlist_666_666;
	}
	friend class ChSystemPBD;

};

CH_CLASS_VERSION(ChContactContainerPBD, 0)

}  // end namespace chrono

#endif
