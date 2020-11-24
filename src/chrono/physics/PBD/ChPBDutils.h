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

#ifndef CH_PBD_UTILS_H
#define CH_PBD_UTILS_H

#include "chrono/physics/ChSystem.h"

namespace chrono {
//enum PBD_DOF {ALL, PARTIAL, NONE };
//enum RotDOF {ALL, PARTIAL, NONE };
/// Struct collecting a Chrono ChLink together with additional info needed by 
struct ChApi ChLinkPBD {
	// Objects needed by PBD link
	std::shared_ptr<ChLink> link;
	// Relative Position of the link w.r.t. body 1 & 2 respectively
	ChFrame<> f1;
	ChFrame<> f2;
	// constrained DOF
	bool mask[6] = {};
	//PBD_DOF p_dof;
	//PBD_DOF r_dof;
	// By element-wise multiplication these vectors constrain only along the locked directions
	ChVector<> p_dir = ();
	ChVector<> r_dir = ();
	// Skip the whole correction if the pos/rot not constrained at all
	bool p_free = false;
	bool r_free = false;
	// Lagrangian
	double lambda = 0;
	// Compliance (TODO)
	double alpha;
	//TODO: not implementing limits and actuators yet.
	bool is_limited = false;
	double lims[6] = {};
	bool is_actuated = false;
	
	/// Create a LinkPBD
	ChLinkPBD(std::shared_ptr<ChLink> link);

    /// Copy constructor
	//ChLinkPBD(const ChLinkPBD& other);

    /// Destructor
    //virtual ~ChLinkPBD() {}

	/// Correct position to respect constraint
	void SolvePositions();

    // SERIALIZATION
	/* TODO
    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
	*/
};

CH_CLASS_VERSION(ChLinkPBD, 0)

}  // end namespace chrono

#endif
