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
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"

namespace chrono {
//enum PBD_DOF {ALL, PARTIAL, NONE };
enum RotDOF {X, Y, Z, NONE};
/// Struct collecting a Chrono ChLink together with additional info needed by 
class ChApi ChLinkPBD {
  public:

	/// Create a LinkPBD
	ChLinkPBD();

	/// Copy constructor
	//ChLinkPBD(const ChLinkPBD& other);

	/// Destructor
	virtual ~ChLinkPBD() {}
	// Objects needed by PBD link
	ChBody* Body1;
	ChBody* Body2;
	// Relative Position of the link w.r.t. body 1 & 2 respectively
	ChFrame<double> f1;
	ChFrame<double> f2;
	// constrained DOF
	bool mask[6] = {};
	// By element-wise multiplication these vectors constrain only along the locked directions
	ChVector<double> p_dir;
	ChVector<double> r_dir;
	// Skip the whole correction if the pos/rot not constrained at all
	bool p_free;
	bool r_free;
	// Rotational DOF
	RotDOF r_dof = NONE;
	ChVector<> a = VNULL;
	// Lagrangian of force and torque
	double lambda_f = 0;
	double lambda_t = 0;
	// Compliance (TODO: make it settable, and separate for torque and force)
	double alpha = 0.0001;
	//TODO: not implementing limits and actuators yet.
	bool is_limited = false;
	double lims[6] = {};
	bool is_actuated = false;
	/// Correct position to respect constraint
	void SolvePositions();

	/// evaluate the quaternion correction depending on rot DOF
	ChVector<> getQdelta();

	/// evaluate the quaternion correction depending on rot DOF
	void findRDOF();

    // SERIALIZATION
	/* TODO
    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
	*/
};

CH_CLASS_VERSION(ChLinkPBD, 0)

class ChApi ChLinkPBDLock : public ChLinkPBD {
  public:
	ChLinkLock* link;
	/// Create a LinkPBD
	ChLinkPBDLock(ChLinkLock* alink);

	/// Copy constructor
	//ChLinkPBD(const ChLinkPBD& other);

	/// Destructor
	virtual ~ChLinkPBDLock() {};
};

CH_CLASS_VERSION(ChLinkPBDLock, 0);


class ChApi ChLinkPBDMate : public ChLinkPBD {
  public:
	ChLinkMateGeneric* MGlink;
	/// Create a LinkPBD
	ChLinkPBDMate(ChLinkMateGeneric* alink);

	/// Copy constructor
	//ChLinkPBD(const ChLinkPBD& other);

	/// Destructor
	virtual ~ChLinkPBDMate() {};
	

};

CH_CLASS_VERSION(ChLinkPBDMate, 0)
/*
/// PBD method timesteppers.
class ChApi ChTimestepperPBD : public ChTimestepperIorder {
protected:
	// In base class
	//ChState Y;
	//ChStateDelta dYdt;


public:
	/// Constructor
	ChTimestepperPBD(ChIntegrable* intgr = nullptr) : ChTimestepper(intgr) { SetIntegrable(intgr); }

	/// Destructor
	virtual ~ChTimestepperIorder() {}

	/// Access the state at current time
	virtual ChState& get_Y() { return Y; }

	/// Access the derivative of state at current time
	virtual ChStateDelta& get_dYdt() { return dYdt; }

	/// Set the integrable object
	virtual void SetIntegrable(ChSystemPBD* intgr) {
		ChTimestepper::SetIntegrable(intgr);
		Y.setZero(1, intgr);
		dYdt.setZero(1, intgr);
	}
};
*/
}  // end namespace chrono

#endif
