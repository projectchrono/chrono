//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHNODEXYZ_H
#define CHNODEXYZ_H

#include "physics/ChNodeBase.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpVariablesNode.h"

namespace chrono {

/// Class for a single 'point' node, that has 3 DOF degrees of
/// freedom and a mass.

class ChApi ChNodeXYZ : public ChNodeBase,  public ChContactable_1vars<3> {
  public:
    ChNodeXYZ();
    virtual ~ChNodeXYZ();

    ChNodeXYZ(const ChNodeXYZ& other);             // Copy constructor
    ChNodeXYZ& operator=(const ChNodeXYZ& other);  // Assignment operator

    //
    // FUNCTIONS
    //

			// Access the xyz 'LCP variables' of the node
	virtual ChLcpVariablesNode& Variables() =0;

			// Position of the node - in absolute csys.
	ChVector<> GetPos() {return pos;}
			// Position of the node - in absolute csys.
	void SetPos(const ChVector<>& mpos) {pos = mpos;}

    // Velocity of the node - in absolute csys.
    ChVector<> GetPos_dt() { return pos_dt; }
    // Velocity of the node - in absolute csys.
    void SetPos_dt(const ChVector<>& mposdt) { pos_dt = mposdt; }

    // Acceleration of the node - in absolute csys.
    ChVector<> GetPos_dtdt() { return pos_dtdt; }
    // Acceleration of the node - in absolute csys.
    void SetPos_dtdt(const ChVector<>& mposdtdt) { pos_dtdt = mposdtdt; }

    // Get mass of the node. To be implemented in children classes
    virtual double GetMass() const = 0;
    // Set mass of the node. To be implemented in children classes
    virtual void SetMass(double mm) = 0;

    /// Get the number of degrees of freedom
    virtual int Get_ndof_x() { return 3; }


    //
    // INTERFACE TO ChContactable
    //

        /// Access variables
    virtual ChLcpVariables* GetVariables1() {return &Variables(); }

        /// Tell if the object must be considered in collision detection
    virtual bool IsContactActive() { return true; }

        /// Get the absolute speed of point abs_point if attached to the 
        /// surface. Easy in this case because there are no roations..
    virtual ChVector<> GetContactPointSpeed(const ChVector<>& abs_point) {return this->pos_dt;};

        /// ChCollisionModel might call this to get the position of the 
        /// contact model (when rigid) and sync it
    virtual ChCoordsys<> GetCsysForCollisionModel() {return ChCoordsys<>(this->pos, QNULL);}

        /// Apply the force, expressed in absolute reference, applied in pos, to the 
        /// coordinates of the variables. Force for example could come from a penalty model.
    virtual void ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, 
                            const unsigned int off, ChVectorDynamic<>& R, const double c);

        /// Compute the jacobian(s) part(s) for this contactable item. For example,
        /// if the contactable is a ChBody, this should update the corresponding 1x6 jacobian.
    virtual void ComputeJacobianForContactPart(const ChVector<>& abs_point, ChMatrix33<>& contact_plane, 
                            type_constraint_tuple& jacobian_tuple_N,
                            type_constraint_tuple& jacobian_tuple_U,
                            type_constraint_tuple& jacobian_tuple_V,
                            bool second);


    //
    // DATA
    //
    ChVector<> pos;
    ChVector<> pos_dt;
    ChVector<> pos_dtdt;
};

}  // END_OF_NAMESPACE____

#endif
