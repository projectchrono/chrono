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

#ifndef CHNODEBASE_H
#define CHNODEBASE_H

#include "chrono/core/ChRunTimeType.h"
#include "chrono/lcp/ChLcpVariablesBodyOwnMass.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Class for a node, that has some degrees of 
/// freedom and that contain a proxy to the solver.
/// It is like a lightweight version of a ChPhysicsItem,
/// often a ChPhysicsItem is used as a container for a cluster 
/// of these ChNodeBase.

class ChApi ChNodeBase {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChMaterialSurfaceBase);

  protected:
	unsigned int offset_x; // offset in vector of state (position part)
	unsigned int offset_w; // offset in vector of state (speed part)

  public:
	ChNodeBase ();
	virtual ~ChNodeBase ();

	ChNodeBase (const ChNodeBase& other); // Copy constructor
	ChNodeBase& operator= (const ChNodeBase& other); //Assignment operator

					//
					// FUNCTIONS
					//


			//
			// Functions for interfacing to the state bookkeeping
			//

				/// Get the number of degrees of freedom
	virtual int Get_ndof_x() = 0; 
			
				/// Get the number of degrees of freedom, derivative
				/// This might be different from ndof if quaternions are used for rotations, 
				/// as derivative might be angular velocity.
	virtual int Get_ndof_w() { return this->Get_ndof_x(); }

				/// Get offset in the state vector (position part)
	unsigned int NodeGetOffset_x()	{ return this->offset_x; }
				/// Get offset in the state vector (speed part)
	unsigned int NodeGetOffset_w()	{ return this->offset_w; }

				/// Set offset in the state vector (position part)
	void NodeSetOffset_x(const unsigned int moff) { this->offset_x= moff; }
				/// Set offset in the state vector (speed part)
	void NodeSetOffset_w(const unsigned int moff) { this->offset_w = moff; }

	virtual void NodeIntStateGather(const unsigned int off_x,	ChState& x,	const unsigned int off_v, ChStateDelta& v,	double& T) {};	
	virtual void NodeIntStateScatter(const unsigned int off_x,	const ChState& x, const unsigned int off_v,	const ChStateDelta& v,	const double T) {};
	virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {};	
	virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {};
	virtual void NodeIntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x,	const unsigned int off_v, const ChStateDelta& Dv)
		{
				for (int i = 0; i< this->Get_ndof_x(); ++i)
				{
					x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
				}
		}
	virtual void NodeIntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c ) {};
	virtual void NodeIntLoadResidual_Mv(const unsigned int off,	ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) {};
	virtual void NodeIntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R) {};
	virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v) {};

			//
			// Functions for interfacing to the LCP solver
			//

	            /// Tell to a system descriptor that there are variables of type
                /// ChLcpVariables in this object (for further passing it to a LCP solver)
	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor) {};

				/// Sets the 'fb' part (the known term) of the encapsulated ChLcpVariables to zero.
	virtual void VariablesFbReset() {}

				/// Adds the current forces (applied to node) into the
				/// encapsulated ChLcpVariables, in the 'fb' part: qf+=forces*factor
	virtual void VariablesFbLoadForces(double factor=1.) {};

				/// Initialize the 'qb' part of the ChLcpVariables with the 
				/// current value of speeds. 
	virtual void VariablesQbLoadSpeed() {};

				/// Adds M*q (masses multiplied current 'qb') to Fb, ex. if qb is initialized
				/// with v_old using VariablesQbLoadSpeed, this method can be used in 
				/// timestepping schemes that do: M*v_new = M*v_old + forces*dt
	virtual void VariablesFbIncrementMq() {};

				/// Fetches the item speed (ex. linear velocity, in xyz nodes) from the
				/// 'qb' part of the ChLcpVariables and sets it as the current item speed.
				/// If 'step' is not 0, also should compute the approximate acceleration of
				/// the item using backward differences, that is  accel=(new_speed-old_speed)/step.
				/// Mostly used after the LCP provided the solution in ChLcpVariables.
	virtual void VariablesQbSetSpeed(double step=0.) {};

				/// Increment node positions by the 'qb' part of the ChLcpVariables,
				/// multiplied by a 'step' factor.
				///     pos+=qb*step
				/// If qb is a speed, this behaves like a single step of 1-st order
				/// numerical integration (Eulero integration).
	virtual void VariablesQbIncrementPosition(double step) {};


    // SERIALIZATION

    virtual void ArchiveOUT(ChArchiveOut& marchive);
    virtual void ArchiveIN(ChArchiveIn& marchive);
};



} // END_OF_NAMESPACE____


#endif
