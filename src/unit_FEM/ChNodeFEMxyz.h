//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHNODEFEMXYZ_H
#define CHNODEFEMXYZ_H

//////////////////////////////////////////////////
//
//   ChNodeFEMxyz.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChNodeFEMbase.h"


namespace chrono
{
namespace fem
{


/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement. This is the typical
/// node that can be used for tetahedrons, etc.

class ChApiFem ChNodeFEMxyz : public ChNodeFEMbase

{
private:
	ChLcpVariablesNode	variables; /// 3D node variables, with x,y,z

	ChVector<> X0;		///< reference position
	ChVector<> Force;	///< applied force
	
	double mass;

public:

	ChNodeFEMxyz(ChVector<> initial_pos = VNULL)
					{
						X0 = initial_pos;
						pos = initial_pos;
						Force = VNULL;
						mass = 1.0;
					}

	~ChNodeFEMxyz() {};


	virtual ChLcpVariables& Variables()
					{
						return this->variables; 
					} 

	virtual void Relax () 
					{
						X0 = this->pos; this->pos_dt=VNULL; this->pos_dtdt=VNULL; 
					}

			//
			// Functions for interfacing to the LCP solver
			//

	virtual void VariablesFbLoadForces(double factor=1.) 
					{ 
						this->variables.Get_fb().PasteSumVector( this->Force * factor ,0,0);
					};

	virtual void VariablesQbLoadSpeed() 
					{ 
						this->variables.Get_qb().PasteVector(this->pos_dt,0,0); 
					};

	virtual void VariablesQbSetSpeed(double step=0.) 
					{
						ChVector<> old_dt = this->pos_dt;
						this->SetPos_dt( this->variables.Get_qb().ClipVector(0,0) );
						if (step)
						{
							this->SetPos_dtdt( (this->pos_dt - old_dt)  / step);
						}
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetPos( this->GetPos() + newspeed * step);
					};

			//
			// Custom properties functions
			//
				/// Get mass of the node.
	virtual double GetMass() const {return this->variables.GetNodeMass();}
				/// Set mass of the node.
	virtual void SetMass(double mm) {this->variables.SetNodeMass(mm);}


				/// Set the initial (reference) position
	virtual void SetX0(ChVector<> mx) { X0 = mx;}
				/// Get the initial (reference) position
	virtual ChVector<> GetX0 () {return X0;}

				/// Set the 3d applied force, in absolute reference
	virtual void SetForce(ChVector<> mf) { Force = mf;}
				/// Get the 3d applied force, in absolute reference
	virtual ChVector<> GetForce () {return Force;}

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






