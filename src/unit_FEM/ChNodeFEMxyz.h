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
// File authors: Andrea Favali, Alessandro Tasora

#ifndef CHNODEFEMXYZ_H
#define CHNODEFEMXYZ_H


#include "ChNodeFEMbase.h"
#include "lcp/ChLcpVariablesNode.h"


namespace chrono
{
namespace fem
{


/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement. This is the typical
/// node that can be used for tetahedrons, etc.

class ChApiFem ChNodeFEMxyz : public ChNodeFEMbase

{
public:

	ChNodeFEMxyz(ChVector<> initial_pos = VNULL)
					{
						X0 = initial_pos;
						pos = initial_pos;
						Force = VNULL;
						variables.SetNodeMass(1.0);
					}

	~ChNodeFEMxyz() {};

	ChNodeFEMxyz::ChNodeFEMxyz (const ChNodeFEMxyz& other) :
						ChNodeFEMbase(other) 
	{
		this->X0 = other.X0;
		this->pos = other.pos;
		this->pos_dt = other.pos_dt;
		this->pos_dtdt = other.pos_dtdt;
		this->Force = other.Force;
		this->variables = other.variables;
	}

	ChNodeFEMxyz& ChNodeFEMxyz::operator= (const ChNodeFEMxyz& other)
	{
		if (&other == this) 
			return *this;

		ChNodeFEMbase::operator=(other);

		this->X0 = other.X0;
		this->pos = other.pos;
		this->pos_dt = other.pos_dt;
		this->pos_dtdt = other.pos_dtdt;
		this->Force = other.Force;
		this->variables = other.variables;
		return *this;
	}

	virtual ChLcpVariables& Variables()
					{
						return this->variables; 
					} 

				/// Set the rest position as the actual position.
	virtual void Relax () 
					{
						X0 = this->pos; this->pos_dt=VNULL; this->pos_dtdt=VNULL; 
					}

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

				/// Position of the node - in absolute csys.
	ChVector<> GetPos() {return pos;}
				/// Position of the node - in absolute csys.
	void SetPos(const ChVector<>& mpos) {pos = mpos;}

				/// Velocity of the node - in absolute csys.
	ChVector<> GetPos_dt() {return pos_dt;}
				/// Velocity of the node - in absolute csys.
	void SetPos_dt(const ChVector<>& mposdt) {pos_dt = mposdt;}

				/// Acceleration of the node - in absolute csys.
	ChVector<> GetPos_dtdt() {return pos_dtdt;}
				/// Acceleration of the node - in absolute csys.
	void SetPos_dtdt(const ChVector<>& mposdtdt) {pos_dtdt = mposdtdt;}



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

	virtual void VariablesFbIncrementMq() 
					{
						this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetPos( this->GetPos() + newspeed * step);
					};

private:
	ChLcpVariablesNode	variables; /// 3D node variables, with x,y,z

	ChVector<> X0;		///< reference position
	ChVector<> Force;	///< applied force
	
public:
	ChVector<> pos;		
	ChVector<> pos_dt;
	ChVector<> pos_dtdt;


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






