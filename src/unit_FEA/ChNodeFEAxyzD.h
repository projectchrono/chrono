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
// File authors: Alessandro Tasora

#ifndef CHNODEFEAXYZD_H
#define CHNODEFEAXYZD_H


#include "ChNodeFEAxyz.h"
#include "lcp/ChLcpVariablesGenericDiagonalMass.h"


namespace chrono
{
namespace fea
{


/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement and a D direction,
/// (a gradient vector to be used in ANCF gradient deficient beams)

class  ChNodeFEAxyzD : public ChNodeFEAxyz

{
public:

	ChNodeFEAxyzD(ChVector<> initial_pos = VNULL, ChVector<> initial_dir = VECT_X) :
					ChNodeFEAxyz(initial_pos)
					{
						D = initial_dir;
						D_dt = VNULL;
						D_dtdt = VNULL;
						variables_D  = new ChLcpVariablesGenericDiagonalMass(3);
						variables_D->GetMassDiagonal().FillElem(0.0); // default: no atomic mass associated to fea node, the fea element will add mass matrix
					}

	~ChNodeFEAxyzD() {};

	ChNodeFEAxyzD (const ChNodeFEAxyzD& other) :
						ChNodeFEAxyz(other) 
	{
		(*this->variables_D) = (*other.variables_D);
		this->D = other.D;
		this->D_dt = other.D_dt;
		this->D_dtdt = other.D_dtdt;
	}

	ChNodeFEAxyzD& operator= (const ChNodeFEAxyzD& other)
	{
		if (&other == this) 
			return *this;

		ChNodeFEAxyz::operator=(other);

		this->D = other.D;
		this->D_dt = other.D_dt;
		this->D_dtdt = other.D_dtdt;
		(*this->variables_D) = (*other.variables_D);
		return *this;
	}
				/// Set the direction
	virtual void SetD(ChVector<> mD) { D = mD;}
				/// Get the direction
	virtual ChVector<> GetD () {return D;}

				/// Set the direction speed
	virtual void SetD_dt(ChVector<> mD) { D_dt = mD;}
				/// Get the direction speed
	virtual ChVector<> GetD_dt () {return D_dt;}

				/// Set the direction acceleration
	virtual void SetD_dtdt(ChVector<> mD) { D_dtdt = mD;}
				/// Get the direction acceleration
	virtual ChVector<> GetD_dtdt () {return D_dtdt;}



	virtual ChLcpVariables& Variables_D()
					{
						return *this->variables_D; 
					} 

				/// Reset to no speed and acceleration.
	virtual void SetNoSpeedNoAcceleration () 
					{
						ChNodeFEAxyz::SetNoSpeedNoAcceleration();

						this->D_dt=VNULL; 
						this->D_dtdt=VNULL; 
					}

				/// Get mass of the node.
	virtual ChVectorDynamic<>& GetMassDiagonal() {return this->variables_D->GetMassDiagonal();}



				/// Sets the 'fixed' state of the node. If true, it does not move
                /// respect to the absolute world, despite constraints, forces, etc.
	void SetFixed  (bool mev) 
					{ 
						variables.SetDisabled(mev);
						variables_D->SetDisabled(mev); 
					}
				/// Gets the 'fixed' state of the node.
    bool GetFixed()  {return variables_D->IsDisabled(); }

	void SetFixedPos  (bool mev) 
					{ 
						variables.SetDisabled(mev);
					}

	void SetFixedD  (bool mev) 
					{ 
						variables_D->SetDisabled(mev);
					}

				/// Get the number of degrees of freedom
	virtual int Get_ndof_x() { return 6;}



			//
			// Functions for interfacing to the state bookkeeping
			//

	virtual void NodeIntStateGather(const unsigned int off_x,	ChState& x,	const unsigned int off_v, ChStateDelta& v,	double& T)
	{
		x.PasteVector  (this->pos,  off_x, 0);
		x.PasteVector  (this->D,    off_x+3, 0);
		v.PasteVector  (this->pos_dt,   off_v, 0);
		v.PasteVector  (this->D_dt,     off_v+3, 0);
	}

	virtual void NodeIntStateScatter(const unsigned int off_x,	const ChState& x, const unsigned int off_v,	const ChStateDelta& v,	const double T)
	{
		this->SetPos     (x.ClipVector(off_x, 0));
		this->SetD       (x.ClipVector(off_x+3, 0));
		this->SetPos_dt  (v.ClipVector(off_v, 0));
		this->SetD_dt    (v.ClipVector(off_v+3, 0));
	}

	virtual void NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a)
	{
		a.PasteVector  (this->pos_dtdt,   off_a, 0);
		a.PasteVector  (this->D_dtdt,     off_a+3, 0);
	}

	virtual void NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a)
	{
		this->SetPos_dtdt   (a.ClipVector(off_a, 0));
		this->SetD_dtdt     (a.ClipVector(off_a+3, 0));
	}

	virtual void NodeIntStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x,	const unsigned int off_v, const ChStateDelta& Dv)
	{ 
		x_new(off_x+0) = x(off_x+0) + Dv(off_v+0);
		x_new(off_x+1) = x(off_x+1) + Dv(off_v+1);
		x_new(off_x+2) = x(off_x+2) + Dv(off_v+2);
		x_new(off_x+3) = x(off_x+3) + Dv(off_v+3);
		x_new(off_x+4) = x(off_x+4) + Dv(off_v+4);
		x_new(off_x+5) = x(off_x+5) + Dv(off_v+5);
	}

	virtual void NodeIntLoadResidual_F(const unsigned int off,	ChVectorDynamic<>& R, const double c )
	{
		R.PasteSumVector( this->Force * c , off, 0);
		R.PasteSumVector( VNULL ,			off+3, 0); // TODO something about applied nodal torque..
	}

	virtual void NodeIntLoadResidual_Mv(const unsigned int off,	ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c)
	{
		R(off+0) += c* GetMass() * w(off+0);
		R(off+1) += c* GetMass() * w(off+1);
		R(off+2) += c* GetMass() * w(off+2);
		R(off+3) += c* GetMassDiagonal()(0) * w(off+3); // unuseful? mass for D isalways zero..
		R(off+4) += c* GetMassDiagonal()(1) * w(off+4);
		R(off+5) += c* GetMassDiagonal()(2) * w(off+5);
	}

	virtual void NodeIntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R)
	{
		ChNodeFEAxyz::NodeIntToLCP(off_v, v, R);
		this->variables_D->Get_qb().PasteClippedMatrix(&v, off_v+3,0, 3,1, 0,0);
		this->variables_D->Get_fb().PasteClippedMatrix(&R, off_v+3,0, 3,1, 0,0);
	}

	virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v)
	{
		ChNodeFEAxyz::NodeIntFromLCP(off_v, v);
		v.PasteMatrix(&this->variables_D->Get_qb(), off_v+3, 0);
	}



			//
			// Functions for interfacing to the LCP solver
			//

	virtual void InjectVariables(ChLcpSystemDescriptor& mdescriptor)
					{   
						ChNodeFEAxyz::InjectVariables(mdescriptor);
						mdescriptor.InsertVariables(this->variables_D);
					};

	virtual void VariablesFbReset() 
					{ 
						ChNodeFEAxyz::VariablesFbReset();
						this->variables_D->Get_fb().FillElem(0.0); 
					};

	virtual void VariablesFbLoadForces(double factor=1.) 
					{ 
						ChNodeFEAxyz::VariablesFbLoadForces(factor);
						//this->variables_D->Get_fb().PasteSumVector( VNULL ,3,0); // TODO something related to inertia?
					};

	virtual void VariablesQbLoadSpeed() 
					{ 
						ChNodeFEAxyz::VariablesQbLoadSpeed();			
						this->variables_D->Get_qb().PasteVector(this->D_dt,3,0);
					};

	virtual void VariablesQbSetSpeed(double step=0.) 
					{
						ChNodeFEAxyz::VariablesQbSetSpeed(step);

						ChVector<> oldD_dt = this->D_dt;
						this->SetD_dt  ( this->variables_D->Get_qb().ClipVector(3,0) );
						if (step)
						{
							this->SetD_dtdt  ( (this->D_dt   - oldD_dt)  / step);
						}
					};

	virtual void VariablesFbIncrementMq() 
					{
						ChNodeFEAxyz::VariablesFbIncrementMq();
						this->variables_D->Compute_inc_Mb_v(this->variables_D->Get_fb(), this->variables_D->Get_qb());
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						ChNodeFEAxyz::VariablesQbIncrementPosition(step);

						ChVector<> newspeed_D = variables_D->Get_qb().ClipVector(3,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetD  ( this->GetD()   + newspeed_D * step);
					};

private:
	/// 3D node variable - the direction part: Dx,Dy,Dz (the position part is in parent class)
 	ChLcpVariablesGenericDiagonalMass*  variables_D; 

public:

	ChVector<> D;		
	ChVector<> D_dt;
	ChVector<> D_dtdt;


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






