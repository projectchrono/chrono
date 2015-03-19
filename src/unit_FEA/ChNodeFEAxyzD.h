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


#include "ChNodeFEAbase.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"


namespace chrono
{
namespace fea
{


/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement and a D direction,
/// (a gradient vector to be used in ANCF gradient deficient beams)

class  ChNodeFEAxyzD : public ChNodeFEAbase

{
public:

	ChNodeFEAxyzD(ChVector<> initial_pos = VNULL, ChVector<> initial_dir = VECT_X)
					{
						X0 = initial_pos;
						pos = initial_pos;
						Force = VNULL;
						D = initial_dir;
						D_dt = VNULL;
						D_dtdt = VNULL;
						variables.SetBodyMass(0.0);
						variables.SetBodyInertia(ChMatrix33<>(0)); //***TODO*** tensor of inertia not needed, ChLcpVariablesBodyOwnMass is a temporary hack that should be promoted to some other ChLcpVariablesXXX stuff
					}

	~ChNodeFEAxyzD() {};

	ChNodeFEAxyzD (const ChNodeFEAxyzD& other) :
						ChNodeFEAbase(other) 
	{
		this->X0 = other.X0;
		this->pos = other.pos;
		this->pos_dt = other.pos_dt;
		this->pos_dtdt = other.pos_dtdt;
		this->Force = other.Force;
		this->variables = other.variables;
		this->D = other.D;
		this->D_dt = other.D_dt;
		this->D_dtdt = other.D_dtdt;
	}

	ChNodeFEAxyzD& operator= (const ChNodeFEAxyzD& other)
	{
		if (&other == this) 
			return *this;

		ChNodeFEAbase::operator=(other);

		this->X0 = other.X0;
		this->pos = other.pos;
		this->pos_dt = other.pos_dt;
		this->pos_dtdt = other.pos_dtdt;
		this->D = other.D;
		this->D_dt = other.D_dt;
		this->D_dtdt = other.D_dtdt;
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
						X0 = this->pos; 
						this->SetNoSpeedNoAcceleration(); 
					}

				/// Reset to no speed and acceleration.
	virtual void SetNoSpeedNoAcceleration () 
					{
						this->pos_dt=VNULL; 
						this->pos_dtdt=VNULL; 
						this->D_dt=VNULL; 
						this->D_dtdt=VNULL; 
					}

				/// Get mass of the node.
	virtual double GetMass() {return this->variables.GetBodyMass();}
				/// Set mass of the node.
	virtual void SetMass(double mm) {this->variables.SetBodyMass(mm);}

				/// Get inertia (for D vector coord!) of the node.
	//virtual  ChMatrix33<>& GetInertia() const {return this->variables.GetBodyInertia();}
				/// Get inertia (for D vector coord!) of the node
	//virtual void SetInertia(ChMatrix33<>& mm) {this->variables.SetBodyInertia(mm);}


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

				/// Direction of the node - in absolute csys.
	ChVector<> GetD() {return D;}
				/// Direction of the node - in absolute csys.
	void SetD(const ChVector<>& mD) {D = mD;}

				/// Velocity of the node - in absolute csys.
	ChVector<> GetD_dt() {return D_dt;}
				/// Velocity of the node - in absolute csys.
	void SetD_dt(const ChVector<>& mDdt) {D_dt = mDdt;}

				/// Acceleration of the node - in absolute csys.
	ChVector<> GetD_dtdt() {return D_dtdt;}
				/// Acceleration of the node - in absolute csys.
	void SetD_dtdt(const ChVector<>& mDdtdt) {D_dtdt = mDdtdt;}


				/// Sets the 'fixed' state of the node. If true, it does not move
                /// respect to the absolute world, despite constraints, forces, etc.
	void SetFixed  (bool mev) { variables.SetDisabled(mev); }
				/// Gets the 'fixed' state of the node.
    bool GetFixed()  {return variables.IsDisabled(); }


				/// Get the number of degrees of freedom
	virtual int Get_ndof_x() { return 6;}


			//
			// Functions for interfacing to the state bookkeeping
			//

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
		//R(off+3) += c* GetInertia() * w(off+3); // no rotational inertia effect
		//R(off+4) += c* GetInertia() * w(off+4);
		//R(off+5) += c* GetInertia() * w(off+5);
	}

	virtual void NodeIntToLCP(const unsigned int off_v,	const ChStateDelta& v, const ChVectorDynamic<>& R)
	{
		this->variables.Get_qb().PasteClippedMatrix(&v, off_v,0, 6,1, 0,0);
		this->variables.Get_fb().PasteClippedMatrix(&R, off_v,0, 6,1, 0,0);
	}

	virtual void NodeIntFromLCP(const unsigned int off_v, ChStateDelta& v)
	{
		v.PasteMatrix(&this->variables.Get_qb(), off_v, 0);
	}



			//
			// Functions for interfacing to the LCP solver
			//

	virtual void VariablesFbLoadForces(double factor=1.) 
					{ 
						this->variables.Get_fb().PasteSumVector( this->Force * factor ,0,0);
						//this->variables.Get_fb().PasteSumVector( VNULL ,3,0); // TODO something related to inertia?
					};

	virtual void VariablesQbLoadSpeed() 
					{ 
						this->variables.Get_qb().PasteVector(this->pos_dt,0,0);
						this->variables.Get_qb().PasteVector(this->D_dt,3,0);
					};

	virtual void VariablesQbSetSpeed(double step=0.) 
					{
						ChVector<> old_dt  = this->pos_dt;
						ChVector<> oldD_dt = this->D_dt;
						this->SetPos_dt( this->variables.Get_qb().ClipVector(0,0) );
						this->SetD_dt  ( this->variables.Get_qb().ClipVector(3,0) );
						if (step)
						{
							this->SetPos_dtdt( (this->pos_dt - old_dt )  / step);
							this->SetD_dtdt  ( (this->D_dt   - oldD_dt)  / step);
						}
					};

	virtual void VariablesFbIncrementMq() 
					{
						this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						ChVector<> newspeed   = variables.Get_qb().ClipVector(0,0);
						ChVector<> newspeed_D = variables.Get_qb().ClipVector(3,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetPos( this->GetPos() + newspeed   * step);
						this->SetD  ( this->GetD()   + newspeed_D * step);
					};

private:
	/// 3D node variables, with x,y,z and Rx,Ry,Rz (Hack! the Rx Ry Rz used here for the D gradient vector stuff)
	ChLcpVariablesBodyOwnMass  variables; 

	ChVector<> X0;		///< reference position
	ChVector<> Force;	///< applied force
	
public:
	ChVector<> pos;		
	ChVector<> pos_dt;
	ChVector<> pos_dtdt;

	ChVector<> D;		
	ChVector<> D_dt;
	ChVector<> D_dtdt;


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






