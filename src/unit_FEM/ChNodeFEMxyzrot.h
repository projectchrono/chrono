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

#ifndef CHNODEFEMXYZROT_H
#define CHNODEFEMXYZROT_H


#include "ChNodeFEMbase.h"
#include "physics/ChBodyFrame.h"
#include "core/ChFrameMoving.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"


namespace chrono
{
namespace fem
{


/// Class for a generic finite element node 
/// in 3D space, with x,y,z displacement and 3D rotation.
/// This is the typical node that can be used for beams, etc.

class ChApiFem ChNodeFEMxyzrot : public ChNodeFEMbase,
								 public ChBodyFrame

{
public:

	ChNodeFEMxyzrot(ChFrame<> initialf = ChFrame<>() ) 	
					{
						this->Frame() = initialf;

						X0    = ChFrame<>(initialf);

						Force = VNULL;
						Torque = VNULL;

						variables.SetBodyMass(0.0);
						variables.GetBodyInertia().FillElem(0.0);
					}

	~ChNodeFEMxyzrot() {};

	ChNodeFEMxyzrot (const ChNodeFEMxyzrot& other) :
						ChNodeFEMbase(other),
						ChBodyFrame(other)
	{
		this->X0 = other.X0;

		this->Force = other.Force;
		this->Force = other.Torque;

		this->variables = other.variables;
	}

	ChNodeFEMxyzrot& operator= (const ChNodeFEMxyzrot& other)
	{
		if (&other == this) 
			return *this;

		ChNodeFEMbase::operator=(other);
		ChBodyFrame::operator=(other);

		this->X0 = other.X0;

		this->Force = other.Force;
		this->Force = other.Torque;

		this->variables = other.variables;

		return *this;
	}

	virtual ChLcpVariables& Variables()
					{
						return this->variables; 
					} 

    virtual ChLcpVariablesBodyOwnMass& VariablesBody() 
					{
						return this->variables; 
					}


				/// Set the rest position as the actual position.
	virtual void Relax () 
					{
						this->X0 = *this; 
						this->GetPos_dt() = VNULL; 
						this->GetRot_dtdt() = QNULL;
						this->GetPos_dtdt() = VNULL; 
						this->GetRot_dtdt() = QNULL;
					}

				/// Get atomic mass of the node.
	virtual double GetMass() {return this->variables.GetBodyMass();}
				/// Set atomic mass of the node.
	virtual void SetMass(double mm) {this->variables.SetBodyMass(mm);}

				/// Access atomic inertia of the node.
	virtual ChMatrix33<>& GetInertia() {return this->variables.GetBodyInertia();}

				/// Set the initial (reference) frame
	virtual void SetX0(ChFrame<> mx) { X0 = mx;}
				/// Get the initial (reference) frame
	virtual ChFrame<> GetX0 () {return X0;}

				/// Set the 3d applied force, in absolute reference
	virtual void SetForce(ChVector<> mf) { Force = mf;}
				/// Get the 3d applied force, in absolute reference
	virtual ChVector<> GetForce () {return Force;}

				/// Set the 3d applied torque, in absolute reference
	virtual void SetTorque(ChVector<> mf) { Torque = mf;}
				/// Get the 3d applied torque, in absolute reference
	virtual ChVector<> GetTorque () {return Torque;}

				/// Access the frame of the node - in absolute csys,
				/// with infos on actual position, speed, acceleration, etc.
	ChFrameMoving<>& Frame() {return *this;}


				/// Sets the 'fixed' state of the node. If true, it does not move
                /// respect to the absolute world, despite constraints, forces, etc.
	void SetFixed  (bool mev) { variables.SetDisabled(mev); }
				/// Gets the 'fixed' state of the node.
    bool GetFixed()  {return variables.IsDisabled(); }


			//
			// Functions for interfacing to the LCP solver
			//

	virtual void VariablesFbLoadForces(double factor=1.) 
					{ 
						ChVector<> gyro = Vcross ( this->GetWvel_loc(), (variables.GetBodyInertia().Matr_x_Vect (this->GetWvel_loc())));

						this->variables.Get_fb().PasteSumVector( this->Force * factor ,0,0);
						this->variables.Get_fb().PasteSumVector((this->Torque - gyro)* factor ,3,0);
					};

	virtual void VariablesQbLoadSpeed() 
					{ 
						// set current speed in 'qb', it can be used by the LCP solver when working in incremental mode
						this->variables.Get_qb().PasteVector(this->GetCoord_dt().pos,0,0);
						this->variables.Get_qb().PasteVector(this->GetWvel_loc()    ,3,0);
					};

	virtual void VariablesQbSetSpeed(double step=0.) 
					{
						ChCoordsys<> old_coord_dt = this->GetCoord_dt();

						// from 'qb' vector, sets body speed, and updates auxiliary data
						this->SetPos_dt(   this->variables.Get_qb().ClipVector(0,0) );
						this->SetWvel_loc( this->variables.Get_qb().ClipVector(3,0) );

						// apply limits (if in speed clamping mode) to speeds.
						//ClampSpeed(); 

						// Compute accel. by BDF (approximate by differentiation);
						if (step)
						{
							this->SetPos_dtdt( (this->GetCoord_dt().pos - old_coord_dt.pos)  / step);
							this->SetRot_dtdt( (this->GetCoord_dt().rot - old_coord_dt.rot)  / step);
						}
					};

	virtual void VariablesFbIncrementMq() 
					{
						this->variables.Compute_inc_Mb_v(this->variables.Get_fb(), this->variables.Get_qb());
					};

	virtual void VariablesQbIncrementPosition(double step) 
					{
						//if (!this->IsActive()) 
						//	return;

						// Updates position with incremental action of speed contained in the
						// 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

						ChVector<> newspeed = variables.Get_qb().ClipVector(0,0);
						ChVector<> newwel   = variables.Get_qb().ClipVector(3,0);

						// ADVANCE POSITION: pos' = pos + dt * vel
						this->SetPos( this->GetPos() + newspeed * step);

						// ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
						ChQuaternion<> mdeltarot;
						ChQuaternion<> moldrot = this->GetRot();
						ChVector<> newwel_abs = this->Amatrix * newwel;
						double mangle = newwel_abs.Length() * step;
						newwel_abs.Normalize();
						mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
						ChQuaternion<> mnewrot = mdeltarot % moldrot;
						this->SetRot( mnewrot );
					};

private:
	ChLcpVariablesBodyOwnMass	variables; /// 3D node variables, with x,y,z displ. and 3D rot.

	//ChFrameMoving<> frame;	///< frame

	ChFrame<> X0;		///< reference frame

	ChVector<> Force;	///< applied force
	ChVector<> Torque;	///< applied torque
	


};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






