///////////////////////////////////////////////////
//
//   ChLinkClearance.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
 
#include "physics/ChLinkClearance.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). 

 
namespace chrono
{

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR CLEARANCE LINKS
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkClearance> a_registration_ChLinkClearance;


ChLinkClearance::ChLinkClearance ()
{
    type = LNK_CLEARANCE;     // initializes type

    clearance = 0.1;
    c_friction = 0.;
    c_viscous = 0.;
    c_restitution = 0.9;
    c_tang_restitution = 0.9;
	diameter = 0.8;

	contact_F_abs = VNULL;
	contact_V_abs = VNULL;

	this->limit_X->Set_active(TRUE);
    this->limit_X->Set_max(clearance);
    this->limit_X->Set_maxElastic(c_restitution);
    this->limit_X->Set_min(-1000.0);

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                       false, true, true, false,
                       false, false);

    ChangedLinkMask();

}

ChLinkClearance::~ChLinkClearance ()
{

}

void ChLinkClearance::Copy(ChLinkClearance* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:
    clearance = source->clearance;
	c_friction = source->c_friction;
	c_restitution = source->c_restitution;
	c_tang_restitution = source->c_tang_restitution;
	c_viscous = source->c_viscous;
	diameter = source->diameter;

	contact_F_abs = source->contact_F_abs;
	contact_V_abs = source->contact_V_abs;
}

ChLink* ChLinkClearance::new_Duplicate ()
{
    ChLinkClearance* m_l;
    m_l = new ChLinkClearance;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

// easy data getting

double  ChLinkClearance::Get_axis_eccentricity()
{
	return (this->GetDist());
}
double  ChLinkClearance::Get_axis_phase()
{
	if (!this->GetMarker2()) return 0;
	double mangle;
	Vector maxis;
	Quaternion temp=this->GetMarker2()->GetCoord().rot;
	Q_to_AngAxis(&temp, &mangle, &maxis);
	if (maxis.z < 0.0)
    {
		maxis = Vmul (maxis, -1.0);
		mangle = (2.0*CH_C_PI)-mangle;
    }
	return (mangle);
}
double  ChLinkClearance::Get_rotation_angle()
{
	return (this->GetRelAngle());
}
Vector  ChLinkClearance::Get_contact_P_abs()
{
	if (!this->GetMarker2()) return VNULL;
	return Vadd(this->GetMarker2()->GetAbsCoord().pos,
		        Vmul(this->Get_contact_N_abs(), -(this->clearance+this->diameter/2)) );
}
Vector  ChLinkClearance::Get_contact_N_abs()
{
	if (!this->GetMarker2()) return VECT_X;
	Vector mNrel = VECT_X;
	mNrel = Vmul(mNrel, -1);
	return (this->GetMarker2()->Dir_Ref2World(&mNrel));
}
Vector  ChLinkClearance::Get_contact_F_abs()
{
	return (this->contact_F_abs);
}
double  ChLinkClearance::Get_contact_F_n()
{
	if (!this->GetMarker2()) return 0;
	return (this->GetMarker2()->Dir_World2Ref(&this->contact_F_abs).x);
}
double  ChLinkClearance::Get_contact_F_t()
{
	if (!this->GetMarker2()) return 0;
	return (this->GetMarker2()->Dir_World2Ref(&this->contact_F_abs).y);
}
double  ChLinkClearance::Get_contact_V_t()
{
	if (!this->GetMarker2()) return 0;
	return (this->GetMarker2()->Dir_World2Ref(&this->contact_V_abs).y);
}


void ChLinkClearance::UpdateForces (double mytime)
{
    // May avoid inheriting parent class force computation, since not
    // needed...
    // LinkLock::UpdateForces(mytime);

	Vector m_friction_F_abs = VNULL;
	double m_norm_force = - this->react_force.x;

    // Just add coloumb kinematic friction...

	if (((ChLinkMaskLF*)(this->GetMask()))->Constr_X().IsActive() )
    {
		Vector temp=Get_contact_P_abs();
        Vector pb1 = Body1->Point_World2Body(&temp);
        Vector pb2 = Body2->Point_World2Body(&temp);
        Vector m_V1_abs = Body1->RelPoint_AbsSpeed(&pb1 );
        Vector m_V2_abs = Body2->RelPoint_AbsSpeed(&pb2 );
		this->contact_V_abs = Vsub(m_V1_abs, m_V2_abs);
		Vector m_tang_V_abs = Vsub(contact_V_abs,
							  Vmul(Get_contact_N_abs(), Vdot(contact_V_abs, Get_contact_N_abs())));

			// absolute friction force, as applied in contact point
        m_friction_F_abs = Vmul(Vnorm(m_tang_V_abs), Get_c_friction()*(- m_norm_force));
		
			// transform the friction force in link master coords ***TO CHECK*** (new version!)
		this->C_force += this->marker2->Dir_World2Ref(&m_friction_F_abs); 
    }

	// update internal data: the abs. vector of all contact forces, is a sum of reaction and friction
	this->contact_F_abs = Vadd(Vmul(Get_contact_N_abs(), m_norm_force) ,
								m_friction_F_abs);

}





void ChLinkClearance::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);


    // Move (well, rotate...) marker 2 to align it in actuator direction

        // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos,
                          marker2->GetAbsCoord().pos);

	Vector mz = ma.Get_A_Zaxis();
	Vector my = Vnorm (Vcross(ma.Get_A_Zaxis(), absdist));
	Vector mx = Vnorm (Vcross(my, ma.Get_A_Zaxis() ));

    ma.Set_A_axis(mx,my,mz);

    Coordsys newmarkpos;
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);        //rotate "main" marker2 into tangent position


        // imposed relative positions/speeds
    deltaC.pos = VNULL;
    deltaC.pos.x = this->clearance;        // distance is always on M2 'X' axis

    deltaC_dt.pos = VNULL;
    deltaC_dt.pos.x = 0; // distance speed

    deltaC_dtdt.pos = VNULL;

        // add also the centripetal acceleration if distance vector's rotating,
        // as centripetal acc. of point sliding on a sphere surface.
     Vector tang_speed = GetRelM_dt().pos;
      tang_speed.x = 0; // only z-y coords in relative tang speed vector
	 double Rcurvature = Vlenght(absdist);
    deltaC_dtdt.pos.x  = - pow(Vlenght(tang_speed), 2) / Rcurvature;  // An =  -(Vt^2 / r)

    deltaC.rot = QUNIT;             // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}


void ChLinkClearance::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << clearance;
    mstream << c_friction;
    mstream << c_restitution;
    mstream << diameter;
    mstream << c_tang_restitution;
    mstream << c_viscous;
}

void ChLinkClearance::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	mstream >> clearance;
    mstream >> c_friction;
    mstream >> c_restitution;
    mstream >> diameter;
    mstream >> c_tang_restitution;
    mstream >> c_viscous;
}




///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


