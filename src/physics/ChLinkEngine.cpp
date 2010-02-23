///////////////////////////////////////////////////
//
//   ChLinkEngine.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 

#include "physics/ChLinkEngine.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR ENGINE LINKS
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkEngine> a_registration_ChLinkEngine;


ChLinkEngine::ChLinkEngine ()
{
    type = LNK_ENGINE;      // initializes type

    rot_funct =   new ChFunction (0);
    spe_funct =   new ChFunction (0);
    tor_funct =   new ChFunction (0);
    torque_w  =   new ChFunction (1);

	rot_funct_x =   new ChFunction (0);
	rot_funct_y =   new ChFunction (0);

    mot_rot = mot_rot_dt = mot_rot_dtdt = 0.0;
    mot_rerot = mot_rerot_dt = mot_rerot_dtdt = 0.0;
    mot_torque = mot_retorque = 0.0;
	last_r3mot_rot = 0;
	last_r3mot_rot_dt = 0;
	last_r3relm_rot = QUNIT;
	last_r3relm_rot_dt = QNULL;
	last_r3time = 0;
	keyed_polar_rotation = QNULL;
    impose_reducer = FALSE;

    mot_tau = 1.0;
    mot_eta = 1.0;
    mot_inertia = 0.0;

	cache_li_speed1 = 0;
	cache_li_pos1 = 0;
	torque_react1 = 0;
	cache_li_speed2 = 0;
	cache_li_pos2 = 0;
	torque_react2 = 0;

    eng_mode= ENG_MODE_ROTATION;
    learn = FALSE;

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to E3 only.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false,
                       false, false, false, true,
                       false, false);
    ChangedLinkMask();
            // Mask: initialize remaining LinkMaskLF (lock formulation mask) for the engine.
            // All shaft modes at least are setting the lock on E3 (z-rotation) coordinate.
    Set_shaft_mode(ENG_SHAFT_LOCK);
}

ChLinkEngine::~ChLinkEngine ()
{
    if (rot_funct) delete rot_funct;
    if (spe_funct) delete spe_funct;
    if (tor_funct) delete tor_funct;
    if (torque_w)  delete torque_w;

	if (rot_funct_x) delete rot_funct_x;
	if (rot_funct_y) delete rot_funct_y;
}

void ChLinkEngine::Copy(ChLinkEngine* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:
    learn = source->learn;
    eng_mode= source->eng_mode;
    shaft_mode = source->shaft_mode;

    mot_rot = source->mot_rot;
    mot_rot_dt = source->mot_rot_dt;
    mot_rot_dtdt = source->mot_rot_dtdt;
    mot_rerot = source->mot_rerot;
    mot_rerot_dt = source->mot_rerot_dt;
    mot_rerot_dtdt = source->mot_rerot_dtdt;
    mot_torque = source->mot_torque;
    mot_retorque = source->mot_retorque;
    impose_reducer = source->impose_reducer;
	last_r3time = source->last_r3time;
	last_r3mot_rot = source->last_r3mot_rot;
	last_r3mot_rot_dt = source->last_r3mot_rot_dt;
	last_r3relm_rot = source->last_r3relm_rot;
	last_r3relm_rot_dt = source->last_r3relm_rot_dt;
	keyed_polar_rotation = source->keyed_polar_rotation;

    if (rot_funct) delete rot_funct;
        rot_funct =   source->rot_funct->new_Duplicate();
    if (spe_funct) delete spe_funct;
        spe_funct =   source->spe_funct->new_Duplicate();
    if (tor_funct) delete tor_funct;
        tor_funct =   source->tor_funct->new_Duplicate();
    if (torque_w)  delete torque_w;
        torque_w =   source->torque_w->new_Duplicate();

	if (rot_funct_x) delete rot_funct_x;
        rot_funct_x =   source->rot_funct_x->new_Duplicate();
    if (rot_funct_y) delete rot_funct_y;
        rot_funct_y =   source->rot_funct_y->new_Duplicate();

    mot_tau = source->mot_tau;
    mot_eta = source->mot_eta;
    mot_inertia = source->mot_inertia;

	cache_li_speed1 = 0;
	cache_li_pos1 = 0;
	torque_react1 = source->torque_react1;
	cache_li_speed2 = 0;
	cache_li_pos2 = 0;
	torque_react2 = source->torque_react2;
}

ChLink* ChLinkEngine::new_Duplicate ()
{
    ChLinkEngine* m_l;
    m_l = new ChLinkEngine;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkEngine::Set_learn(int mset)
{
    learn = mset;

    if ((this->eng_mode == ENG_MODE_ROTATION) ||
        (this->eng_mode == ENG_MODE_SPEED) ||
		(this->eng_mode == ENG_MODE_KEY_ROTATION) )
    {
        if (mset)
            ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
        else
            ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);

        ChangedLinkMask ();
    }

	if (this->eng_mode == ENG_MODE_KEY_POLAR)
	{
		if (mset) {
            ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_FREE);
			((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_FREE);
			((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
		}
        else {
            ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_LOCK);
			((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_LOCK);
			((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
        }
        ChangedLinkMask ();
	}

    if (this->eng_mode == ENG_MODE_ROTATION)
        if (rot_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
        {
            delete (rot_funct); rot_funct = NULL;
            rot_funct = new ChFunction_Recorder;
        }

    if (this->eng_mode == ENG_MODE_SPEED)
        if (spe_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
        {
            delete (spe_funct); spe_funct = NULL;
            spe_funct = new ChFunction_Recorder;
        }
}

void ChLinkEngine::Set_rot_funct(ChFunction* m_funct)
{
    if (rot_funct) delete rot_funct;
        rot_funct = m_funct;
}
void ChLinkEngine::Set_spe_funct(ChFunction* m_funct)
{
    if (spe_funct) delete spe_funct;
        spe_funct = m_funct;
}
void ChLinkEngine::Set_tor_funct(ChFunction* m_funct)
{
    if (tor_funct) delete tor_funct;
        tor_funct = m_funct;
}
void ChLinkEngine::Set_torque_w_funct(ChFunction* m_funct)
{
    if (torque_w) delete torque_w;
        torque_w = m_funct;
}
void ChLinkEngine::Set_rot_funct_x(ChFunction* m_funct_x)
{
    if (rot_funct_x) delete rot_funct_x;
        rot_funct_x = m_funct_x;
}
void ChLinkEngine::Set_rot_funct_y(ChFunction* m_funct_y)
{
    if (rot_funct_y) delete rot_funct_y;
        rot_funct_y = m_funct_y;
}
void ChLinkEngine::SetKeyedPolarRotation(Quaternion mq)
{
	keyed_polar_rotation = mq;
}


void ChLinkEngine::Set_eng_mode(int mset)
{
    if (Get_learn()) Set_learn(FALSE); // reset learn state when changing mode

    if (eng_mode != mset)
    {
        eng_mode = mset;
        if (this->eng_mode == ENG_MODE_ROTATION ||
            this->eng_mode == ENG_MODE_SPEED ||
			this->eng_mode == ENG_MODE_KEY_ROTATION)
        {
            ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
        }

		if (this->eng_mode == ENG_MODE_KEY_POLAR)
		{
            ((ChLinkMaskLF*)mask)->Constr_E1().SetMode(CONSTRAINT_LOCK);
			((ChLinkMaskLF*)mask)->Constr_E2().SetMode(CONSTRAINT_LOCK);
			((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_LOCK);
		}

        if (this->eng_mode == ENG_MODE_TORQUE )
        {
            ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);

        }

		if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
        {
            ((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
        }

        ChangedLinkMask();  // update all from new mask
    }

	if (this->eng_mode == ENG_MODE_KEY_ROTATION)
        if (rot_funct->Get_Type() != FUNCT_CONST) // if wasn't constant f()..
        {
            delete (rot_funct); rot_funct = NULL;
            rot_funct = new ChFunction;
        }

	if (this->eng_mode == ENG_MODE_KEY_POLAR)
	{
        if (rot_funct->Get_Type() != FUNCT_CONST)  {
            delete (rot_funct); rot_funct = NULL;
            rot_funct = new ChFunction;
        }
		if (rot_funct_x->Get_Type() != FUNCT_CONST)  {
            delete (rot_funct_x); rot_funct_x = NULL;
            rot_funct_x = new ChFunction;
        }
		if (rot_funct_y->Get_Type() != FUNCT_CONST)  {
            delete (rot_funct_y); rot_funct_y = NULL;
            rot_funct_y = new ChFunction;
        }
	}
}

void ChLinkEngine::Set_shaft_mode(int mset)
{
    shaft_mode = mset;

	eChConstraintMode curr_mode_z = ((ChLinkMaskLF*)mask)->Constr_E3().GetMode();

    switch(this->shaft_mode)
    {
    case ENG_SHAFT_PRISM:
        ((ChLinkMaskLF*)mask)->SetLockMask(true, true, false,
                           false, true, true, true, // <-
                           false, false); break;
    case ENG_SHAFT_UNIVERSAL:
        ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                           false, false, false, true, // <-
                           false, false); break;
    case ENG_SHAFT_CARDANO:
        ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                           false, false, false, true, // <-
                           false, false); break;
    case ENG_SHAFT_OLDHAM:
        ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                           false, true, true, true, // <-
                           false, false); break;
    case ENG_SHAFT_LOCK:
    default:
        ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                           false, true, true, true, // <-
                           false, false); break;
    }

	((ChLinkMaskLF*)mask)->Constr_E3().SetMode(curr_mode_z);


    // change datas
    ChangedLinkMask();
}


void ChLinkEngine::UpdatedExternalTime (double prevtime, double time)
{
	last_r3time = this->ChTime;
	last_r3mot_rot = this->Get_mot_rot();
	last_r3mot_rot_dt = this->Get_mot_rot_dt();
	last_r3relm_rot = this->GetRelM().rot;
	last_r3relm_rot_dt = this->GetRelM_dt().rot;
}

void ChLinkEngine::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

	if (!IsActive()) 
		return;

    // DEFAULTS compute rotation vars...
    // by default for torque control..

    motion_axis = VECT_Z;       // motion axis is always the marker2 Z axis (in m2 relative coords)
    mot_rot = this->relAngle;
    mot_rot_dt = Vdot(this->relWvel, motion_axis);
    mot_rot_dtdt = Vdot(this->relWacc, motion_axis);
    mot_rerot = mot_rot / this->mot_tau;
    mot_rerot_dt = mot_rot_dt / this->mot_tau;
    mot_rerot_dtdt = mot_rot_dtdt / this->mot_tau;

                // nothing more to do here fortorque control
    if (this->eng_mode == ENG_MODE_TORQUE) return; // >>>>>>


    // If LEARN MODE, just record motion
    if (learn == TRUE)
    {
        deltaC.pos = VNULL;
        deltaC_dt.pos = VNULL;
        deltaC_dtdt.pos = VNULL;
		if(!(limit_Rx->Get_active() ||
			 limit_Ry->Get_active() ||
			 limit_Rz->Get_active()))
		{
			deltaC.rot = QUNIT;
			deltaC_dt.rot = QNULL;
			deltaC_dtdt.rot = QNULL;
		}

        if (this->eng_mode == ENG_MODE_ROTATION)
        {
            if (rot_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
            {
                delete (rot_funct); rot_funct = NULL;
                rot_funct = new ChFunction_Recorder;
            }
                    // record point
            double rec_rot = this->relAngle; // ***TO DO*** compute also rotations with cardano mode?
            if (this->impose_reducer)
                rec_rot = rec_rot / this->mot_tau;
            ((ChFunction_Recorder*)rot_funct)->AddPoint(mytime, rec_rot, 1);  //  x=t
        }
        if (this->eng_mode == ENG_MODE_SPEED)
        {
            if (spe_funct->Get_Type() != FUNCT_RECORDER) // if wasn't recorder f()..
            {
                delete (spe_funct); spe_funct = NULL;
                spe_funct = new ChFunction_Recorder;
            }
                    // record point
            double rec_spe = Vlenght(this->relWvel); // ***TO DO*** compute also with cardano mode?
            if (this->impose_reducer)
                rec_spe = rec_spe / this->mot_tau;
            ((ChFunction_Recorder*)spe_funct)->AddPoint(mytime, rec_spe, 1);  //  x=t
        }
    }


    if (learn == TRUE) return; // no need to go on further...--->>>>
                               // no need to go on further...--->>>>

    // Impose relative positions/speeds

    deltaC.pos = VNULL;
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;

    if (this->eng_mode == ENG_MODE_ROTATION)
    {
        if (this->impose_reducer)
        {
            mot_rerot = this->rot_funct->Get_y(ChTime);
            mot_rerot_dt = this->rot_funct->Get_y_dx(ChTime);
            mot_rerot_dtdt = this->rot_funct->Get_y_dxdx(ChTime);
            mot_rot = mot_rerot * this->mot_tau;
            mot_rot_dt = mot_rerot_dt * this->mot_tau;
            mot_rot_dtdt = mot_rerot_dtdt * this->mot_tau;
        } else {
            mot_rot = this->rot_funct->Get_y(ChTime);
            mot_rot_dt = this->rot_funct->Get_y_dx(ChTime);
            mot_rot_dtdt = this->rot_funct->Get_y_dxdx(ChTime);
            mot_rerot = mot_rot / this->mot_tau;
            mot_rerot_dt = mot_rot_dt / this->mot_tau;
            mot_rerot_dtdt = mot_rot_dtdt / this->mot_tau;
        }
        deltaC.rot = Q_from_AngAxis (mot_rot, motion_axis);
        deltaC_dt.rot = Qdt_from_AngAxis (deltaC.rot, mot_rot_dt, motion_axis);
        deltaC_dtdt.rot = Qdtdt_from_AngAxis (mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
    }
    if (this->eng_mode == ENG_MODE_SPEED)
    {
        if (this->impose_reducer)
        {
            mot_rerot_dt = this->spe_funct->Get_y(ChTime);
            mot_rerot_dtdt = this->spe_funct->Get_y_dx(ChTime);
            mot_rot_dt = mot_rerot_dt * this->mot_tau;
            mot_rot_dtdt = mot_rerot_dtdt * this->mot_tau;
        } else {
            mot_rot_dt = this->spe_funct->Get_y(ChTime);
            mot_rot_dtdt = this->spe_funct->Get_y_dx(ChTime);
            mot_rerot_dt = mot_rot_dt / this->mot_tau;
            mot_rerot_dtdt = mot_rot_dtdt / this->mot_tau;
        }
        deltaC.rot = Qnorm(this->GetRelM().rot);    // just keep current position, -assume always good after integration-.
		ChMatrix33<> relA; relA.Set_A_quaternion(this->GetRelM().rot);  // ..but adjust to keep Z axis aligned to shaft, anyway!
		ChVector<> displaced_z_axis; displaced_z_axis = relA.Get_A_Zaxis();
		ChVector<> adjustment = Vcross(displaced_z_axis, VECT_Z);
		deltaC.rot = Q_from_AngAxis(Vlenght(adjustment),Vnorm(adjustment)) % deltaC.rot;
        deltaC_dt.rot = Qdt_from_AngAxis (deltaC.rot, mot_rot_dt, motion_axis);
        deltaC_dtdt.rot = Qdtdt_from_AngAxis (mot_rot_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
    }

}

void ChLinkEngine::UpdateForces (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

	if (!IsActive()) 
		return;

    // DEFAULTS set null torques
    this->mot_torque = 0;
    this->mot_retorque = 0;

    if (this->eng_mode == ENG_MODE_TORQUE)
    {
        // in torque mode, apply the torque vector to both m1 and m2
        //  -  M= f(t)
        double my_torque = this->Get_tor_funct()->Get_y(ChTime);

        if (this->impose_reducer)
        {
            my_torque = my_torque*this->Get_torque_w_funct()->Get_y(this->mot_rerot_dt);
            mot_retorque = my_torque;
            mot_torque = mot_retorque*(this->mot_eta/this->mot_tau);
        } else {
            my_torque = my_torque*this->Get_torque_w_funct()->Get_y(this->mot_rot_dt);
            mot_torque = my_torque;
            mot_retorque = mot_retorque*(this->mot_tau/this->mot_eta);
        }

        Vector mv_torque = Vmul(motion_axis, mot_torque);

        // +++ADD TO LINK TORQUE VECTOR
        C_torque = Vadd(C_torque, mv_torque);

    }

    if ((this->eng_mode == ENG_MODE_ROTATION) ||
        (this->eng_mode == ENG_MODE_SPEED) ||
		(this->eng_mode == ENG_MODE_KEY_ROTATION) )
    {
        this->mot_torque = this->react_torque.z;
        this->mot_retorque = mot_torque*(mot_tau/mot_eta)+mot_rerot_dtdt*mot_inertia;
    }

    if(this->eng_mode == ENG_MODE_SPEED)
    {
        // trick: zeroes Z rotat. violation to tell that rot.position is always ok
        if (C->GetRows())
            C->SetElement(C->GetRows() -1, 0 , 0.0);
    }
}



void ChLinkEngine::SetMarker1 (ChMarker* mark1)
{
	ChLinkLock::SetMarker1 (mark1);
	
	if(this->Body1 && this->Body2)
	{
		this->innerconstraint1.SetVariables(&Body1->Variables(), &innershaft1.Variables());
		this->innerconstraint2.SetVariables(&Body2->Variables(), &innershaft2.Variables());
	}
}

void ChLinkEngine::SetMarker2 (ChMarker* mark2)
{
	ChLinkLock::SetMarker2 (mark2);
	
	if(this->Body1 && this->Body2)
	{
		this->innerconstraint1.SetVariables(&Body1->Variables(), &innershaft1.Variables());
		this->innerconstraint2.SetVariables(&Body2->Variables(), &innershaft2.Variables());
	}
	
}


//
//  LCP functions
//


void ChLinkEngine::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	// First, inherit to parent class
    ChLinkLock::InjectConstraints(mdescriptor);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		mdescriptor.InsertConstraint(&this->innerconstraint1);
		mdescriptor.InsertConstraint(&this->innerconstraint2);
	}
}

void ChLinkEngine::ConstraintsBiReset()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsBiReset();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innerconstraint1.Set_b_i(0.);
		this->innerconstraint2.Set_b_i(0.);
	}
}
 
void ChLinkEngine::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		double res = 0; // no residual
		this->innerconstraint1.Set_b_i(innerconstraint1.Get_b_i() +  factor * res);
		this->innerconstraint2.Set_b_i(innerconstraint2.Get_b_i() +  factor * res);
	}
}

void ChLinkEngine::ConstraintsBiLoad_Ct(double factor)
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsBiLoad_Ct(factor);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		// nothing
	}
}

void ChLinkEngine::ConstraintsLoadJacobians()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsLoadJacobians();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		// compute jacobians
		ChVector<> abs_rot_axis = this->marker2->Dir_Ref2World(&ChVector<>(VECT_Z));
		ChVector<> jacw = this->Body2->Dir_World2Body(&abs_rot_axis);

		this->innerconstraint1.Get_Cq_a()->ElementN(0)=0;
		this->innerconstraint1.Get_Cq_a()->ElementN(1)=0;
		this->innerconstraint1.Get_Cq_a()->ElementN(2)=0;
		this->innerconstraint1.Get_Cq_a()->ElementN(3)=jacw.x;
		this->innerconstraint1.Get_Cq_a()->ElementN(4)=jacw.y;
		this->innerconstraint1.Get_Cq_a()->ElementN(5)=jacw.z;
		this->innerconstraint1.Get_Cq_b()->ElementN(0)=-1;

		this->innerconstraint2.Get_Cq_a()->ElementN(0)=0;
		this->innerconstraint2.Get_Cq_a()->ElementN(1)=0;
		this->innerconstraint2.Get_Cq_a()->ElementN(2)=0;
		this->innerconstraint2.Get_Cq_a()->ElementN(3)=jacw.x;
		this->innerconstraint2.Get_Cq_a()->ElementN(4)=jacw.y;
		this->innerconstraint2.Get_Cq_a()->ElementN(5)=jacw.z;
		this->innerconstraint2.Get_Cq_b()->ElementN(0)=-1;
	}
}
 

void ChLinkEngine::ConstraintsFetch_react(double factor)
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsFetch_react(factor);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		// From constraints to react vector:
		this->torque_react1 = innerconstraint1.Get_l_i() * factor;  
		this->torque_react2 = innerconstraint2.Get_l_i() * factor;  
	}
}

void  ChLinkEngine::ConstraintsLiLoadSuggestedSpeedSolution()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsLiLoadSuggestedSpeedSolution();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		innerconstraint1.Set_l_i(this->cache_li_speed1);
		innerconstraint2.Set_l_i(this->cache_li_speed2);
	}
}

void  ChLinkEngine::ConstraintsLiLoadSuggestedPositionSolution()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsLiLoadSuggestedPositionSolution();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		innerconstraint1.Set_l_i(this->cache_li_pos1);
		innerconstraint2.Set_l_i(this->cache_li_pos2);
	}
}

void  ChLinkEngine::ConstraintsLiFetchSuggestedSpeedSolution()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsLiFetchSuggestedSpeedSolution();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->cache_li_speed1 = (float)innerconstraint1.Get_l_i();
		this->cache_li_speed2 = (float)innerconstraint2.Get_l_i();
	}
}

void  ChLinkEngine::ConstraintsLiFetchSuggestedPositionSolution()
{
	// First, inherit to parent class
    ChLinkLock::ConstraintsLiFetchSuggestedPositionSolution();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->cache_li_pos1 =  (float)innerconstraint1.Get_l_i();
		this->cache_li_pos2 =  (float)innerconstraint2.Get_l_i();
	}
}

void ChLinkEngine::InjectVariables(ChLcpSystemDescriptor& mdescriptor)
{	
	// First, inherit to parent class
    ChLinkLock::InjectVariables(mdescriptor);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.InjectVariables(mdescriptor);
		this->innershaft2.InjectVariables(mdescriptor);
	}
}

void ChLinkEngine::VariablesFbReset()
{
	// First, inherit to parent class
    ChLinkLock::VariablesFbReset();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.VariablesFbReset();
		this->innershaft2.VariablesFbReset();
	}
}

void ChLinkEngine::VariablesFbLoadForces(double factor)
{
	// First, inherit to parent class
    ChLinkLock::VariablesFbLoadForces(factor);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.VariablesFbLoadForces(factor);
		this->innershaft2.VariablesFbLoadForces(factor);
	}
}

void ChLinkEngine::VariablesQbLoadSpeed()
{
	// First, inherit to parent class
    ChLinkLock::VariablesQbLoadSpeed();

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.VariablesQbLoadSpeed();
		this->innershaft2.VariablesQbLoadSpeed();
	}
}

void ChLinkEngine::VariablesQbSetSpeed(double step)
{
	// First, inherit to parent class
    ChLinkLock::VariablesQbSetSpeed(step);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.VariablesQbSetSpeed(step);
		this->innershaft2.VariablesQbSetSpeed(step);
	}
}

void ChLinkEngine::VariablesQbIncrementPosition(double step)
{
	// First, inherit to parent class
    ChLinkLock::VariablesQbIncrementPosition(step);

	if (this->eng_mode == ENG_MODE_TO_POWERTRAIN_SHAFT )
	{
		this->innershaft1.VariablesQbIncrementPosition(step);
		this->innershaft2.VariablesQbIncrementPosition(step);
	}
}







void ChLinkEngine::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream.AbstractWrite(rot_funct);
	mstream.AbstractWrite(spe_funct);
    mstream.AbstractWrite(tor_funct);
    mstream.AbstractWrite(torque_w);
    mstream << learn;
    mstream << impose_reducer;
    mstream << mot_tau;
    mstream << mot_eta;
    mstream << mot_inertia;
    mstream << eng_mode;
    mstream << shaft_mode;
}

void ChLinkEngine::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	ChFunction* ffoo;
	mstream.AbstractReadCreate(&ffoo);		Set_rot_funct(ffoo);
	mstream.AbstractReadCreate(&ffoo);		Set_spe_funct(ffoo);
    mstream.AbstractReadCreate(&ffoo);		Set_tor_funct(ffoo);
    mstream.AbstractReadCreate(&ffoo);		Set_torque_w_funct(ffoo);
    mstream >> learn;
    mstream >> impose_reducer;
    mstream >> mot_tau;
    mstream >> mot_eta;
    mstream >> mot_inertia;
    mstream >> eng_mode;
    mstream >> shaft_mode;
}




///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


