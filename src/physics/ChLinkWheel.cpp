//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkWheel.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
 
 
#ifdef CH_V_PLUGIN
 extern "C"
 {
    #include <r3error.h>
	#include <r3quater.h>
    #include <r3oops.h>
    #include <r3link.h>
    #include <r3freng.h>
    #include "ChRlink.h"
    #include "ChRmarker.h"
    #include "ChRbody.h"
    #include "ChRsystem.h"
    #include "ChRutils.h"
 }
 #include "ChMathR3D.h"
 #include "ChGlobalR3D.h"
 #include "ChExternalObjectR3D.h"

#endif

#include "physics/ChLinkWheel.h"
#include "physics/ChSystem.h"
//#include "physics/ChCollide.h"
//#include "lcp/ChLcpConstraintTwoFrictionOrtho.h"
//#include "lcp/ChLcpConstraintTwoFrictionApprox.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR WHEEL LINKS
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkWheel> a_registration_ChLinkWheel;


ChLinkWheel::ChLinkWheel ()
{
    type = LNK_WHEEL;       // initializes type

    wheel_rotation = new ChFunction_Const (0);
    wcollision = 0;
    speed_handled = 0;
    radius = 0.1;
    thickness = 0.0;
    friction = 0.7;
    fri_spe =  new ChFunction_Const (1);
    fri_norm = new ChFunction_Const (1);
    allow_sticking = FALSE;
    slip_treshold = 0.01;
    static_friction = 1;
    unilateral = TRUE;
    pneus_krp = FALSE;
    rad_k =   1000;
    rad_r =  0.001;
    rad_p = 100000; // about 1 atm..
    rad_k_def = new ChFunction_Const (1);
    pneus_h = 0.2;

    angle = angle_dt = angle_dtdt = slipping = f_slip =
        l_slip = derive_angle = tforce = f_tforce = l_tforce = curr_friction = 0;
    loc_iters = 0;
    mv = mu =  malpha = 0.0;

    this->limit_Z->Set_active(TRUE);
    this->limit_Z->Set_max(100000000.0);
    this->limit_Z->Set_min(0.0);

            // Mask: initialize our LinkMaskLF (lock formulation mask)
    ((ChLinkMaskLF*)mask)->SetLockMask(false,false, true,
                        false,false,false,false,
                        false, false);
	((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_UNILATERAL);

    ChangedLinkMask();
}

ChLinkWheel::~ChLinkWheel ()
{
    delete wheel_rotation;
    delete fri_spe;
    delete fri_norm;
}

void ChLinkWheel::Copy(ChLinkWheel* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:

    wcollision = source->wcollision;
    speed_handled = source->speed_handled;
    radius = source->radius;
    thickness = source->thickness;
    friction = source->friction;
    allow_sticking = source->allow_sticking;
    slip_treshold = source->slip_treshold;
    static_friction = source->static_friction;
    unilateral = source->unilateral;
    pneus_krp = source->pneus_krp;
    rad_k = source->rad_k;
    rad_r = source->rad_r;
    rad_p = source->rad_p;  // about 2 atm..
    pneus_h = source->pneus_h;

    if (wheel_rotation) delete wheel_rotation;
        wheel_rotation =   source->wheel_rotation->new_Duplicate();
    if (fri_spe) delete fri_spe;
        fri_spe =   source->fri_spe->new_Duplicate();
    if (fri_norm) delete fri_norm;
        fri_norm =   source->fri_norm->new_Duplicate();
    if (rad_k_def) delete rad_k_def;
        rad_k_def =   source->rad_k_def->new_Duplicate();

    angle = angle_dt = angle_dtdt = slipping = f_slip =
        l_slip = derive_angle = tforce = f_tforce = l_tforce = curr_friction = 0;
    loc_iters = 0;
    mv = mu =  malpha = 0.0;
}

ChLink* ChLinkWheel::new_Duplicate ()
{
    ChLinkWheel* m_l;
    m_l = new ChLinkWheel;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkWheel::Set_unilateral(int mset)
{
    this->unilateral = mset;
    if (this->unilateral)
    {
        this->limit_Z->Set_active(TRUE);
		((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_FREE);
    }
    else
    {
        this->limit_Z->Set_active(FALSE);
        ((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_LOCK);
    }
    ChangedLinkMask();
}

void ChLinkWheel::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

    Vector m2_pos, m1_pos, vx, vy, vz, m1_relacc, m1_relvel;
    Quaternion m2_rot, m1_rot;


    //defaults: if no spindle obj. is provided, marker 1 is used instead
        //spindle_csys.Set_A_quaternion(marker1->GetAbsCoord().rot);
        //spindle_pos = marker1->GetAbsCoord().pos;
    spindle_csys.Set_A_quaternion(Body1->GetCoord().rot);
    spindle_pos = Body1->GetCoord().pos;


    #ifdef CH_V_PLUGIN

    // Fetch spindle obj.  (1st subobject link in marker 1)


	R3OBJ* anobj = ((ChExternalObjectR3D*)marker1->GetExternalObject())->Get_r3d_object();

    R3OBJ* spindleobj = Ch_GetLinkedParamObject(anobj, 0);
    if (spindleobj)
    {
         // constant update surface geometry after each step... it may be rotated by chrono simulation
        Ch_ForceUpdateOfR3Dcoords(spindleobj);

        R3MATRIX m1;
        R3SendMsgA(spindleobj, R3PRIMM_GETTOABSSPACEMATRIX, &m1);
        R3Matrix_To_Chrono(m1, &spindle_csys, &spindle_pos);
    }


    // Fetch surface of contact (1st subobj link in marker 2)


	R3OBJ* anobjb = ((ChExternalObjectR3D*)marker2->GetExternalObject())->Get_r3d_object();

    R3OBJ* groundobj = Ch_GetLinkedParamObject(anobjb, 0);
    if (groundobj)
    {
        // constant update surface geometry after each step... it may be rotated by chrono simulation
        //Ch_ForceUpdateOfR3Dcoords(groundobj);
        //***NOT NEEDED since an hypothesis for the use of this constraint is that
        //   steady (not moving) grounds must be used!
    }

    #endif


    ///////  CONTACT MODE:          (also used to set defaults, and default mode)
    ///////  -- X-Z PLANE ---
    ///////

    // Default normal
    Vector surf_normal = VECT_Y;
    // Radius vector from spindle to ground (default)
    Vector vgroundrad = Vmul( Vcross (spindle_csys.Get_A_Zaxis(), Vnorm (Vcross (spindle_csys.Get_A_Zaxis() , surf_normal) )), Get_RigidRadius());

    // Defaults for marker positions and rotations
    m1_pos = Vadd(spindle_pos, vgroundrad);

     Vector vm1z = Vmul(Vnorm(vgroundrad), -1.0);
     Vector vm1x = Vnorm(Vcross( spindle_csys.Get_A_Zaxis(), vm1z ));
     ChMatrix33<> a_m1;
     a_m1.Set_A_axis(vm1x, spindle_csys.Get_A_Zaxis(), vm1z );
    m1_rot = Qnorm(a_m1.Get_A_quaternion());


    m2_pos= m1_pos;
    m2_pos.y = 0.0;

     ChMatrix33<> a_m2;
     vz = surf_normal;
     vy = spindle_csys.Get_A_Zaxis();
     vx = Vnorm(Vcross(vy, vz));
     vy = Vnorm(Vcross(vz, vx));
     a_m2.Set_A_axis(vx, vy, vz);
    m2_rot = a_m2.Get_A_quaternion();

    // Compute relative speed of reference M1
    // (hypothesis: ground is flat and steady: contact must move with same vel. as wheel spindle)
    Vector vabsradius = Vsub(m1_pos, spindle_pos);
    m1_relvel = Body1->GetA()->MatrT_x_Vect(
        Vcross(vabsradius, Body1->GetWvel_par()) );
    // Compute relative acceleration of reference M1
    // (hypothesis: ground is flat and steady: contact must move with same acc. as wheel spindle)
    m1_relacc = Body1->GetA()->MatrT_x_Vect(
        Vcross(Body1->GetWvel_par() , Vcross(vabsradius, Body1->GetWvel_par()) ) ); // cut centrip
    m1_relacc = Vadd( m1_relacc,
        Vmul (Vcross(Body1->GetWvel_loc(), m1_relvel) , -2.0 ) );   // cut coriolis
    m1_relacc = Vadd( m1_relacc,
        Body1->GetA()->MatrT_x_Vect(Vcross(vabsradius, Body1->GetWacc_par() ) ) ); // cut tang acc


    ///////  CONTACT MODE:
    ///////  -- Y-COLLISION ---
    ///////

   #ifdef CH_V_PLUGIN
    if (this->wcollision == WCOLLISION_YCOLLIDE)
    {
        R3OBJ* target;
        if (groundobj)
            target = groundobj;
        else
            target = ((ChExternalObjectR3D*)GetBody2()->GetExternalObject())->Get_r3d_object();

        R3OBJ* renderer = NULL;
        if (!renderer)
            if(!(renderer = R3ObjectCreate(R3CLID_FRENGINE, R3TAG_END))) return;
        // describe scene to rendering engine
        if(!(int)R3SendMsgA(renderer, R3FRM_BEGINWORLD, NULL)) return;
        R3SendMsg(target, R3PRIMM_RENDER,
            R3PRIMA_RenderEngine, renderer,
            R3PRIMA_RenderAlways, TRUE,
            R3TAG_END);
        if(!(int)R3SendMsgA(renderer, R3FRM_ENDWORLD, NULL)) return;
        // optimize octree??
        //R3SendMsgA(renderer, R3FRM_OPTIMIZEHIT, NULL);

        Vector direction, origin, hitpoint, m_uv;
        direction = Vnorm(Vmul(vgroundrad, -1.0));
        origin = Vadd(spindle_pos, Vmul(vgroundrad, 1.001) );  // start ray from circumference toward center (speed optimization)
        int hit_ok;

        // launch intersection ray!
        hit_ok =(int) R3SendMsgA3(renderer, R3FRM_FINDHIT, &origin, &direction, &hitpoint);

        if(hit_ok &&
            (Vdot(Vsub(hitpoint,origin), direction)>= ((-0.00001)*this->Get_RigidRadius() )) )
        {
            // something has hit...
            R3SendMsgA2(renderer, R3FRM_EVALHITGEOMETRY, &m_uv, &surf_normal);
            // useful vars
            surf_normal = Vnorm (surf_normal);
            Vector real_radius;
            real_radius = Vsub(hitpoint,spindle_pos);
            double hit_length = Vlength(real_radius);
             // compute position of m2
            m2_pos = hitpoint;
             // compute rotation of m2 (ground marker)
            vz = surf_normal;
            vy = spindle_csys.Get_A_Zaxis();
            vx = Vnorm(Vcross(vy, vz));
            vy = Vnorm(Vcross(vz, vx));
            a_m2.Set_A_axis(vx, vy, vz);
            m2_rot = a_m2.Get_A_quaternion();
             // compute position and rotation of m1 (wheel marker) (not needed?)
            //m1_rot = m2_rot; // NOT!
            //m1_pos = Vadd(spindle_pos, vgroundrad ); // NOT!

            this->limit_Z->Set_active(TRUE);
        }
        else
        {
            // no hit found!!
            m2_pos = m1_pos;
            m2_rot = m1_rot;

            this->limit_Z->Set_active(FALSE);

            // force link opening!!!
            if (((ChLinkMaskLF*)mask)->Constr_Z().IsActive())
            {
                ((ChLinkMaskLF*)mask)->Constr_Z().SetMode(CONSTRAINT_FREE);
                ChangedLinkMask();
                if (system)
                    ((ChSystem*)system)->Setup(); // ndoc has changed!
            }
        }
        // suppose flat surface

		if(renderer) R3ObjectDelete(renderer);

    }
   #endif

    // MOVE "MAIN" MARKER 2 INTO UPDATED POSITION
    //
    Coordsys newmarkpos2;
    newmarkpos2.pos = m2_pos;
    newmarkpos2.rot = m2_rot;
    marker2->Impose_Abs_Coord(newmarkpos2);   // move "main" marker2 into tangent position
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL); //M_MOTION_KEYFRAMED );  // the BDF routine won't handle speed and acc.calculus of the point
    marker2->UpdateState();

    // MOVE "SLAVE" MARKER 1 INTO UPDATED POSITION
    //
    Coordsys newmarkpos1;
    Coordsys relmotioncsys = CSYSNULL;
    newmarkpos1.pos = m1_pos;
    newmarkpos1.rot = m1_rot;
    marker1->Impose_Abs_Coord(newmarkpos1);   // impose position to slave marker
    relmotioncsys.pos = m1_relvel;
    marker1->SetCoord_dt(relmotioncsys);   // impose rel.speed
    relmotioncsys.pos = m1_relacc;
    marker1->SetCoord_dtdt(relmotioncsys); // impose rel.accel.
    marker1->SetMotionType(ChMarker::M_MOTION_EXTERNAL);  //M_MOTION_KEYFRAMED ); // the BDF routine won't handle speed and acc.calculus of the point
    marker1->UpdateState();

    // THE RELATIVE MARKER POSITION:
    //
    deltaC.pos = VNULL;
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;

    deltaC.rot = QUNIT;
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}



void ChLinkWheel::UpdateForces (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

   #ifdef CH_V_PLUGIN

            // fetch tyre surface of wheel (the 2nd subobject link of marker 1)


	R3OBJ* anobj1 = ((ChExternalObjectR3D*)marker1->GetExternalObject())->Get_r3d_object();

	R3OBJ* anobj2 = ((ChExternalObjectR3D*)marker2->GetExternalObject())->Get_r3d_object();

    R3OBJ* tyreobj = Ch_GetLinkedParamObject(anobj1, 1);
            // fetch surface of contact (1st subobject link of marker 2)
    R3OBJ* surfobj  = Ch_GetLinkedParamObject(anobj2, 0);


    // Now, if ground and wheel surfaces are here, we can do the "soft wheel" model
    if (surfobj && tyreobj && (this->thickness>0.0))
    {

        R3OBJ* renderer;
        if(!(renderer = R3ObjectCreate(R3CLID_FRENGINE, R3TAG_END))) return;
        // describe scene to rendering engine
        if(!(int)R3SendMsgA(renderer, R3FRM_BEGINWORLD, NULL)) return;
        R3SendMsg(surfobj, R3PRIMM_RENDER,
            R3PRIMA_RenderEngine, renderer,
            R3PRIMA_RenderAlways, TRUE,
            R3TAG_END);
        if(!(int)R3SendMsgA(renderer, R3FRM_ENDWORLD, NULL)) return;
        // optimize octree
        R3SendMsgA(renderer, R3FRM_OPTIMIZEHIT, NULL);

        #define U_WHEEL 36
        #define V_WHEEL 4
        Vector m_normals[U_WHEEL][V_WHEEL];
        Vector m_pos[U_WHEEL][V_WHEEL];
        double m_areas[U_WHEEL][V_WHEEL];
        double m_disp[U_WHEEL][V_WHEEL];
        Vector forces_sum = VNULL;
        Vector torques_sum = VNULL;
        Vector polo_sum = this->marker1->GetAbsCoord().pos;

        for (int iu = 0; iu < U_WHEEL; iu++)
        {
            for (int iv = 0; iv < V_WHEEL; iv++)
            {
                double mu = (iu / ((double)U_WHEEL));
                double mv = (iv / ((double)V_WHEEL));
                Vector param, duparam, dvparam;
                param.x = mu + 0.5* 1.0/((double)U_WHEEL);
                param.y = mv + 0.5* 1.0/((double)V_WHEEL);
                param.z = 0;
                duparam = param; duparam.x += 0.0001;
                dvparam = param; dvparam.y += 0.0001;
                Vector ppos, dupos, dvpos, mnorm, vdu, vdv;
                Vector direction, origin, hitpoint, m_uv, surf_normal;
                R3SendMsgA3(tyreobj, R3PRIMM_EVALUATE, (void *)R3SPACE_ABSOLUTE, &param, &ppos);
                R3SendMsgA3(tyreobj, R3PRIMM_EVALUATE, (void *)R3SPACE_ABSOLUTE, &duparam, &dupos);
                R3SendMsgA3(tyreobj, R3PRIMM_EVALUATE, (void *)R3SPACE_ABSOLUTE, &dvparam, &dvpos);
                vdu = Vsub(dupos, ppos);
                vdv = Vsub(dvpos, ppos);
                mnorm = Vnorm(Vcross(vdu, vdv));
                direction = Vmul(mnorm, -1.0);
                origin = Vadd(ppos, Vmul(mnorm, this->thickness));
                m_pos[iu][iv] = ppos;
                m_normals[iu][iv] = mnorm;
                m_areas[iu][iv] = (1.0/0.0001) * (1.0/0.0001) * Vlength(Vcross(vdu, vdv));
                m_disp[iu][iv] = 0.0;
                // findhit
                if(Vdot(direction,VECT_Y) < -0.5)  // optimization: check only toward xz plane
                 if(R3SendMsgA3(renderer, R3FRM_FINDHIT, &origin, &direction, &hitpoint))
                {
                    R3SendMsgA2(renderer, R3FRM_EVALHITGEOMETRY, &m_uv, &surf_normal);
                    surf_normal = Vnorm (surf_normal);
                    if (Vdot(mnorm,surf_normal)<0.0)
                        surf_normal = Vmul(surf_normal, -1.0);
                    if (Vdot(mnorm,surf_normal)<0.0)
                        surf_normal = Vmul(surf_normal, -1.0);
                    m_normals[iu][iv] = surf_normal;

                    m_disp[iu][iv] = this->thickness - Vlength(Vsub(origin, hitpoint));
                    if (m_disp[iu][iv] < 0.0)
                           m_disp[iu][iv] = 0.0;
//-
                    if (m_disp[iu][iv] > 0.0)
                    {

                        Vector vrad_force, vtan_force, vtotal_force;
                        Vector vrel_speed, vtan_speed, vrad_speed;
                        double m_rad_force, m_rad_speed;
                        // radial compression force
                        m_rad_force = (m_areas[iu][iv] * m_disp[iu][iv] *
                                         this->rad_k * this->rad_k_def->Get_y(m_disp[iu][iv]) );
                        // radial damping force , proportional to orthogonal speed of compression
                        vrel_speed = Vsub( Body2->PointSpeedLocalToParent(Body2->Point_World2Body(m_pos[iu][iv])),
                                           Body1->PointSpeedLocalToParent(Body1->Point_World2Body(m_pos[iu][iv]))  ) ;
                        m_rad_speed = Vdot(m_normals[iu][iv], vrel_speed);
                        vrad_speed = Vmul(m_normals[iu][iv], m_rad_speed);
                        vtan_speed = Vsub(vrel_speed, vrad_speed);
                        m_rad_force +=  m_areas[iu][iv] * this->rad_r * m_rad_speed;

                        if (m_rad_force <= 0) m_rad_force = 0; // no radial traction force, only compression.

                        vrad_force = Vmul (m_normals[iu][iv], m_rad_force);
                        vtan_force = Vmul (Vnorm(vtan_speed), m_rad_force * this->friction);
                        vtotal_force = Vadd(vrad_force, vtan_force);

                        forces_sum = Vadd(forces_sum, vtotal_force);
                        torques_sum = Vadd(torques_sum, Vcross(vtotal_force, Vsub(m_pos[iu][iv], polo_sum)));

                        this->Body1->Add_as_lagrangian_force(
                            vtotal_force,               // Force in abs space! (as seen from Body1)
                            m_pos[iu][iv],          // application point
                            FALSE,                  // reference: absolute
                            Qf1);                   // store += the resulting lagrangian torque;

                        this->Body2->Add_as_lagrangian_force(
                            Vmul(vtotal_force, -1.0),   // Force in abs space! (as seen from Body2, F2=-F1)
                            m_pos[iu][iv],          // application point
                            FALSE,                  // reference: absolute
                            Qf2);                   // store += the resulting lagrangian torque;

                    }

                }
                else
                    m_disp[iu][iv] = 0.0;
            }
        }

        if(renderer) R3ObjectDelete(renderer);

    } // end soft wheel model

   #endif

    // COMPUTE SLIP FORCES

    Vector mvradius = Vsub(marker2->GetAbsCoord().pos, this->spindle_pos);

    // point of contact on Body1: absolute speed
    Vector vp1 = Vadd (Body1->GetCoord_dt().pos,
                       Body1->GetA_dt().Matr_x_Vect(marker1->GetCoord().pos) );

    // point of contact on Body2: absolute speed
    Vector vp2 = Vadd (Body2->GetCoord_dt().pos,
                       Body2->GetA_dt().Matr_x_Vect(marker2->GetCoord().pos) );

    Vector mabs_slip = Vsub(vp1, vp2);

    ChMatrix33<> mtra1;
    mtra1.Set_A_quaternion(marker1->GetAbsCoord().rot);
    ChMatrix33<> mtra2;
    mtra2.Set_A_quaternion(marker2->GetAbsCoord().rot);



    // skip further computations if wheel do not touch ground
    //
    if (Vlength(mvradius) > this->radius*1.01)
    {
        slipping = f_slip = l_slip = derive_angle = tforce = f_tforce = l_tforce = curr_friction = 0;
        return;     // <<<<<
    }

    // effect of imposed rotation:
    //
    if (speed_handled == 1)    // set artificial speed
    {   angle  = wheel_rotation->Get_y(ChTime);
        angle_dt = wheel_rotation->Get_y_dx(ChTime);
        angle_dtdt =  wheel_rotation->Get_y_dxdx(ChTime);
    }
    if (speed_handled == 2)    // set artificial speed (function already represents dy/dx mode)
    {   angle  = 0; // ***TO DO*** wheel_rotation->Get_y_Ix(ChTime, initial_rot);
        angle_dt = wheel_rotation->Get_y(ChTime);
        angle_dtdt =  wheel_rotation->Get_y_dx(ChTime);
    }
    if (speed_handled)
        mabs_slip = Vadd( mabs_slip,        // s = s + w x R
                        Vcross( Vmul(this->spindle_csys.Get_A_Zaxis(), angle_dt),   // w
                        mvradius)   // R
                        );
 
    Vector mrel_slip2 = mtra2.MatrT_x_Vect(mabs_slip);
    //Vector mrel_slip1 = mtra1.MatrT_x_Vect(mabs_slip);

    this->slipping = Vlength(mabs_slip);

    this->l_slip = mrel_slip2.y;
    this->f_slip = mrel_slip2.x;

    double norm_force = 0.0;
    if (this->react)
        norm_force += this->GetReact()->GetElement(0,0);  // N = constraint reaction
//if (norm_force >0.01)
//  R3Error("pos force %g", norm_force);

    this->curr_friction = this->friction *
                   this->fri_spe->Get_y(slipping) *
                   this->fri_norm->Get_y(norm_force);

    this->tforce = this->curr_friction * norm_force;  // Ft = u() *Fn
    tforce = fabs(tforce);

    Vector m_force2 = Vmul( Vnorm(mrel_slip2), (tforce * -1.0) );
    m_force2.z= 0; // Ft_z should be already near zero, in m2 csys, but imposed for safety.

    Vector mabs_slipforce = mtra2.Matr_x_Vect(m_force2);
    //Vector mabs_slipforce = Vmul( Vnorm(mabs_slip), (tforce * -1.0) );

    this->l_tforce = m_force2.y;
    this->f_tforce = m_force2.x;

/* ***should use only C_force and C_torque*** .... Qf1 and Qf2 are deprecated ... TO DO
    this->Body1->Add_as_lagrangian_force(
        mabs_slipforce,         // Force in abs space! (as seen from Body1)
        marker2->GetAbsCoord().pos,  // application point: marker 2 (the ground contact point)
        FALSE,                  // reference: absolute
        Qf1);                   // store += the resulting lagrangian force/torque;

    this->Body2->Add_as_lagrangian_force(
        Vmul(mabs_slipforce, -1.0), // Force in abs space! (as seen from Body2, F2=-F1)
        marker2->GetAbsCoord().pos,  // application point: marker 2 (the ground contact point)
        FALSE,                  // reference: absolute
        Qf2);                   // store += the resulting lagrangian force/torque;
*/
}

void ChLinkWheel::Set_wheel_rotation(ChFunction* m_funct)
{
    if (wheel_rotation) delete wheel_rotation;  wheel_rotation = m_funct;
}
void ChLinkWheel::Set_fri_spe(ChFunction* m_funct)
{
    if (fri_spe) delete fri_spe;    fri_spe = m_funct;
}
void ChLinkWheel::Set_fri_norm(ChFunction* m_funct)
{
    if (fri_norm) delete fri_norm;  fri_norm = m_funct;
}
void ChLinkWheel::Set_rad_k_def(ChFunction* m_funct)
{
    if (rad_k_def) delete rad_k_def; rad_k_def = m_funct;
}



// FILE I/O
//

void ChLinkWheel::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << speed_handled;
    mstream << radius;
    mstream << thickness;
    mstream << friction;
    mstream << allow_sticking;
    mstream << slip_treshold;
    mstream << static_friction;
    mstream << unilateral;
    mstream << pneus_krp;
    mstream << rad_k;
    mstream << rad_r;
    mstream << rad_p;
    mstream << pneus_h;
	mstream.AbstractWrite(wheel_rotation);
	mstream.AbstractWrite(fri_spe);
    mstream.AbstractWrite(fri_norm);
	mstream.AbstractWrite(rad_k_def);
    mstream << wcollision;
}

void ChLinkWheel::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

	    // To restore mask array when loading finishes, (maybe following data loading changes mask..)
    ChLinkMask* moriginalmask = this->mask->NewDuplicate();

		// stream in all member data
	ChFunction* ffoo;
	mstream >> speed_handled;
    mstream >> radius;
    mstream >> thickness;
    mstream >> friction;
    mstream >> allow_sticking;
    mstream >> slip_treshold;
    mstream >> static_friction;
    mstream >> unilateral;
    mstream >> pneus_krp;
    mstream >> rad_k;
    mstream >> rad_r;
    mstream >> rad_p;
    mstream >> pneus_h;
	mstream.AbstractReadCreate(&ffoo);	Set_wheel_rotation(ffoo);
	mstream.AbstractReadCreate(&ffoo);	Set_fri_spe(ffoo);
    mstream.AbstractReadCreate(&ffoo);	Set_fri_norm(ffoo);
	mstream.AbstractReadCreate(&ffoo);	Set_rad_k_def(ffoo);
    mstream >> wcollision;

	this->mask->Copy(moriginalmask);
    this->ChangedLinkMask();
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


