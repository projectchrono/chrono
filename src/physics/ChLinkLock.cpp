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
//   ChLinkLock.cpp
//
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLinkLock.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkLock> a_registration_ChLinkLock;


                // BUILDERS
ChLinkLock::ChLinkLock ()
{
    type = LNK_SPHERICAL;       // initializes type

    relC = CSYSNORM;        // custom variables
    relC_dt = CSYSNULL;
    relC_dtdt = CSYSNULL;
    deltaC = CSYSNORM;
    deltaC_dt = CSYSNULL;
    deltaC_dtdt = CSYSNULL;
    Ct_temp = CSYSNULL;
                            // matrices used by lock formulation
    Cq1_temp    = new ChMatrixDynamic<>(7, BODY_QDOF);
    Cq2_temp    = new ChMatrixDynamic<>(7, BODY_QDOF);
    Qc_temp     = new ChMatrixDynamic<>(7, 1);

    motion_X =   new ChFunction_Const (0);  // default: no motion
    motion_Y =   new ChFunction_Const (0);
    motion_Z =   new ChFunction_Const (0);
    motion_ang  = new ChFunction_Const (0);
    motion_ang2 = new ChFunction_Const (0);
    motion_ang3 = new ChFunction_Const (0);
    motion_axis = VECT_Z;
    angleset = ANGLESET_ANGLE_AXIS;

    limit_X = new ChLinkLimit;    // default: inactive limits
    limit_Y = new ChLinkLimit;
    limit_Z = new ChLinkLimit;
    limit_Rx = new ChLinkLimit;
    limit_Ry = new ChLinkLimit;
    limit_Rz = new ChLinkLimit;
    limit_D = new ChLinkLimit;
    limit_Rp = new ChLinkLimit;   // the polar limit;
    limit_Rp->Set_polar(TRUE);

    if (mask) delete mask;          // delete the class mask created by base constructor, and,
    mask = new ChLinkMaskLF();        // create the LF-mask (the extended version,for lock-formulation)
                                    // instead.

    BuildLinkType (LNK_SPHERICAL);  // default type: spherical link
                                    // Sets the mask
                                    // Sets up all the matrices and  n. of DOC and DOF,
}


            // DESTROYER
ChLinkLock::~ChLinkLock ()
{
    if (Cq1_temp)   delete Cq1_temp;
    if (Cq2_temp)   delete Cq2_temp;
    if (Qc_temp)    delete Qc_temp;

    if (motion_X) delete motion_X;
    if (motion_Y) delete motion_Y;
    if (motion_Z) delete motion_Z;
    if (motion_ang)  delete motion_ang;
    if (motion_ang2) delete motion_ang2;
    if (motion_ang3) delete motion_ang3;

    if (limit_X) delete limit_X;
    if (limit_Y) delete limit_Y;
    if (limit_Z) delete limit_Z;
    if (limit_Rx) delete limit_Rx;
    if (limit_Ry) delete limit_Ry;
    if (limit_Rz) delete limit_Rz;
    if (limit_Rp) delete limit_Rp;
    if (limit_D) delete limit_D;

    // jacobians etc. are deleted by base class, which also calls DestroyLinkType()
}

void ChLinkLock::Copy(ChLinkLock* source)
{
    // first copy the parent class data...
    //
    ChLinkMasked::Copy(source);

	type = source->type;

    limit_X->Copy (source->limit_X);    // copy limits
    limit_Y->Copy (source->limit_Y);
    limit_Z->Copy (source->limit_Z);
    limit_Rx->Copy (source->limit_Rx);
    limit_Ry->Copy (source->limit_Ry);
    limit_Rz->Copy (source->limit_Rz);
    limit_Rp->Copy (source->limit_Rp);
    limit_D->Copy (source->limit_D);

    deltaC = source->deltaC;
    deltaC_dt = source->deltaC_dt;
    deltaC_dtdt = source->deltaC_dtdt;
    relC = source->relC;
    relC_dt = source->relC_dt;
    relC_dtdt = source->relC_dtdt;
    Ct_temp = source->Ct_temp;

    if (motion_X)   delete motion_X;    // replace and copy functions
    if (motion_Y)   delete motion_Y;
    if (motion_Z)   delete motion_Z;
    if (motion_ang) delete motion_ang;
    if (motion_ang2) delete motion_ang2;
    if (motion_ang3) delete motion_ang3;

    motion_X =   source->motion_X->new_Duplicate();
    motion_Y =   source->motion_Y->new_Duplicate();
    motion_Z =   source->motion_Z->new_Duplicate();
    motion_ang = source->motion_ang->new_Duplicate();
    motion_ang2 = source->motion_ang2->new_Duplicate();
    motion_ang3 = source->motion_ang3->new_Duplicate();

    motion_axis = source->motion_axis;
    angleset = source->angleset;
}


ChLink* ChLinkLock::new_Duplicate ()   // inherited classes:  Link* MyInheritedLink::new_Duplicate()
{
    ChLinkLock* m_l;
    m_l = new ChLinkLock;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}


void ChLinkLock::BuildLinkType (int link_type)
{
    type = link_type;

    ChLinkMaskLF m_mask;

    // Note that the SetLockMask() sets the costraints for the
    // link coordinates: (X,Y,Z, E0, E1,E2,E3, P, D);

    // default.. free
    m_mask.SetLockMask (false,false,false,
                        false,false,false,false,
                        false,false);

    if (type == LNK_FREE)
        m_mask.SetLockMask (false,false,false,
                        false,false,false,false,
                        false,false);

    if (type == LNK_LOCK)
        m_mask.SetLockMask (true, true, true,
                        false,true, true, true,
                        false,false);

    if (type == LNK_SPHERICAL)
        m_mask.SetLockMask (true, true, true,
                        false,false,false,false,
                        false,false);

    if (type == LNK_RACKPIN)
        m_mask.SetLockMask (false,false,false,
                        false,false,false,false,
                        false,false);

    if (type == LNK_POINTPLANE)
        m_mask.SetLockMask (false,false,true,
                        false,false,false,false,
                        false,false);

    if (type == LNK_POINTLINE)
        m_mask.SetLockMask (false,true, true,
                        false,false,false,false,
                        false,false);

    if (type == LNK_REVOLUTE)
        m_mask.SetLockMask (true, true, true,
                        false,true, true,false,
                        false,false);

    if (type == LNK_CYLINDRICAL)
        m_mask.SetLockMask (true, true, false,
                        false,true, true, false,
                        false,false);

    if (type == LNK_PRISMATIC)
        m_mask.SetLockMask (true, true, false,
                        false,true, true, true,
                        false,false);

    if (type == LNK_PLANEPLANE)
        m_mask.SetLockMask (false,false,true,
                        false,true, true, false,
                        false,false);

    if (type == LNK_OLDHAM)
        m_mask.SetLockMask (false,false,true,
                        false,true, true, true,
                        false,false);

    if (type == LNK_HOOK)
        m_mask.SetLockMask (true, true, true,
                        false,false,true, false,
                        false,false);

    if (type == LNK_UNIVERSAL)
        m_mask.SetLockMask (true, true, true,
                        false,false, false, true,
                        false,false);

    if (type == LNK_CARDANO)
        m_mask.SetLockMask (false, false, false,
                        false,false, false, true,
                        false,false);

    if (type == LNK_ALIGN)
        m_mask.SetLockMask (false,false,false,
                        false,true, true, true,
                        false,false);

    if (type == LNK_PARALLEL)
        m_mask.SetLockMask (false,false,false,
                        false,true, true, false,
                        false,false);

    if (type == LNK_PERPEND)
        m_mask.SetLockMask (false,false,false,
                        false,true, false,true,
                        false,false);

    BuildLink (&m_mask);    // , TRUE);
}


void ChLinkLock::ChangeLinkType (int new_link_type)
{
    DestroyLink ();
    BuildLinkType (new_link_type);

            // Also...
            // reset all motions and limits!

    if (motion_X) delete motion_X;
    if (motion_Y) delete motion_Y;
    if (motion_Z) delete motion_Z;
    if (motion_ang)  delete motion_ang;
    if (motion_ang2) delete motion_ang2;
    if (motion_ang3) delete motion_ang3;

    motion_X =   new ChFunction_Const (0);  // default: no motion
    motion_Y =   new ChFunction_Const (0);
    motion_Z =   new ChFunction_Const (0);
    motion_ang  = new ChFunction_Const (0);
    motion_ang2 = new ChFunction_Const (0);
    motion_ang3 = new ChFunction_Const (0);
    motion_axis = VECT_Z;
    angleset = ANGLESET_ANGLE_AXIS;

    if (limit_X) delete limit_X;
    if (limit_Y) delete limit_Y;
    if (limit_Z) delete limit_Z;
    if (limit_Rx) delete limit_Rx;
    if (limit_Ry) delete limit_Ry;
    if (limit_Rz) delete limit_Rz;
    if (limit_Rp) delete limit_Rp;
    if (limit_D) delete limit_D;

    limit_X = new ChLinkLimit;    // default: inactive limits
    limit_Y = new ChLinkLimit;
    limit_Z = new ChLinkLimit;
    limit_Rx = new ChLinkLimit;
    limit_Ry = new ChLinkLimit;
    limit_Rz = new ChLinkLimit;
    limit_D = new ChLinkLimit;
    limit_Rp = new ChLinkLimit;   // the polar limit;
    limit_Rp->Set_polar(TRUE);
}


void ChLinkLock::Set2Dmode(int mode)
{
    ChLinkMaskLF* mmask = (ChLinkMaskLF*) this->mask;

    if (mmask->Set2Dmode(mode) >0)
    {
        ChangedLinkMask ();
    }
}


// setup the functions when user changes them.

void ChLinkLock::SetMotion_X  (ChFunction* m_funct)
{
    if (motion_X) delete motion_X;
    motion_X = m_funct;
}

void ChLinkLock::SetMotion_Y  (ChFunction* m_funct)
{
    if (motion_Y) delete motion_Y;
    motion_Y = m_funct;
}

void ChLinkLock::SetMotion_Z  (ChFunction* m_funct)
{
    if (motion_Z) delete motion_Z;
    motion_Z = m_funct;
}

void ChLinkLock::SetMotion_ang    (ChFunction* m_funct)
{
    if (motion_ang) delete motion_ang;
    motion_ang = m_funct;
}

void ChLinkLock::SetMotion_ang2   (ChFunction* m_funct)
{
    if (motion_ang2) delete motion_ang2;
    motion_ang2 = m_funct;
}

void ChLinkLock::SetMotion_ang3   (ChFunction* m_funct)
{
    if (motion_ang3) delete motion_ang3;
    motion_ang3 = m_funct;
}

void ChLinkLock::SetMotion_axis (Vector m_axis)
{
    motion_axis = m_axis;
}









////////////////////////////////////
////////////////////////////////////
///
///    UPDATING PROCEDURES



/////////   1-   UPDATE TIME
/////////

void ChLinkLock::UpdateTime (double time)
{
	ChLinkMasked::UpdateTime(time);

    double ang, ang_dt, ang_dtdt;

        // If some limit is provided, the delta values may have been
        // changed by limits themselves, so no further modifications by motion laws..
    if (limit_X->Get_active() ||
        limit_Y->Get_active() ||
        limit_Z->Get_active() ||
        limit_Rx->Get_active() ||
        limit_Ry->Get_active() ||
        limit_Rz->Get_active())
    return ;

        // Update motion position/speed/acceleration by motion laws
        // as expressed by specific link CH funcions
    deltaC.pos.x= motion_X->Get_y(time);
    deltaC_dt.pos.x= motion_X->Get_y_dx(time);
    deltaC_dtdt.pos.x= motion_X->Get_y_dxdx(time);

    deltaC.pos.y= motion_Y->Get_y(time);
    deltaC_dt.pos.y= motion_Y->Get_y_dx(time);
    deltaC_dtdt.pos.y= motion_Y->Get_y_dxdx(time);

    deltaC.pos.z= motion_Z->Get_y(time);
    deltaC_dt.pos.z= motion_Z->Get_y_dx(time);
    deltaC_dtdt.pos.z= motion_Z->Get_y_dxdx(time);

    switch (angleset) {
    case ANGLESET_ANGLE_AXIS:
        ang     = motion_ang->Get_y(time);
        ang_dt  = motion_ang->Get_y_dx(time);
        ang_dtdt= motion_ang->Get_y_dxdx(time);

        if ((ang !=0)||(ang_dt !=0)||(ang_dtdt !=0))
        {
            deltaC.rot = Q_from_AngAxis (ang, motion_axis);
            deltaC_dt.rot = Qdt_from_AngAxis (deltaC.rot, ang_dt, motion_axis);
            deltaC_dtdt.rot = Qdtdt_from_AngAxis (ang_dtdt, motion_axis, deltaC.rot, deltaC_dt.rot);
        }
        else
        {
            deltaC.rot = QUNIT;
            deltaC_dt.rot = QNULL;
            deltaC_dtdt.rot = QNULL;
        }
        break;
    case ANGLESET_EULERO:
    case ANGLESET_CARDANO:
    case ANGLESET_HPB:
    case ANGLESET_RXYZ:
        Vector vangles, vangles_dt, vangles_dtdt;
        vangles.x = motion_ang->Get_y(time);
        vangles.y = motion_ang2->Get_y(time);
        vangles.z = motion_ang3->Get_y(time);
        vangles_dt.x = motion_ang->Get_y_dx(time);
        vangles_dt.y = motion_ang2->Get_y_dx(time);
        vangles_dt.z = motion_ang3->Get_y_dx(time);
        vangles_dtdt.x = motion_ang->Get_y_dxdx(time);
        vangles_dtdt.y = motion_ang2->Get_y_dxdx(time);
        vangles_dtdt.z = motion_ang3->Get_y_dxdx(time);
        deltaC.rot = Angle_to_Quat(angleset, &vangles);
        deltaC_dt.rot = AngleDT_to_QuatDT(angleset, &vangles_dt, &deltaC.rot);
        deltaC_dtdt.rot = AngleDTDT_to_QuatDTDT(angleset, &vangles_dtdt, &deltaC.rot);
        break;
    }
}



/////////   2-   UPDATE RELATIVE MARKER COORDINATES
/////////

void ChLinkLock::UpdateRelMarkerCoords()
{
    // FOR ALL THE 6(or3) COORDINATES OF RELATIVE MOTION OF THE TWO MARKERS.
    //  Also set some static vectors/quaternions which will be used later in the
    // UpdateState function for the Lock-Formulation method (this customization,
    // happens only for speed readsons, otherwise the base UpdateRelMarkerCoords()
    // could be sufficient)

    PQw = Vsub (marker1->GetAbsCoord().pos, marker2->GetAbsCoord().pos);
    PQw_dt = Vsub (marker1->GetAbsCoord_dt().pos, marker2->GetAbsCoord_dt().pos);
    PQw_dtdt = Vsub (marker1->GetAbsCoord_dtdt().pos, marker2->GetAbsCoord_dtdt().pos);

    dist    = Vlenght (PQw);                // distance between origins, modulus
    dist_dt = Vdot (Vnorm(PQw), PQw_dt);    // speed between origins, modulus.

    Vector vtemp1;       // for intermediate calculus
    Vector vtemp2;
    Quaternion qtemp1;
    ChMatrixNM<double,3,4> relGw;
    Quaternion temp1=marker1->GetCoord_dt().rot;
    Quaternion temp2=marker2->GetCoord_dt().rot;

    if (Qnotnull(temp2) || Qnotnull(temp1))
    {
      q_AD =                    //  q'qqq + qqqq'
       Qadd (
        Qcross (
          Qconjugate(marker2->GetCoord_dt().rot),
        Qcross (
          Qconjugate(marker2->GetBody()->GetCoord().rot),
        Qcross (
          (marker1->GetBody()->GetCoord().rot),
          (marker1->GetCoord().rot)
          ))),
        Qcross (
          Qconjugate(marker2->GetCoord().rot),
        Qcross (
          Qconjugate(marker2->GetBody()->GetCoord().rot),
        Qcross (
          (marker1->GetBody()->GetCoord().rot),
          (marker1->GetCoord_dt().rot)
          )))
        );
    }
    else
        q_AD=QNULL;

    q_BC =                  // qq'qq + qqq'q
       Qadd (
        Qcross (
          Qconjugate(marker2->GetCoord().rot),
        Qcross (
          Qconjugate(marker2->GetBody()->GetCoord_dt().rot),
        Qcross (
          (marker1->GetBody()->GetCoord().rot),
          (marker1->GetCoord().rot)
          ))),
        Qcross (
          Qconjugate(marker2->GetCoord().rot),
        Qcross (
          Qconjugate(marker2->GetBody()->GetCoord().rot),
        Qcross (
          (marker1->GetBody()->GetCoord_dt().rot),
          (marker1->GetCoord().rot)
          )))
       );

                            // q_8 = q''qqq + 2q'q'qq + 2q'qq'q + 2q'qqq'
                            //     + 2qq'q'q + 2qq'qq' + 2qqq'q' + qqqq''
    temp2=marker2->GetCoord_dtdt().rot;
    if (Qnotnull(temp2))
    q_8=Qcross(Qconjugate(marker2->GetCoord_dtdt().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord().rot)));        // q_dtdt'm2 * q'o2 * q,o1 * q,m1
    else
    q_8 = QNULL;
    temp1=marker1->GetCoord_dtdt().rot;
    if (Qnotnull(temp1))
    {
     qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord_dtdt().rot)));   // q'm2 * q'o2 * q,o1 * q_dtdt,m1
     q_8= Qadd (q_8, qtemp1);
    }
    temp2=marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2))
    {
     qtemp1=Qcross(Qconjugate(marker2->GetCoord_dt().rot),
        Qcross(Qconjugate(Body2->GetCoord_dt().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord().rot)));
     qtemp1= Qscale (qtemp1, 2);                // 2( q_dt'm2 * q_dt'o2 * q,o1 * q,m1)
     q_8= Qadd (q_8, qtemp1);
    }
    temp2=marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2))
    {
     qtemp1=Qcross(Qconjugate(marker2->GetCoord_dt().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord_dt().rot,
        marker1->GetCoord().rot)));
     qtemp1= Qscale (qtemp1, 2);                // 2( q_dt'm2 * q'o2 * q_dt,o1 * q,m1)
     q_8= Qadd (q_8, qtemp1);
    }
    temp1=marker1->GetCoord_dt().rot;
    temp2=marker2->GetCoord_dt().rot;
    if (Qnotnull(temp2) && Qnotnull(temp1))
    {
     qtemp1=Qcross(Qconjugate(marker2->GetCoord_dt().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord_dt().rot)));
     qtemp1= Qscale (qtemp1, 2);                // 2( q_dt'm2 * q'o2 * q,o1 * q_dt,m1)
     q_8= Qadd (q_8, qtemp1);
    }

    qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord_dt().rot),
        Qcross(Body1->GetCoord_dt().rot,
        marker1->GetCoord().rot)));
    qtemp1= Qscale (qtemp1, 2);             // 2( q'm2 * q_dt'o2 * q_dt,o1 * q,m1)
    q_8= Qadd (q_8, qtemp1);
    temp1=marker1->GetCoord_dt().rot;
    if (Qnotnull(temp1))
    {
    qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord_dt().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord_dt().rot)));
    qtemp1= Qscale (qtemp1, 2);             // 2( q'm2 * q_dt'o2 * q,o1 * q_dt,m1)
    q_8= Qadd (q_8, qtemp1);
    }
    temp1=marker1->GetCoord_dt().rot;
    if (Qnotnull(temp1))
    {
    qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord_dt().rot,
        marker1->GetCoord_dt().rot)));
    qtemp1= Qscale (qtemp1, 2);             // 2( q'm2 * q'o2 * q_dt,o1 * q_dt,m1)
    q_8= Qadd (q_8, qtemp1);
    }

                                // q_4 = [Adtdt]'[A]'q + 2[Adt]'[Adt]'q
                                //       + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt
	ChMatrix33<> m2_Rel_A_dt;
	marker2->Compute_Adt(m2_Rel_A_dt);
	ChMatrix33<> m2_Rel_A_dtdt;
	marker2->Compute_Adtdt(m2_Rel_A_dtdt);

    vtemp1=Body2->GetA_dt().MatrT_x_Vect(PQw);
    vtemp2=m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
    q_4= Vmul (vtemp2, 2);          // 2[Aq_dt]'[Ao2_dt]'*Qpq,w

    vtemp1=Body2->GetA()->MatrT_x_Vect(PQw_dt);
    vtemp2=m2_Rel_A_dt.MatrT_x_Vect(vtemp1);
    vtemp2= Vmul (vtemp2, 2);       // 2[Aq_dt]'[Ao2]'*Qpq,w_dt
    q_4= Vadd (q_4, vtemp2);

    vtemp1=Body2->GetA_dt().MatrT_x_Vect(PQw_dt);
    vtemp2=marker2->GetA()->MatrT_x_Vect(vtemp1);
    vtemp2= Vmul (vtemp2, 2);       // 2[Aq]'[Ao2_dt]'*Qpq,w_dt
    q_4= Vadd (q_4, vtemp2);

    vtemp1=Body2->GetA()->MatrT_x_Vect(PQw);
    vtemp2=m2_Rel_A_dtdt.MatrT_x_Vect(vtemp1);
    q_4= Vadd (q_4, vtemp2);        //  [Aq_dtdt]'[Ao2]'*Qpq,w



            // ----------- RELATIVE MARKER COORDINATES

            // relM.pos
    relM.pos =
        marker2->GetA()->MatrT_x_Vect(
        Body2->GetA()->MatrT_x_Vect(PQw));

            // relM.rot
    relM.rot =
        Qcross (
          Qconjugate(marker2->GetCoord().rot),
        Qcross (
          Qconjugate(marker2->GetBody()->GetCoord().rot),
        Qcross (
          (marker1->GetBody()->GetCoord().rot),
          (marker1->GetCoord().rot)
          )));

            // relM_dt.pos
    relM_dt.pos =
        Vadd (
        Vadd (
        m2_Rel_A_dt.MatrT_x_Vect(
        Body2->GetA()->MatrT_x_Vect(PQw)) ,
        marker2->GetA()->MatrT_x_Vect(
        Body2->GetA_dt().MatrT_x_Vect(PQw))),
        marker2->GetA()->MatrT_x_Vect(
        Body2->GetA()->MatrT_x_Vect(PQw_dt)));

            // relM_dt.rot
    relM_dt.rot =
       Qadd ( q_AD, q_BC );

            // relM_dtdt.pos
    relM_dtdt.pos =
        Vadd (
        Vadd (
        marker2->GetA()->MatrT_x_Vect(
         Body2->GetA_dtdt().MatrT_x_Vect(PQw)),
        marker2->GetA()->MatrT_x_Vect(
         Body2->GetA()->MatrT_x_Vect(PQw_dtdt))),
        q_4);


            // relM_dtdt.rot
    qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord_dtdt().rot),
        Qcross(Body1->GetCoord().rot,
        marker1->GetCoord().rot)));    // ( q'm2 * q_dtdt'o2 * q,o1 * q,m1)
    relM_dtdt.rot= Qadd (q_8, qtemp1);
    qtemp1=Qcross(Qconjugate(marker2->GetCoord().rot),
        Qcross(Qconjugate(Body2->GetCoord().rot),
        Qcross(Body1->GetCoord_dtdt().rot,
        marker1->GetCoord().rot)));    // ( q'm2 * q'o2 * q_dtdt,o1 * q,m1)
    relM_dtdt.rot= Qadd (relM_dtdt.rot, qtemp1);    // = q_8 + qq''qq + qqq''q


                // ... and also "user-friendly" relative coordinates:

            // relAngle and relAxis
    Q_to_AngAxis (&relM.rot, &relAngle, &relAxis);
    // flip rel rotation axis if jerky sign
    if (relAxis.z < 0)
    {
        relAxis = Vmul (relAxis, -1);
        relAngle = -relAngle;
    }
            // rotation axis
    relRotaxis = Vmul(relAxis, relAngle);
            // relWvel
    ChFrame<>::SetMatrix_Gw(relGw, relM.rot); // relGw.Set_Gw_matrix(relM.rot);
    relWvel = relGw.Matr34_x_Quat (relM_dt.rot);
            // relWacc
    relWacc = relGw.Matr34_x_Quat (relM_dtdt.rot);

}





/////////   4-   UPDATE STATE
/////////

void ChLinkLock::UpdateState ()
{


                        // ---------------------
                        // Updates Cq1_temp, Cq2_temp, Qc_temp,
                        // etc., i.e. all LOCK-FORMULATION temp.matrices
                        // ---------------------

    ChVector<> vtemp1;       // for intermediate calculus
    ChVector<> vtemp2;

    ChMatrix33<> mtemp1;
    ChMatrix33<> mtemp2;
    ChMatrix33<> mtemp3;
    ChMatrixNM<double,4,4> mtempQ1;
    ChMatrixNM<double,4,4> mtempQ2;

    ChMatrix33<> CqxT;     // the 3x3 piece of Cq_temp for trasl. link,   trasl.coords,
    ChMatrixNM<double,3,4> CqxR;// the 3x4 piece of Cq_temp for trasl. link,   rotat. coords,
    ChMatrixNM<double,4,4> CqrR;// the 4x4 piece of Cq_temp for rotat..link,   rotat. coords,
    ChVector<>  Qcx;     // the 3x1 vector of Qc     for trasl. link
    ChQuaternion<> Qcr;  // the 4x1 quaternion of Qc for rotat. link

    // [Cq_temp]= [[CqxT] [CqxR]]     {Qc_temp} ={[Qcx]}
    //            [[ 0  ] [CqrR]]                {[Qcr]}

            // ----------- SOME PRECALCULATED VARIABLES, to optimize speed

    ChMatrix33<> P1star;       // [P] star matrix of rel pos of mark1
    P1star.Set_X_matrix(marker1->GetCoord().pos);
    ChMatrix33<> Q2star;       // [Q] star matrix of rel pos of mark2
    Q2star.Set_X_matrix(marker2->GetCoord().pos);


	ChMatrixNM<double,3,4> body1Gl;
	ChMatrixNM<double,3,4> body2Gl;

	ChFrame<>::SetMatrix_Gl(body1Gl, Body1->GetCoord().rot);
	ChFrame<>::SetMatrix_Gl(body2Gl, Body2->GetCoord().rot);


            // ----------- RELATIVE LINK-LOCK COORDINATES (violations)

            // relC.pos
    relC.pos = Vsub (relM.pos, deltaC.pos);

            // relC.rot
    relC.rot = Qcross (Qconjugate(deltaC.rot), relM.rot);

            // relC_dt.pos
    relC_dt.pos = Vsub (relM_dt.pos, deltaC_dt.pos);

            // relC_dt.rot
    relC_dt.rot = Qadd (
                Qcross (Qconjugate(deltaC_dt.rot),relM.rot),
                Qcross (Qconjugate(deltaC.rot),relM_dt.rot) );

            // relC_dtdt.pos
    relC_dtdt.pos = Vsub (relM_dtdt.pos, deltaC_dtdt.pos);

            // relC_dtdt.rot
    relC_dtdt.rot = Qadd (
                    Qadd (
                Qcross (Qconjugate(deltaC_dtdt.rot),relM.rot),
                Qcross (Qconjugate(deltaC.rot),relM_dtdt.rot) ),
                Qscale (Qcross (Qconjugate(deltaC_dt.rot),relM_dt.rot),2) );


    // +++++++++ COMPUTE THE  Cq Ct Qc    matrices (temporary, for complete lock contraint)

	ChMatrix33<> m2_Rel_A_dt;
	marker2->Compute_Adt(m2_Rel_A_dt);
	ChMatrix33<> m2_Rel_A_dtdt;
	marker2->Compute_Adtdt(m2_Rel_A_dtdt);

            // ----------- PARTIAL DERIVATIVE Ct OF CONSTRAINT
    Ct_temp.pos =
        Vadd (
          m2_Rel_A_dt.MatrT_x_Vect(
          Body2->GetA()->MatrT_x_Vect(PQw)),
          marker2->GetA()->MatrT_x_Vect(
           Vsub(Body2->GetA()->MatrT_x_Vect(
            Body1->GetA()->Matr_x_Vect(marker1->GetCoord_dt().pos)),
            marker2->GetCoord_dt().pos))
          );
    Ct_temp.pos = Vsub (Ct_temp.pos,  deltaC_dt.pos);   // the deltaC contribute


    Ct_temp.rot =       // deltaC^*(q_AD) + deltaC_dt^*q_pq
        Qadd (
        Qcross (Qconjugate (deltaC.rot), q_AD),
        Qcross (Qconjugate (deltaC_dt.rot), relM.rot)
        );



            //------------ COMPLETE JACOBIANS Cq1_temp AND Cq2_temp AND Qc_temp VECTOR.

            //  JACOBIANS Cq1_temp, Cq2_temp:

    mtemp1.CopyFromMatrixT(*marker2->GetA());
    CqxT.MatrMultiplyT(mtemp1,*Body2->GetA());     // [CqxT]=[Aq]'[Ao2]'

    Cq1_temp->PasteMatrix(&CqxT,0,0);       // *- -- Cq1_temp(1-3)  =[Aqo2]

    CqxT.MatrNeg();
    Cq2_temp->PasteMatrix(&CqxT,0,0);       // -- *- Cq2_temp(1-3)  =-[Aqo2]


    mtemp1.MatrMultiply(CqxT, *Body1->GetA());
    mtemp2.MatrMultiply(mtemp1,P1star);

    CqxR.MatrMultiply   (mtemp2, body1Gl);

    Cq1_temp->PasteMatrix(&CqxR,0,3);       // -* -- Cq1_temp(4-7)


    CqxT.MatrNeg();
    mtemp1.MatrMultiply(CqxT,*Body2->GetA());
    mtemp2.MatrMultiply(mtemp1,Q2star);
    CqxR.MatrMultiply   (mtemp2, body2Gl);
    Cq2_temp->PasteMatrix(&CqxR,0,3);

    mtemp1.CopyFromMatrixT(*marker2->GetA());
    mtemp2.Set_X_matrix ( Body2->GetA()->MatrT_x_Vect(PQw));
    mtemp3.MatrMultiply (mtemp1, mtemp2);
    CqxR.MatrMultiply    (mtemp3, body2Gl);

    Cq2_temp->PasteSumMatrix(&CqxR,0,3);        // -- -* Cq1_temp(4-7)


    mtempQ1.Set_Xq_matrix(Qcross(Qconjugate(marker2->GetCoord().rot),
                                 Qconjugate(Body2->GetCoord().rot)));
    CqrR.Set_Xq_matrix(marker1->GetCoord().rot);
    CqrR.MatrXq_SemiTranspose();
    mtempQ2.MatrMultiply (mtempQ1, CqrR);
    mtempQ1.Set_Xq_matrix(Qconjugate(deltaC.rot));
    CqrR.MatrMultiply (mtempQ1, mtempQ2);

    Cq1_temp->PasteMatrix(&CqrR,3,3);       // =* == Cq1_temp(col 4-7, row 4-7)


    mtempQ1.Set_Xq_matrix(Qconjugate(marker2->GetCoord().rot));
    CqrR.Set_Xq_matrix(Qcross(Body1->GetCoord().rot,
                          marker1->GetCoord().rot));
    CqrR.MatrXq_SemiTranspose();
    CqrR.MatrXq_SemiNeg();
    mtempQ2.MatrMultiply (mtempQ1, CqrR);
    mtempQ1.Set_Xq_matrix(Qconjugate(deltaC.rot));
    CqrR.MatrMultiply (mtempQ1, mtempQ2);

    Cq2_temp->PasteMatrix(&CqrR,3,3);       // == =* Cq2_temp(col 4-7, row 4-7)


        //--------- COMPLETE Qc VECTOR

    vtemp1= Vcross ( Body1->GetWvel_loc(), Vcross(Body1->GetWvel_loc(), marker1->GetCoord().pos));
    vtemp1= Vadd (vtemp1, marker1->GetCoord_dtdt().pos);
    vtemp1= Vadd (vtemp1, Vmul (Vcross (Body1->GetWvel_loc(), marker1->GetCoord_dt().pos) , 2));
    vtemp1= Body1->GetA()->Matr_x_Vect(vtemp1);

    vtemp2= Vcross ( Body2->GetWvel_loc(), Vcross(Body2->GetWvel_loc(), marker2->GetCoord().pos));
    vtemp2= Vadd (vtemp2, marker2->GetCoord_dtdt().pos);
    vtemp2= Vadd (vtemp2, Vmul (Vcross (Body2->GetWvel_loc(), marker2->GetCoord_dt().pos) , 2));
    vtemp2= Body2->GetA()->Matr_x_Vect(vtemp2);

    vtemp1= Vsub (vtemp1, vtemp2);
    Qcx= CqxT.Matr_x_Vect (vtemp1);

    mtemp1.Set_X_matrix ( Body2->GetWvel_loc());
    mtemp2.MatrMultiply (mtemp1, mtemp1);
    mtemp3.MatrMultiply (*Body2->GetA(), mtemp2);
    mtemp3.MatrTranspose();
    vtemp1=mtemp3.Matr_x_Vect(PQw);
    vtemp2=marker2->GetA()->MatrT_x_Vect(vtemp1);  // [Aq]'[[A2][w2][w2]]'*Qpq,w
    Qcx= Vadd (Qcx, vtemp2);

    Qcx= Vadd (Qcx, q_4);             // [Adtdt]'[A]'q + 2[Adt]'[Adt]'q + 2[Adt]'[A]'qdt + 2[A]'[Adt]'qdt

    Qcx= Vsub (Qcx, deltaC_dtdt.pos); // ... - deltaC_dtdt

    Qc_temp->PasteVector(Qcx,0,0);  // * Qc_temp, for all translational coords




    Qcr = Qcross (Qconjugate(deltaC.rot), q_8);
    Qcr = Qadd (Qcr,
            Qscale (Qcross(Qconjugate(deltaC_dt.rot),relM_dt.rot),2));
    Qcr = Qadd (Qcr,
            Qcross(Qconjugate(deltaC_dtdt.rot),relM.rot)); // = deltaC'*q_8 + 2*deltaC_dt'*q_dt,po + deltaC_dtdt'*q,po

    Qc_temp->PasteQuaternion(Qcr,3,0);      // * Qc_temp, for all rotational coords


    // *** NOTE! The definitive  Qc must change sign, to be used in
    // lagrangian equation:    [Cq]*q_dtdt = Qc
    // because until now we have computed it as [Cq]*q_dtdt + "Qc" = 0,
    // but the most used form is the previous, so let's change sign!!

    Qc_temp->MatrNeg();

                        // FINALLY.....
                        // ---------------------
                        // Updates Cq1, Cq2, Qc,
                        // C, C_dt, C_dtdt, Ct.
                        // ---------------------
    int index = 0;

    ChLinkMaskLF* mmask = (ChLinkMaskLF*) this->mask;

	if (mmask->Constr_X().IsActive()) // for X costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 0,0, 1,7, index, 0);
        Cq2->PasteClippedMatrix (Cq2_temp, 0,0, 1,7, index, 0);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (0,0));

        C->SetElement(index,0,      relC.pos.x);
        C_dt->SetElement(index,0,   relC_dt.pos.x);
        C_dtdt->SetElement(index,0, relC_dtdt.pos.x);

        Ct->SetElement(index,0, Ct_temp.pos.x );

        index ++;
    }

    if (mmask->Constr_Y().IsActive()) // for Y costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 1,0, 1,7, index, 0);
        Cq2->PasteClippedMatrix (Cq2_temp, 1,0, 1,7, index, 0);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (1,0));

        C->SetElement(index,0,      relC.pos.y);
        C_dt->SetElement(index,0,   relC_dt.pos.y);
        C_dtdt->SetElement(index,0, relC_dtdt.pos.y);

        Ct->SetElement(index,0, Ct_temp.pos.y);

        index ++;
    }

    if (mmask->Constr_Z().IsActive()) // for Z costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 2,0, 1,7, index, 0);
        Cq2->PasteClippedMatrix (Cq2_temp, 2,0, 1,7, index, 0);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (2,0));

        C->SetElement(index,0,      relC.pos.z);
        C_dt->SetElement(index,0,   relC_dt.pos.z);
        C_dtdt->SetElement(index,0, relC_dtdt.pos.z);

        Ct->SetElement(index,0, Ct_temp.pos.z);

        index ++;
    }

    if (mmask->Constr_E0().IsActive())    // for E0 costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 3,3, 1,4, index, 3);
        Cq2->PasteClippedMatrix (Cq2_temp, 3,3, 1,4, index, 3);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (3,0));

        C->SetElement(index,0,      relC.rot.e0);
        C_dt->SetElement(index,0,   relC_dt.rot.e0);
        C_dtdt->SetElement(index,0, relC_dtdt.rot.e0);

        Ct->SetElement(index,0, Ct_temp.rot.e0);

        index ++;
    }

    if (mmask->Constr_E1().IsActive())   // for E1 costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 4,3, 1,4, index, 3);
        Cq2->PasteClippedMatrix (Cq2_temp, 4,3, 1,4, index, 3);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (4,0));

        C->SetElement(index,0,      relC.rot.e1);
        C_dt->SetElement(index,0,   relC_dt.rot.e1);
        C_dtdt->SetElement(index,0, relC_dtdt.rot.e1);

        Ct->SetElement(index,0, Ct_temp.rot.e1);

        index ++;
    }

    if (mmask->Constr_E2().IsActive())    // for E2 costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 5,3, 1,4, index, 3);
        Cq2->PasteClippedMatrix (Cq2_temp, 5,3, 1,4, index, 3);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (5,0));

        C->SetElement(index,0,      relC.rot.e2);
        C_dt->SetElement(index,0,   relC_dt.rot.e2);
        C_dtdt->SetElement(index,0, relC_dtdt.rot.e2);

        Ct->SetElement(index,0, Ct_temp.rot.e2);

        index ++;
    }

    if (mmask->Constr_E3().IsActive())    // for E3 costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 6,3, 1,4, index, 3);
        Cq2->PasteClippedMatrix (Cq2_temp, 6,3, 1,4, index, 3);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (6,0));

        C->SetElement(index,0,      relC.rot.e3);
        C_dt->SetElement(index,0,   relC_dt.rot.e3);
        C_dtdt->SetElement(index,0, relC_dtdt.rot.e3);

        Ct->SetElement(index,0, Ct_temp.rot.e3);

        index ++;
    }

    if (mmask->Constr_P().IsActive()) // for P (polar, conical rotation) costraint...
    {
        Cq1->PasteClippedMatrix (Cq1_temp, 6,3, 1,4, index, 3);
        Cq2->PasteClippedMatrix (Cq2_temp, 6,3, 1,4, index, 3);

        Qc->SetElement(index,0,
                      Qc_temp->GetElement (6,0));

        C->SetElement(index,0,      relC.rot.e3);
        C_dt->SetElement(index,0,   relC_dt.rot.e3);
        C_dtdt->SetElement(index,0, relC_dtdt.rot.e3);

        Ct->SetElement(index,0, Ct_temp.rot.e3);

        index ++;
    }

    if (mmask->Constr_D().IsActive()) // for D (distance costraint)
    {
        ChMatrixNM<double,3,7> Cq1tmp;
        ChMatrixNM<double,3,7> Cq2tmp;
        ChMatrixNM<double,1,7> Cq1d;
        ChMatrixNM<double,1,7> Cq2d;
        ChMatrixNM<double,3,1> mfact;
        double mQd;

        ChVector<> vfact;
        vfact= Vnorm(relM.pos);
        mfact.PasteVector(vfact,0,0);

        Cq1tmp.PasteClippedMatrix (Cq1_temp, 0,0, 3,7, 0,0);
        Cq2tmp.PasteClippedMatrix (Cq2_temp, 0,0, 3,7, 0,0);
                // Cq= (1/||relM||)*relM'*Cq_lock
        Cq1d.MatrTMultiply(mfact,Cq1tmp);
        Cq2d.MatrTMultiply(mfact,Cq2tmp);

        Cq1->PasteClippedMatrix (&Cq1d, 0,0, 1,7, index, 0);
        Cq2->PasteClippedMatrix (&Cq2d, 0,0, 1,7, index, 0);

        ChVector<> Qvect;
        Qvect.x = Qc_temp->GetElement(0,0);
        Qvect.y = Qc_temp->GetElement(1,0);
        Qvect.z = Qc_temp->GetElement(2,0);

        Qvect = Vsub (Qvect, deltaC_dtdt.pos); // cut away effect of xyz deltas
        mQd = Vdot (vfact, Qvect);
        mQd = mQd - (1/Vlenght(relM.pos))*Vdot(relM_dt.pos, relM_dt.pos);
        double sq_q = Vdot(relM_dt.pos, relM.pos); sq_q = sq_q*sq_q;
        mQd = mQd + (1/Vlenght(relM.pos))*Vdot(relM.pos, relM.pos)*sq_q;
        mQd = mQd + deltaC_dtdt.pos.x;

        Qc->SetElement(index,0, mQd);

        C->SetElement(index,0,      (Vlenght(relM.pos)-deltaC.pos.x));
        C_dt->SetElement(index,0,   ((Vdot(relM_dt.pos,   Vnorm(relM.pos)))-deltaC_dt.pos.x));
        C_dtdt->SetElement(index,0, ((Vdot(relM_dtdt.pos, Vnorm(relM.pos)))-deltaC_dtdt.pos.x));

        Ct->SetElement(index,0,
                      Vdot(vfact,Ct_temp.pos));

        index ++;
    }
}



/////////   5-   UPDATE FORCES
/////////

void ChLinkLock::UpdateForces(double mytime)
{
    // Inherit force computation:
    // also base class can add its own forces.
    ChLinkMasked::UpdateForces(mytime);

    // now add:

    // ========== the link-limits "cushion forces"

    ChVector<> m_force = VNULL;
    ChVector<> m_torque = VNULL;

    if (limit_X->Get_active())
    {
        m_force.x= limit_X->GetForce(relM.pos.x, relM_dt.pos.x);
    }
    if (limit_Y->Get_active())
    {
        m_force.y= limit_Y->GetForce(relM.pos.y, relM_dt.pos.y);
    }
    if (limit_Z->Get_active())
    {
        m_force.z= limit_Z->GetForce(relM.pos.z, relM_dt.pos.z);
    }

    if (limit_D->Get_active())
    {
        m_force = Vadd(m_force,
                       Vmul(Vnorm(relM.pos), limit_D->GetForce(dist, dist_dt)));
    }

    if (limit_Rx->Get_active())
    {
        m_torque.x= limit_Rx->GetForce(relRotaxis.x, relWvel.x);
    }
    if (limit_Ry->Get_active())
    {
        m_torque.y= limit_Ry->GetForce(relRotaxis.y, relWvel.y);
    }
    if (limit_Rz->Get_active())
    {
        m_torque.z= limit_Rz->GetForce(relRotaxis.z, relWvel.z);
    }
    if (limit_Rp->Get_active())
    {
        ChVector<> torq_axis;
        ChVector<> arm_xaxis;
        ChVector<> projected_arm;
        double zenithspeed;
        double zenith;
        double polar;

        arm_xaxis = VaxisXfromQuat (relM.rot);  // the X axis of the marker1, respect to m2.
        zenith = VangleYZplaneNorm (&arm_xaxis);// the angle of m1 Xaxis about normal to YZ plane
        polar  = VangleRX (&arm_xaxis);         // the polar angle of m1 Xaxis spinning about m2 Xaxis

        projected_arm.x = 0; projected_arm.y= arm_xaxis.y; projected_arm.z= arm_xaxis.z;
        ChVector<> vx = VECT_X;
        torq_axis = Vcross (vx, projected_arm);
        torq_axis = Vnorm (torq_axis);      // the axis of torque, laying on YZ plane.

        zenithspeed = Vdot (torq_axis, relWvel);    // the speed of zenith rotation toward cone.

        m_torque= Vadd (m_torque,
            Vmul (torq_axis, limit_Rp->GetPolarForce(zenith, zenithspeed, polar)) );
    }

    C_force = Vadd(C_force, m_force);       // +++
    C_torque = Vadd(C_torque, m_torque);    // +++


    // ========== other forces??
}






//
// Reimplement parent 'lcp stuff' because 'upper/lower limits' may 
// add constraints
//


void ChLinkLock::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	// parent 
	ChLinkMasked::InjectConstraints(mdescriptor);

	if (limit_X) if (limit_X->Get_active())
	{
		limit_X->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_X->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_X->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_X->constr_lower);
		if (limit_X->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_X->constr_upper);
	}
	if (limit_Y) if (limit_Y->Get_active())
	{
		limit_Y->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Y->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_Y->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_Y->constr_lower);
		if (limit_Y->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_Y->constr_upper);
	}
	if (limit_Z) if (limit_Z->Get_active())
	{
		limit_Z->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Z->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_Z->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_Z->constr_lower);
		if (limit_Z->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_Z->constr_upper);
	}
	if (limit_Rx) if (limit_Rx->Get_active())
	{
		limit_Rx->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Rx->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_Rx->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_Rx->constr_lower);
		if (limit_Rx->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_Rx->constr_upper);
	}
	if (limit_Ry) if (limit_Ry->Get_active())
	{
		limit_Ry->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Ry->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_Ry->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_Ry->constr_lower);
		if (limit_Ry->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_Ry->constr_upper);
	}
	if (limit_Rz) if (limit_Rz->Get_active())
	{
		limit_Rz->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Rz->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		if (limit_Rz->constr_lower.IsActive())
			mdescriptor.InsertConstraint(&limit_Rz->constr_lower);
		if (limit_Rz->constr_upper.IsActive())
			mdescriptor.InsertConstraint(&limit_Rz->constr_upper);
	}
}

void ChLinkLock::ConstraintsBiReset()
{
	// parent 
	ChLinkMasked::ConstraintsBiReset();

	if (limit_X) if (limit_X->Get_active())
	{
		limit_X->constr_lower.Set_b_i(0.);
		limit_X->constr_upper.Set_b_i(0.);
	}
	if (limit_Y) if (limit_Y->Get_active())
	{
		limit_Y->constr_lower.Set_b_i(0.);
		limit_Y->constr_upper.Set_b_i(0.);
	}
	if (limit_Z) if (limit_Z->Get_active())
	{
		limit_Z->constr_lower.Set_b_i(0.);
		limit_Z->constr_upper.Set_b_i(0.);
	}
	if (limit_Rx) if (limit_Rx->Get_active())
	{
		limit_Rx->constr_lower.Set_b_i(0.);
		limit_Rx->constr_upper.Set_b_i(0.);
	}
	if (limit_Ry) if (limit_Ry->Get_active())
	{
		limit_Ry->constr_lower.Set_b_i(0.);
		limit_Ry->constr_upper.Set_b_i(0.);
	}
	if (limit_Rz) if (limit_Rz->Get_active())
	{
		limit_Rz->constr_lower.Set_b_i(0.);
		limit_Rz->constr_upper.Set_b_i(0.);
	}
}

void ChLinkLock::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	// parent
	ChLinkMasked::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

	if (limit_X) if (limit_X->Get_active())
	{
		if (limit_X->constr_lower.IsActive())
			if (!do_clamp)
				limit_X->constr_lower.Set_b_i(limit_X->constr_lower.Get_b_i()+factor*(-limit_X->Get_min()+relM.pos.x));
			else
				limit_X->constr_lower.Set_b_i(limit_X->constr_lower.Get_b_i()+ChMax (factor*(-limit_X->Get_min()+relM.pos.x), -recovery_clamp));
		if (limit_X->constr_upper.IsActive())
			if (!do_clamp)
				limit_X->constr_upper.Set_b_i(limit_X->constr_upper.Get_b_i()+factor*( limit_X->Get_max()-relM.pos.x));
			else
				limit_X->constr_upper.Set_b_i(limit_X->constr_upper.Get_b_i()+ChMax (factor*( limit_X->Get_max()-relM.pos.x), -recovery_clamp));
	}
	if (limit_Y) if (limit_Y->Get_active())
	{
		if (limit_Y->constr_lower.IsActive())
			if (!do_clamp)
				limit_Y->constr_lower.Set_b_i(limit_Y->constr_lower.Get_b_i()+factor*(-limit_Y->Get_min()+relM.pos.y));
			else
				limit_Y->constr_lower.Set_b_i(limit_Y->constr_lower.Get_b_i()+ChMax (factor*(-limit_Y->Get_min()+relM.pos.y), -recovery_clamp));
		if (limit_Y->constr_upper.IsActive())
			if (!do_clamp)
				limit_Y->constr_upper.Set_b_i(limit_Y->constr_upper.Get_b_i()+factor*( limit_Y->Get_max()-relM.pos.y));
			else
				limit_Y->constr_upper.Set_b_i(limit_Y->constr_upper.Get_b_i()+ChMax (factor*( limit_Y->Get_max()-relM.pos.y), -recovery_clamp));
	}
	if (limit_Z) if (limit_Z->Get_active())
	{
		if (limit_Z->constr_lower.IsActive())
			if (!do_clamp)
				limit_Z->constr_lower.Set_b_i(limit_Z->constr_lower.Get_b_i()+factor*(-limit_Z->Get_min()+relM.pos.z));
			else
				limit_Z->constr_lower.Set_b_i(limit_Z->constr_lower.Get_b_i()+ChMax (factor*(-limit_Z->Get_min()+relM.pos.z), -recovery_clamp));
		if (limit_Z->constr_upper.IsActive())
			if (!do_clamp)
				limit_Z->constr_upper.Set_b_i(limit_Z->constr_upper.Get_b_i()+factor*( limit_Z->Get_max()-relM.pos.z));
		else
				limit_Z->constr_upper.Set_b_i(limit_Z->constr_upper.Get_b_i()+ChMax (factor*( limit_Z->Get_max()-relM.pos.z), -recovery_clamp));
	}
	if (limit_Rx) if (limit_Rx->Get_active())
	{
		if (limit_Rx->constr_lower.IsActive())
			if (!do_clamp)
				limit_Rx->constr_lower.Set_b_i(limit_Rx->constr_lower.Get_b_i()+factor*( - sin(0.5*limit_Rx->Get_min()) +relM.rot.e1));
			else
				limit_Rx->constr_lower.Set_b_i(limit_Rx->constr_lower.Get_b_i()+ChMax (factor*(- sin(0.5*limit_Rx->Get_min()) +relM.rot.e1), -recovery_clamp));
		if (limit_Rx->constr_upper.IsActive())
			if (!do_clamp)
				limit_Rx->constr_upper.Set_b_i(limit_Rx->constr_upper.Get_b_i()+factor*(   sin(0.5*limit_Rx->Get_max()) -relM.rot.e1));
		else
				limit_Rx->constr_upper.Set_b_i(limit_Rx->constr_upper.Get_b_i()+ChMax (factor*(   sin(0.5*limit_Rx->Get_max()) -relM.rot.e1), -recovery_clamp));
	}
	if (limit_Ry) if (limit_Ry->Get_active())
	{
		if (limit_Ry->constr_lower.IsActive())
			if (!do_clamp)
				limit_Ry->constr_lower.Set_b_i(limit_Ry->constr_lower.Get_b_i()+factor*( - sin(0.5*limit_Ry->Get_min()) +relM.rot.e2));
			else
				limit_Ry->constr_lower.Set_b_i(limit_Ry->constr_lower.Get_b_i()+ChMax (factor*(- sin(0.5*limit_Ry->Get_min()) +relM.rot.e2), -recovery_clamp));
		if (limit_Ry->constr_upper.IsActive())
			if (!do_clamp)
				limit_Ry->constr_upper.Set_b_i(limit_Ry->constr_upper.Get_b_i()+factor*(   sin(0.5*limit_Ry->Get_max()) -relM.rot.e2));
		else
				limit_Ry->constr_upper.Set_b_i(limit_Ry->constr_upper.Get_b_i()+ChMax (factor*(  sin(0.5*limit_Ry->Get_max()) -relM.rot.e2), -recovery_clamp));
	}
	if (limit_Rz) if (limit_Rz->Get_active())
	{
		if (limit_Rz->constr_lower.IsActive())
			if (!do_clamp)
				limit_Rz->constr_lower.Set_b_i(limit_Rz->constr_lower.Get_b_i()+factor*( - sin(0.5*limit_Rz->Get_min()) +relM.rot.e3));
			else
				limit_Rz->constr_lower.Set_b_i(limit_Rz->constr_lower.Get_b_i()+ChMax (factor*(- sin(0.5*limit_Rz->Get_min()) +relM.rot.e3), -recovery_clamp));
		if (limit_Rz->constr_upper.IsActive())
			if (!do_clamp)
				limit_Rz->constr_upper.Set_b_i(limit_Rz->constr_upper.Get_b_i()+factor*(  sin(0.5*limit_Rz->Get_max()) -relM.rot.e3));
		else
				limit_Rz->constr_upper.Set_b_i(limit_Rz->constr_upper.Get_b_i()+ChMax (factor*(  sin(0.5*limit_Rz->Get_max()) -relM.rot.e3), -recovery_clamp));
	}
}

void ChLinkLock::ConstraintsBiLoad_Ct(double factor)
{
	// parent
	ChLinkMasked::ConstraintsBiLoad_Ct(factor);
}

void ChLinkLock::ConstraintsBiLoad_Qc(double factor)
{
	// parent 
	ChLinkMasked::ConstraintsBiLoad_Qc(factor);
}


template <class Real>
void Transform_Cq_to_Cqw_row(ChMatrix<>* mCq, int qrow, ChMatrix<Real>* mCqw, int qwrow, ChBodyFrame* mbody)
{
    // traslational part - not changed
    mCqw->PasteClippedMatrix(mCq, qrow,0, 1,3, qwrow,0);

    // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
    register int col, colres;
    double sum;
	ChMatrixNM<double,3,4> mGl;
	ChFrame<>::SetMatrix_Gl(mGl, mbody->GetCoord().rot);
    for (colres=0; colres < 3; colres++)    {
            sum = 0;
            for (col=0; col < 4; col++)         {
                sum+= ((mCq->GetElement (qrow,col+3))*(mGl.GetElement (colres,col)));
            }
            mCqw->SetElement (qwrow, colres+3, (float)(sum * 0.25));
    }
}


void ChLinkLock::ConstraintsLoadJacobians()
{
	// parent
	ChLinkMasked::ConstraintsLoadJacobians();

	if (limit_X) if (limit_X->Get_active())
	{
		limit_X->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_X->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,0, limit_X->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,0, limit_X->constr_lower.Get_Cq_b() ,0, Body2);
		limit_X->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_X->constr_lower.Get_Cq_a());
		limit_X->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_X->constr_lower.Get_Cq_b());
		limit_X->constr_upper.Get_Cq_a()->MatrNeg();
		limit_X->constr_upper.Get_Cq_b()->MatrNeg();
	}
	if (limit_Y) if (limit_Y->Get_active())
	{
		limit_Y->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Y->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,1, limit_Y->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,1, limit_Y->constr_lower.Get_Cq_b() ,0, Body2);
		limit_Y->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_Y->constr_lower.Get_Cq_a());
		limit_Y->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_Y->constr_lower.Get_Cq_b());
		limit_Y->constr_upper.Get_Cq_a()->MatrNeg();
		limit_Y->constr_upper.Get_Cq_b()->MatrNeg();
	}
	if (limit_Z) if (limit_Z->Get_active())
	{
		limit_Z->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Z->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,2, limit_Z->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,2, limit_Z->constr_lower.Get_Cq_b() ,0, Body2);
		limit_Z->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_Z->constr_lower.Get_Cq_a());
		limit_Z->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_Z->constr_lower.Get_Cq_b());
		limit_Z->constr_upper.Get_Cq_a()->MatrNeg();
		limit_Z->constr_upper.Get_Cq_b()->MatrNeg();
	}
	if (limit_Rx) if (limit_Rx->Get_active())
	{
		limit_Rx->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Rx->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,4, limit_Rx->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,4, limit_Rx->constr_lower.Get_Cq_b() ,0, Body2);
		limit_Rx->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_Rx->constr_lower.Get_Cq_a());
		limit_Rx->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_Rx->constr_lower.Get_Cq_b());
		limit_Rx->constr_upper.Get_Cq_a()->MatrNeg();
		limit_Rx->constr_upper.Get_Cq_b()->MatrNeg();
	}
	if (limit_Ry) if (limit_Ry->Get_active())
	{
		limit_Ry->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Ry->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,5, limit_Ry->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,5, limit_Ry->constr_lower.Get_Cq_b() ,0, Body2);
		limit_Ry->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_Ry->constr_lower.Get_Cq_a());
		limit_Ry->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_Ry->constr_lower.Get_Cq_b());
		limit_Ry->constr_upper.Get_Cq_a()->MatrNeg();
		limit_Ry->constr_upper.Get_Cq_b()->MatrNeg();
	}
	if (limit_Rz) if (limit_Rz->Get_active())
	{
		limit_Rz->constr_lower.SetVariables(&Body1->Variables(),&Body2->Variables());
		limit_Rz->constr_upper.SetVariables(&Body1->Variables(),&Body2->Variables());
		Transform_Cq_to_Cqw_row(Cq1_temp,6, limit_Rz->constr_lower.Get_Cq_a() ,0, Body1);
		Transform_Cq_to_Cqw_row(Cq2_temp,6, limit_Rz->constr_lower.Get_Cq_b() ,0, Body2);
		limit_Rz->constr_upper.Get_Cq_a()->CopyFromMatrix(*limit_Rz->constr_lower.Get_Cq_a());
		limit_Rz->constr_upper.Get_Cq_b()->CopyFromMatrix(*limit_Rz->constr_lower.Get_Cq_b());
		limit_Rz->constr_upper.Get_Cq_a()->MatrNeg();
		limit_Rz->constr_upper.Get_Cq_b()->MatrNeg();
	}
}


void ChLinkLock::ConstraintsFetch_react(double factor)
{
	// parent (from ChConstraint objects to react vector)
	ChLinkMasked::ConstraintsFetch_react(factor);

	// from react vector to the 'intuitive' react_force and react_torque
	ChLinkMaskLF* mmask = (ChLinkMaskLF*) this->mask;
	int n_costraint = 0;

	if (mmask->Constr_X().IsActive()) {
        react_force.x = - react->GetElement(n_costraint, 0);
        n_costraint++ ; }
    if (mmask->Constr_Y().IsActive()) {
        react_force.y = - react->GetElement(n_costraint, 0);
        n_costraint++ ; }
    if (mmask->Constr_Z().IsActive()) {
        react_force.z = - react->GetElement(n_costraint, 0);
        n_costraint++ ; }
    if (mmask->Constr_E1().IsActive()) {
        react_torque.x = - 0.5* (react->GetElement(n_costraint, 0));
        n_costraint++ ; }
    if (mmask->Constr_E2().IsActive()) {
        react_torque.y = - 0.5* (react->GetElement(n_costraint, 0));
        n_costraint++ ; }
    if (mmask->Constr_E3().IsActive()) {
        react_torque.z = - 0.5* (react->GetElement(n_costraint, 0));
        n_costraint++ ; }
    if (mmask->Constr_P().IsActive()) {
        react_torque.z = - 0.5* (react->GetElement(n_costraint, 0));
        n_costraint++ ; }
    if (mmask->Constr_D().IsActive()) {
        react_force  = Vadd (react_force,
            Vmul (Vnorm(relM.pos),- (react->GetElement(n_costraint, 0))));
        n_costraint++ ; }

    // ***TO DO***?: TRASFORMATION FROM delta COORDS TO LINK COORDS, if non-default delta
    // if delta rotation?

	// add also the contribute from link limits to the 'intuitive' react_force and 'react_torque'.
	if (limit_X) if (limit_X->Get_active()) {
		react_force.x -= factor*limit_X->constr_lower.Get_l_i();
		react_force.x += factor*limit_X->constr_upper.Get_l_i();
		}
	if (limit_Y) if (limit_Y->Get_active()) {
		react_force.y -= factor*limit_Y->constr_lower.Get_l_i();
		react_force.y += factor*limit_Y->constr_upper.Get_l_i();
		}
	if (limit_Z) if (limit_Z->Get_active()) {
		react_force.z -= factor*limit_Z->constr_lower.Get_l_i();
		react_force.z += factor*limit_Z->constr_upper.Get_l_i();
		}
	if (limit_Rx) if (limit_Rx->Get_active()) {
		react_torque.x -=  0.5*factor*limit_Rx->constr_lower.Get_l_i();
		react_torque.x +=  0.5*factor*limit_Rx->constr_upper.Get_l_i();
		}
	if (limit_Ry) if (limit_Ry->Get_active()) {
		react_torque.y -=  0.5*factor*limit_Ry->constr_lower.Get_l_i();
		react_torque.y +=  0.5*factor*limit_Ry->constr_upper.Get_l_i();
		}
	if (limit_Rz) if (limit_Rz->Get_active()) {
		react_torque.z -=  0.5*factor*limit_Rz->constr_lower.Get_l_i();
		react_torque.z +=  0.5*factor*limit_Rz->constr_upper.Get_l_i();
		}
    // the internal forces add their contribute to the reactions
    // NOT NEEDED?, since C_force and react_force must stay separated???
    //react_force  = Vadd(react_force, C_force);
    //react_torque = Vadd(react_torque, C_torque);
}




/////////
///////// FILE I/O
/////////

void ChLinkLock::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(10);
		// serialize parent class too
	ChLinkMasked::StreamOUT(mstream);

		// stream out all member data
	mstream << type;
	mstream.AbstractWrite(GetMotion_X());
    mstream.AbstractWrite(GetMotion_Y());
    mstream.AbstractWrite(GetMotion_Z());
    mstream.AbstractWrite(GetMotion_ang());
    mstream.AbstractWrite(GetMotion_ang2());
    mstream.AbstractWrite(GetMotion_ang3());
    mstream << motion_axis;
    mstream << angleset;

	mstream << *limit_X;
    mstream << *limit_Y;
    mstream << *limit_Z;
    mstream << *limit_Rx;
    mstream << *limit_Ry;
    mstream << *limit_Rz;
    mstream << *limit_Rp;
    mstream << *limit_D;
}

void ChLinkLock::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkMasked::StreamIN(mstream);

		// stream in all member data
	// To restore mask array when loading finishes, (maybe following data loading changes mask..)
    ChLinkMask* moriginalmask = this->mask->NewDuplicate();
	int ifoo;
	Vector vfoo;
	ChFunction* ffoo;
	mstream >> ifoo;					ChangeLinkType(ifoo);
	mstream.AbstractReadCreate(&ffoo);	SetMotion_X(ffoo);
    mstream.AbstractReadCreate(&ffoo);	SetMotion_Y(ffoo);
    mstream.AbstractReadCreate(&ffoo);	SetMotion_Z(ffoo);
    mstream.AbstractReadCreate(&ffoo);	SetMotion_ang(ffoo);
    mstream.AbstractReadCreate(&ffoo);	SetMotion_ang2(ffoo);
    mstream.AbstractReadCreate(&ffoo);	SetMotion_ang3(ffoo);
    mstream >> vfoo;					SetMotion_axis(vfoo);
    mstream >> ifoo;					Set_angleset(ifoo);
	mstream >> *limit_X;
    mstream >> *limit_Y;
    mstream >> *limit_Z;
    mstream >> *limit_Rx;
    mstream >> *limit_Ry;
    mstream >> *limit_Rz;
    mstream >> *limit_Rp;
    mstream >> *limit_D;

	this->mask->Copy(moriginalmask);  // restore mask array, if ChangeLinkType() modified it..
	this->ChangedLinkMask();
}






///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


