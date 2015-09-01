//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   ChLinkMate.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkMate.h"
#include "physics/ChSystem.h"

namespace chrono {

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLinkMate> a_registration_ChLinkMate;

void ChLinkMate::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);
    // serialize parent class too
    ChLink::StreamOUT(mstream);

    // stream out all member data
}

void ChLinkMate::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChLink::StreamIN(mstream);

    // stream in all member data
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateGeneric> a_registration_ChLinkMateGeneric;

ChLinkMateGeneric::ChLinkMateGeneric(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
    c_rx = mc_rx;
    c_ry = mc_ry;
    c_rz = mc_rz;

    C = 0;
    cache_li_pos = 0;
    cache_li_speed = 0;

    mask = new ChLinkMask();

    SetupLinkMask();
}

ChLinkMateGeneric::~ChLinkMateGeneric() {
    if (C)
        delete C;
    C = 0;

    if (cache_li_pos)
        delete cache_li_pos;
    cache_li_pos = 0;

    if (cache_li_speed)
        delete cache_li_speed;
    cache_li_speed = 0;

    if (mask)
        delete mask;
    mask = 0;
}

void ChLinkMateGeneric::Copy(ChLinkMateGeneric* source) {
    // first copy the parent class data...
    //
    ChLinkMate::Copy(source);

    c_x = source->c_x;
    c_y = source->c_y;
    c_z = source->c_z;
    c_rx = source->c_rx;
    c_ry = source->c_ry;
    c_rz = source->c_rz;

    SetupLinkMask();
}

ChLink* ChLinkMateGeneric::new_Duplicate() {
    ChLinkMateGeneric* m_l = new ChLinkMateGeneric;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateGeneric::SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
    c_rx = mc_rx;
    c_ry = mc_ry;
    c_rz = mc_rz;

    SetupLinkMask();
}

void ChLinkMateGeneric::SetupLinkMask() {
    int nc = 0;
    if (c_x)
        nc++;
    if (c_y)
        nc++;
    if (c_z)
        nc++;
    if (c_rx)
        nc++;
    if (c_ry)
        nc++;
    if (c_rz)
        nc++;

    mask->ResetNconstr(nc);

    if (C)
        delete C;
    C = new ChMatrixDynamic<>(nc, 1);

    if (cache_li_pos)
        delete cache_li_pos;
    cache_li_pos = new ChMatrixDynamic<>(nc, 1);

    if (cache_li_speed)
        delete cache_li_speed;
    cache_li_speed = new ChMatrixDynamic<>(nc, 1);

    ChangedLinkMask();
}

void ChLinkMateGeneric::ChangedLinkMask() {
    ndoc = mask->GetMaskDoc();
    ndoc_c = mask->GetMaskDoc_c();
    ndoc_d = mask->GetMaskDoc_d();
}

void ChLinkMateGeneric::SetDisabled(bool mdis) {
    ChLinkMate::SetDisabled(mdis);

    if (mask->SetAllDisabled(mdis) > 0)
        ChangedLinkMask();
}

void ChLinkMateGeneric::SetBroken(bool mbro) {
    ChLinkMate::SetBroken(mbro);

    if (mask->SetAllBroken(mbro) > 0)
        ChangedLinkMask();
}

int ChLinkMateGeneric::RestoreRedundant() {
    int mchanges = mask->RestoreRedundant();
    if (mchanges)
        ChangedLinkMask();
    return mchanges;
}

void ChLinkMateGeneric::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::Update(mytime, update_assets);

    if (this->Body1 && this->Body2) {
        this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

        ChFrame<> aframe = this->frame1 >> (*this->Body1);
        ChVector<> p1_abs = aframe.GetPos();
        ChFrame<> aframe2 = this->frame2 >> (*this->Body2);
        ChVector<> p2_abs = aframe2.GetPos();
        ChFrame<> bframe;
        static_cast<ChFrame<>*>(this->Body2)->TransformParentToLocal(aframe, bframe);
        this->frame2.TransformParentToLocal(bframe, aframe);
        // Now 'aframe' contains the position/rotation of frame 1 respect to frame 2, in frame 2 coords.

        ChMatrix33<> Jx1, Jx2, Jr1, Jr2, Jw1, Jw2;
        ChMatrix33<> mtempM, mtempQ;

        ChMatrix33<> abs_plane;
        abs_plane.MatrMultiply(Body2->GetA(), frame2.GetA());

        Jx1.CopyFromMatrixT(abs_plane);
        Jx2.CopyFromMatrixT(abs_plane);
        Jx2.MatrNeg();

        Jw1.MatrTMultiply(abs_plane, Body1->GetA());
        Jw2.MatrTMultiply(abs_plane, Body2->GetA());

        mtempM.Set_X_matrix(frame1.GetPos());
        Jr1.MatrMultiply(Jw1, mtempM);
        Jr1.MatrNeg();

        mtempM.Set_X_matrix(frame2.GetPos());
        Jr2.MatrMultiply(Jw2, mtempM);

        ChVector<> p2p1_base2 = (Body2->GetA()).MatrT_x_Vect(Vsub(p1_abs, p2_abs));
        mtempM.Set_X_matrix(p2p1_base2);
        mtempQ.MatrTMultiply(frame2.GetA(), mtempM);
        Jr2.MatrInc(mtempQ);

        Jw2.MatrNeg();

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        // For small misalignment this effect is almost insignificant cause [Fp(q_resid)]=[I],
        // but otherwise it is needed (if you want to use the stabilization term - if not, you can live without).
        mtempM.Set_X_matrix((aframe.GetRot().GetVector()) * 0.5);
        mtempM(0, 0) = aframe.GetRot().e0;
        mtempM(1, 1) = aframe.GetRot().e0;
        mtempM(2, 2) = aframe.GetRot().e0;
        mtempQ.MatrTMultiply(mtempM, Jw1);
        Jw1 = mtempQ;
        mtempQ.MatrTMultiply(mtempM, Jw2);
        Jw2 = mtempQ;

        int nc = 0;

        if (c_x) {
            this->C->ElementN(nc) = aframe.GetPos().x;
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 0, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 0, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_y) {
            this->C->ElementN(nc) = aframe.GetPos().y;
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 1, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 1, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_z) {
            this->C->ElementN(nc) = aframe.GetPos().z;
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 2, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 2, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 2, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rx) {
            this->C->ElementN(nc) = aframe.GetRot().e1;
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_ry) {
            this->C->ElementN(nc) = aframe.GetRot().e2;
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rz) {
            this->C->ElementN(nc) = aframe.GetRot().e3;
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 2, 0, 1, 3, 0, 3);
            nc++;
        }
        /*
                if (this->c_x)
                    GetLog()<< "err.x ="<< aframe.GetPos().x << "\n";
                if (this->c_y)
                    GetLog()<< "err.y ="<< aframe.GetPos().y << "\n";
                if (this->c_z)
                    GetLog()<< "err.z ="<< aframe.GetPos().z << "\n";
                if (this->c_x || this->c_y || this->c_z)
                        GetLog()<< *this->C << "\n";
        */
    }
}

void ChLinkMateGeneric::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChFrame<> mpos1,        ///< mate frame (slave), for 1st body (rel. or abs., see flag above)
    ChFrame<> mpos2         ///< mate frame (master), for 2nd body (rel. or abs., see flag above)
    ) {
    assert(mbody1.get_ptr() != mbody2.get_ptr());

    this->Body1 = mbody1.get_ptr();
    this->Body2 = mbody2.get_ptr();
    // this->SetSystem(mbody1->GetSystem());

    this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

    if (pos_are_relative) {
        this->frame1 = mpos1;
        this->frame2 = mpos2;
    } else {
        // from abs to body-rel
        static_cast<ChFrame<>*>(this->Body1)->TransformParentToLocal(mpos1, this->frame1);
        static_cast<ChFrame<>*>(this->Body2)->TransformParentToLocal(mpos2, this->frame2);
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMateGeneric::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.x;
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.y;
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.z;
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -2 * react_torque.x;
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -2 * react_torque.y;
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -2 * react_torque.z;
        nc++;
    }
}

void ChLinkMateGeneric::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force = VNULL;
    react_torque = VNULL;

    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask->Constr_N(nc).IsActive())
            react_force.x = -L(off_L + nc);
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            react_force.y = -L(off_L + nc);
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            react_force.z = -L(off_L + nc);
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.x = -0.5 * L(off_L + nc);
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.y = -0.5 * L(off_L + nc);
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.z = -0.5 * L(off_L + nc);
        nc++;
    }
}

void ChLinkMateGeneric::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
                                            ChVectorDynamic<>& R,        ///< result: the R residual, R += c*Cq'*L
                                            const ChVectorDynamic<>& L,  ///< the L vector
                                            const double c               ///< a scaling factor
                                            ) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).MultiplyTandAdd(R, L(off_L + cnt) * c);
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
                                            ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*C
                                            const double c,            ///< a scaling factor
                                            bool do_clamp,             ///< apply clamping to c*C?
                                            double recovery_clamp      ///< value for min/max clamping of c*C
                                            ) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            if (do_clamp) {
                if (mask->Constr_N(i).IsUnilateral())
                    Qc(off_L + cnt) += ChMax(c * C->ElementN(cnt), -recovery_clamp);
                else
                    Qc(off_L + cnt) += ChMin(ChMax(c * C->ElementN(cnt), -recovery_clamp), recovery_clamp);
            } else
                Qc(off_L + cnt) += c * C->ElementN(cnt);
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                             ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                             const double c             ///< a scaling factor
                                             ) {
    // NOT NEEDED BECAUSE NO RHEONOMIC TERM
}

void ChLinkMateGeneric::IntToLCP(const unsigned int off_v,  ///< offset in v, R
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,  ///< offset in L, Qc
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Set_l_i(L(off_L + cnt));
            mask->Constr_N(i).Set_b_i(Qc(off_L + cnt));
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntFromLCP(const unsigned int off_v,  ///< offset in v
                                   ChStateDelta& v,
                                   const unsigned int off_L,  ///< offset in L
                                   ChVectorDynamic<>& L) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            L(off_L + cnt) = mask->Constr_N(i).Get_l_i();
            cnt++;
        }
    }
}

////////// LCP INTERFACES ////

void ChLinkMateGeneric::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
    if (!this->IsActive())
        return;

    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive())
            mdescriptor.InsertConstraint(&mask->Constr_N(i));
    }
}

void ChLinkMateGeneric::ConstraintsBiReset() {
    if (!this->IsActive())
        return;

    for (int i = 0; i < mask->nconstr; i++) {
        mask->Constr_N(i).Set_b_i(0.);
    }
}

void ChLinkMateGeneric::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    if (!this->IsActive())
        return;

    //***TEST***
    /*
        GetLog()<< "cload: " ;
        if (this->c_x) GetLog()<< " x";
        if (this->c_y) GetLog()<< " y";
        if (this->c_z) GetLog()<< " z";
        if (this->c_rx) GetLog()<< " Rx";
        if (this->c_ry) GetLog()<< " Ry";
        if (this->c_rz) GetLog()<< " Rz";
        GetLog()<< *this->C << "\n";
    */
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            if (do_clamp) {
                if (mask->Constr_N(i).IsUnilateral())
                    mask->Constr_N(i)
                        .Set_b_i(mask->Constr_N(i).Get_b_i() + ChMax(factor * C->ElementN(cnt), -recovery_clamp));
                else
                    mask->Constr_N(i).Set_b_i(mask->Constr_N(i).Get_b_i() +
                                              ChMin(ChMax(factor * C->ElementN(cnt), -recovery_clamp), recovery_clamp));
            } else
                mask->Constr_N(i).Set_b_i(mask->Constr_N(i).Get_b_i() + factor * C->ElementN(cnt));

            cnt++;
        }
    }
}

void ChLinkMateGeneric::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    // NOT NEEDED BECAUSE NO RHEONOMIC TERM
}

void ChLinkMateGeneric::ConstraintsLoadJacobians() {
    // already loaded when doing Update (which used the matrices of the scalar constraint objects)
}

void ChLinkMateGeneric::ConstraintsFetch_react(double factor) {
    react_force = VNULL;
    react_torque = VNULL;

    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask->Constr_N(nc).IsActive())
            react_force.x = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            react_force.y = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            react_force.z = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.x = -0.5 * mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.y = -0.5 * mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.z = -0.5 * mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
}

void ChLinkMateGeneric::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2,
    ChVector<> mnorm1,
    ChVector<> mnorm2) {
    assert(mbody1.get_ptr() != mbody2.get_ptr());

    this->Body1 = mbody1.get_ptr();
    this->Body2 = mbody2.get_ptr();
    // this->SetSystem(mbody1->GetSystem());

    this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());

    ChVector<> mx, my, mz, mN;
    ChMatrix33<> mrot;

    ChFrame<> mfr1;
    ChFrame<> mfr2;

    if (pos_are_relative) {
        mN = mnorm1;
        mN.DirToDxDyDz(mx, my, mz);
        mrot.Set_A_axis(mx, my, mz);
        mfr1.SetRot(mrot);
        mfr1.SetPos(mpt1);

        mN = mnorm2;
        mN.DirToDxDyDz(mx, my, mz);
        mrot.Set_A_axis(mx, my, mz);
        mfr2.SetRot(mrot);
        mfr2.SetPos(mpt2);
    } else {
        ChVector<> temp = VECT_Z;
        // from abs to body-rel
        mN = this->Body1->TransformDirectionParentToLocal(mnorm1);
        mN.DirToDxDyDz(mx, my, mz, temp);
        mrot.Set_A_axis(mx, my, mz);
        mfr1.SetRot(mrot);
        mfr1.SetPos(this->Body1->TransformPointParentToLocal(mpt1));

        mN = this->Body2->TransformDirectionParentToLocal(mnorm2);
        mN.DirToDxDyDz(mx, my, mz, temp);
        mrot.Set_A_axis(mx, my, mz);
        mfr2.SetRot(mrot);
        mfr2.SetPos(this->Body2->TransformPointParentToLocal(mpt2));
    }

    this->frame1 = mfr1;
    this->frame2 = mfr2;
}

//
// Following functions are for exploiting persistence
//

void ChLinkMateGeneric::ConstraintsLiLoadSuggestedSpeedSolution() {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Set_l_i(cache_li_speed->ElementN(cnt));
            cnt++;
        }
    }
}

void ChLinkMateGeneric::ConstraintsLiLoadSuggestedPositionSolution() {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Set_l_i(cache_li_pos->ElementN(cnt));
            cnt++;
        }
    }
}

void ChLinkMateGeneric::ConstraintsLiFetchSuggestedSpeedSolution() {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            cache_li_speed->ElementN(cnt) = mask->Constr_N(i).Get_l_i();
            cnt++;
        }
    }
}

void ChLinkMateGeneric::ConstraintsLiFetchSuggestedPositionSolution() {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            cache_li_pos->ElementN(cnt) = mask->Constr_N(i).Get_l_i();
            cnt++;
        }
    }
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMatePlane> a_registration_ChLinkMatePlane;

void ChLinkMatePlane::Copy(ChLinkMatePlane* source) {
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);

    this->flipped = source->flipped;
    this->separation = source->separation;
}

ChLink* ChLinkMatePlane::new_Duplicate() {
    ChLinkMatePlane* m_l = new ChLinkMatePlane;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMatePlane::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180� the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMatePlane::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,        ///< point on slave plane, for 1st body (rel. or abs., see flag above)
    ChVector<> mpt2,        ///< point on master plane, for 2nd body (rel. or abs., see flag above)
    ChVector<> mnorm1,      ///< normal of slave plane, for 1st body (rel. or abs., see flag above)
    ChVector<> mnorm2       ///< normal of master plane, for 2nd body (rel. or abs., see flag above)
    ) {
    // set the two frames so that they have the X axis aligned when the
    // two normals are opposed (default behavior, otherwise is considered 'flipped')

    ChVector<> mnorm1_reversed;
    if (!this->flipped)
        mnorm1_reversed = -mnorm1;
    else
        mnorm1_reversed = mnorm1;

    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mnorm1_reversed, mnorm2);
}

/// Override _all_ time, jacobian etc. updating.
void ChLinkMatePlane::Update(double mtime, bool update_assets) {
    // Parent class inherit
    ChLinkMateGeneric::Update(mtime, update_assets);

    // .. then add the effect of imposed distance on C residual vector
    this->C->Element(0, 0) -= this->separation;  // for this mate, C = {Cx, Cry, Crz}
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateCoaxial> a_registration_ChLinkMateCoaxial;

void ChLinkMateCoaxial::Copy(ChLinkMateCoaxial* source) {
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);

    this->flipped = source->flipped;
}

ChLink* ChLinkMateCoaxial::new_Duplicate() {
    ChLinkMateCoaxial* m_l = new ChLinkMateCoaxial;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateCoaxial::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180� the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMateCoaxial::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2,
    ChVector<> mnorm1,
    ChVector<> mnorm2) {
    // set the two frames so that they have the X axis aligned when the
    // two normals are opposed (default behavior, otherwise is considered 'flipped')

    ChVector<> mnorm1_reversed;
    if (!this->flipped)
        mnorm1_reversed = mnorm1;
    else
        mnorm1_reversed = -mnorm1;

    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mnorm1_reversed, mnorm2);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateSpherical> a_registration_ChLinkMateSpherical;

void ChLinkMateSpherical::Copy(ChLinkMateSpherical* source) {
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);
}

ChLink* ChLinkMateSpherical::new_Duplicate() {
    ChLinkMateSpherical* m_l =
        new ChLinkMateSpherical;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateSpherical::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2) {
    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, VECT_X, VECT_X);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateXdistance> a_registration_ChLinkMateXdistance;

void ChLinkMateXdistance::Copy(ChLinkMateXdistance* source) {
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);

    // ...then copy class data
    this->distance = source->distance;
}

ChLink* ChLinkMateXdistance::new_Duplicate() {
    ChLinkMateXdistance* m_l =
        new ChLinkMateXdistance;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateXdistance::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2,
    ChVector<> mdir2) {
    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mdir2, mdir2);
}

void ChLinkMateXdistance::Update(double mtime, bool update_assets) {
    // Parent class inherit
    ChLinkMateGeneric::Update(mtime, update_assets);

    // .. then add the effect of imposed distance on C residual vector
    this->C->Element(0, 0) -= this->distance;  // for this mate, C = {Cx}
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateParallel> a_registration_ChLinkMateParallel;

void ChLinkMateParallel::Copy(ChLinkMateParallel* source) {
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);
}

ChLink* ChLinkMateParallel::new_Duplicate() {
    ChLinkMateParallel* m_l =
        new ChLinkMateParallel;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateParallel::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180� the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMateParallel::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2,
    ChVector<> mnorm1,
    ChVector<> mnorm2) {
    // set the two frames so that they have the X axis aligned when the
    // two axes are aligned (default behavior, otherwise is considered 'flipped')

    ChVector<> mnorm1_reversed;
    if (!this->flipped)
        mnorm1_reversed = mnorm1;
    else
        mnorm1_reversed = -mnorm1;

    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, mnorm1_reversed, mnorm2);
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateOrthogonal> a_registration_ChLinkMateOrthogonal;

void ChLinkMateOrthogonal::Copy(ChLinkMateOrthogonal* source) {
    // first copy the parent class data...
    ChLinkMateGeneric::Copy(source);

    // ..then class data
    this->reldir1 = source->reldir1;
    this->reldir2 = source->reldir2;
}

ChLink* ChLinkMateOrthogonal::new_Duplicate() {
    ChLinkMateOrthogonal* m_l =
        new ChLinkMateOrthogonal;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkMateOrthogonal::Initialize(
    ChSharedPtr<ChBodyFrame> mbody1,  ///< first body to link
    ChSharedPtr<ChBodyFrame> mbody2,  ///< second body to link
    bool pos_are_relative,  ///< true: following posit. are considered relative to bodies. false: pos.are absolute
    ChVector<> mpt1,
    ChVector<> mpt2,
    ChVector<> mnorm1,
    ChVector<> mnorm2) {
    // set the two frames so that they have the X axis aligned

    ChVector<> mabsnorm1, mabsnorm2;
    if (pos_are_relative) {
        this->reldir1 = mnorm1;
        this->reldir2 = mnorm2;
    } else {
        this->reldir1 = mbody1->TransformDirectionParentToLocal(mnorm1);
        this->reldir2 = mbody2->TransformDirectionParentToLocal(mnorm2);
    }

    // do this asap otherwise the following Update() won't work..
    this->Body1 = mbody1.get_ptr();
    this->Body2 = mbody2.get_ptr();

    // Force the alignment of frames so that the X axis is cross product of two dirs, etc.
    // by calling the custom update function of ChLinkMateOrthogonal.
    this->Update(this->ChTime);

    // Perform initialization (set pointers to variables, etc.)
    ChLinkMateGeneric::Initialize(mbody1, mbody2,
                                  true,  // recycle already-updated frames
                                  this->frame1, this->frame2);
}

/// Override _all_ time, jacobian etc. updating.
void ChLinkMateOrthogonal::Update(double mtime, bool update_assets) {
    // Prepare the alignment of the two frames so that the X axis is orthogonal
    // to the two directions

    ChVector<> mabsD1, mabsD2;

    if (this->Body1 && this->Body2) {
        mabsD1 = this->Body1->TransformDirectionLocalToParent(this->reldir1);
        mabsD2 = this->Body2->TransformDirectionLocalToParent(this->reldir2);

        ChVector<> mX = Vcross(mabsD2, mabsD1);
        double xlen = mX.Length();

        // Ops.. parallel directions? -> fallback to singularity handling
        if (fabs(xlen) < 1e-20) {
            ChVector<> ortho_gen;
            if (fabs(mabsD1.z) < 0.9)
                ortho_gen = VECT_Z;
            if (fabs(mabsD1.y) < 0.9)
                ortho_gen = VECT_Y;
            if (fabs(mabsD1.x) < 0.9)
                ortho_gen = VECT_X;
            mX = Vcross(mabsD1, ortho_gen);
            xlen = Vlength(mX);
        }
        mX.Scale(1.0 / xlen);

        ChVector<> mY1, mZ1;
        mZ1 = mabsD1;
        mY1 = Vcross(mZ1, mX);

        ChMatrix33<> mA1;
        mA1.Set_A_axis(mX, mY1, mZ1);

        ChVector<> mY2, mZ2;
        mY2 = mabsD2;
        mZ2 = Vcross(mX, mY2);

        ChMatrix33<> mA2;
        mA2.Set_A_axis(mX, mY2, mZ2);

        ChFrame<> absframe1(VNULL, mA1);  // position not needed for orth. constr. computation
        ChFrame<> absframe2(VNULL, mA2);  // position not needed for orth. constr. computation

        // from abs to body-rel
        static_cast<ChFrame<>*>(this->Body1)->TransformParentToLocal(absframe1, this->frame1);
        static_cast<ChFrame<>*>(this->Body2)->TransformParentToLocal(absframe2, this->frame2);
    }

    // Parent class inherit
    ChLinkMateGeneric::Update(mtime, update_assets);
}

}  // END_OF_NAMESPACE____
