// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMate)

void ChLinkMate::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMate>();

    // serialize parent class
    ChLink::ArchiveOUT(marchive);

    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMate::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMate>();

    // deserialize parent class
    ChLink::ArchiveIN(marchive);

    // deserialize all member data:
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateGeneric)

ChLinkMateGeneric::ChLinkMateGeneric(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz) {
    c_x = mc_x;
    c_y = mc_y;
    c_z = mc_z;
    c_rx = mc_rx;
    c_ry = mc_ry;
    c_rz = mc_rz;

    C = 0;

    mask = new ChLinkMask();

    SetupLinkMask();
}

ChLinkMateGeneric::ChLinkMateGeneric(const ChLinkMateGeneric& other) : ChLinkMate(other) {
    c_x = other.c_x;
    c_y = other.c_y;
    c_z = other.c_z;
    c_rx = other.c_rx;
    c_ry = other.c_ry;
    c_rz = other.c_rz;

    SetupLinkMask();
}

ChLinkMateGeneric::~ChLinkMateGeneric() {
    if (C)
        delete C;
    C = 0;

    if (mask)
        delete mask;
    mask = 0;
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
        mtempM(0, 0) = aframe.GetRot().e0();
        mtempM(1, 1) = aframe.GetRot().e0();
        mtempM(2, 2) = aframe.GetRot().e0();
        mtempQ.MatrTMultiply(mtempM, Jw1);
        Jw1 = mtempQ;
        mtempQ.MatrTMultiply(mtempM, Jw2);
        Jw2 = mtempQ;

        int nc = 0;

        if (c_x) {
            this->C->ElementN(nc) = aframe.GetPos().x();
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jx1, 0, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jr1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jx2, 0, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jr2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_y) {
            this->C->ElementN(nc) = aframe.GetPos().y();
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jx1, 1, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jr1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jx2, 1, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jr2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_z) {
            this->C->ElementN(nc) = aframe.GetPos().z();
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jx1, 2, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jr1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jx2, 2, 0, 1, 3, 0, 0);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jr2, 2, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rx) {
            this->C->ElementN(nc) = aframe.GetRot().e1();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 0, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 0, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_ry) {
            this->C->ElementN(nc) = aframe.GetRot().e2();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 1, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 1, 0, 1, 3, 0, 3);
            nc++;
        }
        if (c_rz) {
            this->C->ElementN(nc) = aframe.GetRot().e3();
            this->mask->Constr_N(nc).Get_Cq_a()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_b()->FillElem(0);
            this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(Jw1, 2, 0, 1, 3, 0, 3);
            this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(Jw2, 2, 0, 1, 3, 0, 3);
            nc++;
        }
        /*
                if (this->c_x)
                    GetLog()<< "err.x() ="<< aframe.GetPos().x() << "\n";
                if (this->c_y)
                    GetLog()<< "err.y() ="<< aframe.GetPos().y() << "\n";
                if (this->c_z)
                    GetLog()<< "err.z() ="<< aframe.GetPos().z() << "\n";
                if (this->c_x || this->c_y || this->c_z)
                        GetLog()<< *this->C << "\n";
        */
    }
}

void ChLinkMateGeneric::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                   std::shared_ptr<ChBodyFrame> mbody2,
                                   bool pos_are_relative,
                                   ChFrame<> mpos1,
                                   ChFrame<> mpos2) {
    assert(mbody1.get() != mbody2.get());

    this->Body1 = mbody1.get();
    this->Body2 = mbody2.get();
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

void ChLinkMateGeneric::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                   std::shared_ptr<ChBodyFrame> mbody2,
                                   bool pos_are_relative,
                                   ChVector<> mpt1,
                                   ChVector<> mpt2,
                                   ChVector<> mnorm1,
                                   ChVector<> mnorm2) {
    assert(mbody1.get() != mbody2.get());

    this->Body1 = mbody1.get();
    this->Body2 = mbody2.get();
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

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMateGeneric::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (!this->IsActive())
        return;

    int nc = 0;
    if (c_x) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.x();
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.y();
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_force.z();
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_torque.x();
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_torque.y();
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            L(off_L + nc) = -react_torque.z();
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
            react_force.x() = -L(off_L + nc);
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            react_force.y() = -L(off_L + nc);
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            react_force.z() = -L(off_L + nc);
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.x() = -L(off_L + nc);
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.y() = -L(off_L + nc);
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.z() = -L(off_L + nc);
        nc++;
    }
}

void ChLinkMateGeneric::IntLoadResidual_CqL(const unsigned int off_L,
                                            ChVectorDynamic<>& R,
                                            const ChVectorDynamic<>& L,
                                            const double c) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).MultiplyTandAdd(R, L(off_L + cnt) * c);
            cnt++;
        }
    }
}

void ChLinkMateGeneric::IntLoadConstraint_C(const unsigned int off_L,
                                            ChVectorDynamic<>& Qc,
                                            const double c,
                                            bool do_clamp,
                                            double recovery_clamp) {
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

void ChLinkMateGeneric::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    // NOT NEEDED BECAUSE NO RHEONOMIC TERM
}

void ChLinkMateGeneric::IntToDescriptor(const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const ChVectorDynamic<>& R,
                                        const unsigned int off_L,
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

void ChLinkMateGeneric::IntFromDescriptor(const unsigned int off_v,
                                          ChStateDelta& v,
                                          const unsigned int off_L,
                                          ChVectorDynamic<>& L) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            L(off_L + cnt) = mask->Constr_N(i).Get_l_i();
            cnt++;
        }
    }
}

// SOLVER INTERFACES

void ChLinkMateGeneric::InjectConstraints(ChSystemDescriptor& mdescriptor) {
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
                    mask->Constr_N(i).Set_b_i(mask->Constr_N(i).Get_b_i() +
                                              ChMax(factor * C->ElementN(cnt), -recovery_clamp));
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
            react_force.x() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_y) {
        if (mask->Constr_N(nc).IsActive())
            react_force.y() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_z) {
        if (mask->Constr_N(nc).IsActive())
            react_force.z() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_rx) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.x() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_ry) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.y() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
    if (c_rz) {
        if (mask->Constr_N(nc).IsActive())
            react_torque.z() = -mask->Constr_N(nc).Get_l_i() * factor;
        nc++;
    }
}

void ChLinkMateGeneric::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMateGeneric>();

    // serialize parent class
    ChLinkMate::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(frame1);
    marchive << CHNVP(frame2);
    marchive << CHNVP(c_x);
    marchive << CHNVP(c_y);
    marchive << CHNVP(c_z);
    marchive << CHNVP(c_rx);
    marchive << CHNVP(c_ry);
    marchive << CHNVP(c_rz);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateGeneric::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMateGeneric>();

    // deserialize parent class
    ChLinkMate::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(frame1);
    marchive >> CHNVP(frame2);
    marchive >> CHNVP(c_x);
    marchive >> CHNVP(c_y);
    marchive >> CHNVP(c_z);
    marchive >> CHNVP(c_rx);
    marchive >> CHNVP(c_ry);
    marchive >> CHNVP(c_rz);
    this->SetConstrainedCoords(c_x, c_y, c_z, c_rx, c_ry, c_rz);  // takes care of mask
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMatePlane)

ChLinkMatePlane::ChLinkMatePlane(const ChLinkMatePlane& other) : ChLinkMateGeneric(other) {
    flipped = other.flipped;
    separation = other.separation;
}

void ChLinkMatePlane::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180° the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMatePlane::Initialize(std::shared_ptr<ChBodyFrame> mbody1, 
                                 std::shared_ptr<ChBodyFrame> mbody2,
                                 bool pos_are_relative, 
                                 ChVector<> mpt1,
                                 ChVector<> mpt2,
                                 ChVector<> mnorm1,
                                 ChVector<> mnorm2) {
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

void ChLinkMatePlane::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMatePlane>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(flipped);
    marchive << CHNVP(separation);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMatePlane::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMatePlane>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(flipped);
    marchive >> CHNVP(separation);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateCoaxial)

ChLinkMateCoaxial::ChLinkMateCoaxial(const ChLinkMateCoaxial& other) : ChLinkMateGeneric(other) {
    flipped = other.flipped;
}

void ChLinkMateCoaxial::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180° the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMateCoaxial::Initialize(std::shared_ptr<ChBodyFrame> mbody1, 
                                   std::shared_ptr<ChBodyFrame> mbody2,
                                   bool pos_are_relative,
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

void ChLinkMateCoaxial::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMateCoaxial>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateCoaxial::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMateCoaxial>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateSpherical)

ChLinkMateSpherical::ChLinkMateSpherical(const ChLinkMateSpherical& other) : ChLinkMateGeneric(other) {}

void ChLinkMateSpherical::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                     std::shared_ptr<ChBodyFrame> mbody2,
                                     bool pos_are_relative,
                                     ChVector<> mpt1,
                                     ChVector<> mpt2) {
    ChLinkMateGeneric::Initialize(mbody1, mbody2, pos_are_relative, mpt1, mpt2, VECT_X, VECT_X);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateXdistance)

ChLinkMateXdistance::ChLinkMateXdistance(const ChLinkMateXdistance& other) : ChLinkMateGeneric(other) {
    distance = other.distance;
}

void ChLinkMateXdistance::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                     std::shared_ptr<ChBodyFrame> mbody2,
                                     bool pos_are_relative,
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

void ChLinkMateXdistance::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMateXdistance>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(distance);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateXdistance::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMateXdistance>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(distance);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateParallel)

ChLinkMateParallel::ChLinkMateParallel(const ChLinkMateParallel& other) : ChLinkMateGeneric(other) {
    flipped = other.flipped;
}

void ChLinkMateParallel::SetFlipped(bool doflip) {
    if (doflip != this->flipped) {
        // swaps direction of X axis by flippping 180° the frame A (slave)

        ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
        this->frame1.ConcatenatePostTransformation(frameRotator);

        this->flipped = doflip;
    }
}

void ChLinkMateParallel::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                    std::shared_ptr<ChBodyFrame> mbody2,
                                    bool pos_are_relative,
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

void ChLinkMateParallel::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMateParallel>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(flipped);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateParallel::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMateParallel>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(flipped);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateOrthogonal)

ChLinkMateOrthogonal::ChLinkMateOrthogonal(const ChLinkMateOrthogonal& other) : ChLinkMateGeneric(other) {
    reldir1 = other.reldir1;
    reldir2 = other.reldir2;
}

void ChLinkMateOrthogonal::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                      std::shared_ptr<ChBodyFrame> mbody2,
                                      bool pos_are_relative,
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
    this->Body1 = mbody1.get();
    this->Body2 = mbody2.get();

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
            if (fabs(mabsD1.z()) < 0.9)
                ortho_gen = VECT_Z;
            if (fabs(mabsD1.y()) < 0.9)
                ortho_gen = VECT_Y;
            if (fabs(mabsD1.x()) < 0.9)
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

void ChLinkMateOrthogonal::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMateOrthogonal>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(reldir1);
    marchive << CHNVP(reldir2);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMateOrthogonal::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMateOrthogonal>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(reldir1);
    marchive >> CHNVP(reldir2);
}



// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMateFix)

ChLinkMateFix::ChLinkMateFix(const ChLinkMateFix& other) : ChLinkMateGeneric(other) {}

void ChLinkMateFix::Initialize(std::shared_ptr<ChBodyFrame> mbody1,
                                     std::shared_ptr<ChBodyFrame> mbody2)
{
    ChLinkMateGeneric::Initialize(
        mbody1, mbody2, 
        false,               // constraint reference frame in abs coords
        ChFrame<>(*mbody1),  // defaults to have constraint reference frame as mbody1 coordsystem
        ChFrame<>(*mbody1)   // defaults to have constraint reference frame as mbody1 coordsystem
        ); 
}




}  // end namespace chrono
