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

#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChLinkMasked.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMasked)

ChLinkMasked::ChLinkMasked() {
    force_D = new ChLinkForce;  // defeault no forces in link dof
    force_R = new ChLinkForce;
    force_X = new ChLinkForce;
    force_Y = new ChLinkForce;
    force_Z = new ChLinkForce;
    force_Rx = new ChLinkForce;
    force_Ry = new ChLinkForce;
    force_Rz = new ChLinkForce;

    d_restlength = 0;

    ndoc_d = ndoc_c = ndoc = 0;

    C = C_dt = C_dtdt = NULL;  // initialize matrices.

    react = 0;
    Qc = Ct = 0;
    Cq1 = Cq2 = 0;
    Cqw1 = Cqw2 = 0;

    mask = new ChLinkMask(1);                    // create the mask;
    mask->Constr_N(0).SetMode(CONSTRAINT_FREE);  // default: one constr.eq. but not working

    BuildLink();  // setup all matrices - if any (i.e. none in this base link)-
                  // setting automatically  n. of DOC and DOF,
}

ChLinkMasked::ChLinkMasked(const ChLinkMasked& other) : ChLinkMarkers(other) {
    mask = other.mask->Clone();

    // setup -alloc all needed matrices!!
    ChangedLinkMask();

    force_D = other.force_D->Clone();
    force_R = other.force_R->Clone();
    force_X = other.force_X->Clone();
    force_Y = other.force_Y->Clone();
    force_Z = other.force_Z->Clone();
    force_Rx = other.force_Rx->Clone();
    force_Ry = other.force_Ry->Clone();
    force_Rz = other.force_Rz->Clone();

    d_restlength = other.d_restlength;
}

ChLinkMasked::~ChLinkMasked() {
    DestroyLink();

    if (force_D)
        delete force_D;
    if (force_R)
        delete force_R;
    if (force_X)
        delete force_X;
    if (force_Y)
        delete force_Y;
    if (force_Z)
        delete force_Z;
    if (force_Rx)
        delete force_Rx;
    if (force_Ry)
        delete force_Ry;
    if (force_Rz)
        delete force_Rz;

    delete mask;
    mask = NULL;
}

void ChLinkMasked::BuildLink() {
    // set ndoc by counting non-dofs
    ndoc = mask->GetMaskDoc();
    ndoc_c = mask->GetMaskDoc_c();
    ndoc_d = mask->GetMaskDoc_d();

    // create matrices
    if (ndoc > 0) {
        C = new ChMatrixDynamic<>(ndoc, 1);
        C_dt = new ChMatrixDynamic<>(ndoc, 1);
        C_dtdt = new ChMatrixDynamic<>(ndoc, 1);
        react = new ChMatrixDynamic<>(ndoc, 1);
        Qc = new ChMatrixDynamic<>(ndoc, 1);
        Ct = new ChMatrixDynamic<>(ndoc, 1);
        Cq1 = new ChMatrixDynamic<>(ndoc, BODY_QDOF);
        Cq2 = new ChMatrixDynamic<>(ndoc, BODY_QDOF);
        Cqw1 = new ChMatrixDynamic<>(ndoc, BODY_DOF);
        Cqw2 = new ChMatrixDynamic<>(ndoc, BODY_DOF);
    } else {
        C = 0;
        C_dt = 0;
        C_dtdt = 0;
        react = 0;
        Qc = 0;
        Ct = 0;
        Cq1 = 0;
        Cq2 = 0;
        Cqw1 = 0;
        Cqw2 = 0;
    }
}

void ChLinkMasked::BuildLink(ChLinkMask* new_mask) {
    // set mask
    delete mask;
    mask = new_mask->Clone();

    // setup matrices;
    BuildLink();
}

void ChLinkMasked::DestroyLink() {
    if (ndoc > 0) {
        if (C) {
            delete C;
            C = NULL;
        }
        if (C_dt) {
            delete C_dt;
            C_dt = NULL;
        }
        if (C_dtdt) {
            delete C_dtdt;
            C_dtdt = NULL;
        }
        if (react) {
            delete react;
            react = NULL;
        }
        if (Qc) {
            delete Qc;
            Qc = NULL;
        }
        if (Ct) {
            delete Ct;
            Ct = NULL;
        }
        if (Cq1) {
            delete Cq1;
            Cq1 = NULL;
        }
        if (Cq2) {
            delete Cq2;
            Cq2 = NULL;
        }
        if (Cqw1) {
            delete Cqw1;
            Cqw1 = NULL;
        }
        if (Cqw2) {
            delete Cqw2;
            Cqw2 = NULL;
        }
        if (react) {
            delete react;
            react = NULL;
        }
    }

    ndoc = 0;
}

void ChLinkMasked::ChangeLinkMask(ChLinkMask* new_mask) {
    DestroyLink();
    BuildLink(new_mask);
}

void ChLinkMasked::ChangedLinkMask() {
    DestroyLink();
    BuildLink();
}

void ChLinkMasked::SetDisabled(bool mdis) {
    ChLinkMarkers::SetDisabled(mdis);

    if (mask->SetAllDisabled(mdis) > 0)
        ChangedLinkMask();
}

void ChLinkMasked::SetBroken(bool mbro) {
    ChLinkMarkers::SetBroken(mbro);

    if (mask->SetAllBroken(mbro) > 0)
        ChangedLinkMask();
}

int ChLinkMasked::RestoreRedundant() {
    int mchanges = mask->RestoreRedundant();
    if (mchanges)
        ChangedLinkMask();
    return mchanges;
}

void ChLinkMasked::SetUpMarkers(ChMarker* mark1, ChMarker* mark2) {
    ChLinkMarkers::SetUpMarkers(mark1, mark2);

    // could the line below be:     assert(this->Body1 && this->Body2); ?
    if (this->Body1 && this->Body2) {
        ((ChLinkMaskLF*)this->mask)->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());
        // This is needed because only if all constraints in mask are now active, and C,Ct,etc.
        // matrices must be allocated accordingly, otherwise are null.
        DestroyLink();
        BuildLink();
    }
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkMasked::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    if (react)
        L.PasteMatrix(*react, off_L, 0);
}

void ChLinkMasked::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    react_force = VNULL;   // Do not update 'intuitive' react force and torque here: just set as 0.
    react_torque = VNULL;  // Child classes implementations should compute them.

    if (react)
        react->PasteClippedMatrix(L, off_L, 0, react->GetRows(), 1, 0, 0);
}

void ChLinkMasked::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
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

void ChLinkMasked::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
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

void ChLinkMasked::IntLoadConstraint_Ct(const unsigned int off_L,  ///< offset in Qc residual
                                        ChVectorDynamic<>& Qc,     ///< result: the Qc residual, Qc += c*Ct
                                        const double c             ///< a scaling factor
                                        ) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            Qc(off_L + cnt) += c * Ct->ElementN(cnt);
            cnt++;
        }
    }
}

void ChLinkMasked::IntToDescriptor(const unsigned int off_v,  ///< offset in v, R
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

void ChLinkMasked::IntFromDescriptor(const unsigned int off_v,  ///< offset in v
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

// SOLVER SYSTEM FUNCTIONS

void ChLinkMasked::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    if (!this->IsActive())
        return;

    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive())
            mdescriptor.InsertConstraint(&mask->Constr_N(i));
    }
}

void ChLinkMasked::ConstraintsBiReset() {
    for (int i = 0; i < mask->nconstr; i++) {
        mask->Constr_N(i).Set_b_i(0.);
    }
}

void ChLinkMasked::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
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

void ChLinkMasked::ConstraintsBiLoad_Ct(double factor) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Set_b_i(mask->Constr_N(i).Get_b_i() + factor * Ct->ElementN(cnt));
            cnt++;
        }
    }
}

void ChLinkMasked::ConstraintsBiLoad_Qc(double factor) {
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Set_b_i(mask->Constr_N(i).Get_b_i() + factor * Qc->ElementN(cnt));
            cnt++;
        }
    }
}

void ChLinkMasked::ConstraintsLoadJacobians() {
    if (!this->ndoc)
        return;

    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            mask->Constr_N(i).Get_Cq_a()->PasteClippedMatrix(*Cqw1, cnt, 0, 1, this->Cqw1->GetColumns(), 0, 0);
            mask->Constr_N(i).Get_Cq_b()->PasteClippedMatrix(*Cqw2, cnt, 0, 1, this->Cqw2->GetColumns(), 0, 0);
            cnt++;

            // sets also the CFM term
            // mask->Constr_N(i).Set_cfm_i(this->attractor);
        }
    }
}

void ChLinkMasked::ConstraintsFetch_react(double factor) {
    react_force = VNULL;   // Do not update 'intuitive' react force and torque here: just set as 0.
    react_torque = VNULL;  // Child classes implementations should compute them.

    // From constraints to react vector:
    int cnt = 0;
    for (int i = 0; i < mask->nconstr; i++) {
        if (mask->Constr_N(i).IsActive()) {
            react->ElementN(cnt) = mask->Constr_N(i).Get_l_i() * factor;
            cnt++;
        }
    }
}

////////////////////////////////////
///
///    UPDATING PROCEDURES

/////////   4.5- UPDATE Cqw1 and Cqw2
/////////
void ChLinkMasked::Transform_Cq_to_Cqw(ChMatrix<>* mCq, ChMatrix<>* mCqw, ChBodyFrame* mbody) {
    if (!mCq)
        return;

    // traslational part - not changed
    mCqw->PasteClippedMatrix(*mCq, 0, 0, mCq->GetRows(), 3, 0, 0);

    // rotational part [Cq_w] = [Cq_q]*[Gl]'*1/4
    int col, row, colres;
    double sum;

    ChMatrixNM<double, 3, 4> mGl;
    ChFrame<>::SetMatrix_Gl(mGl, mbody->GetCoord().rot);

    for (colres = 0; colres < 3; colres++) {
        for (row = 0; row < (mCq->GetRows()); row++) {
            sum = 0;
            for (col = 0; col < 4; col++) {
                sum += ((mCq->GetElement(row, col + 3)) * (mGl.GetElement(colres, col)));
            }
            mCqw->SetElement(row, colres + 3, sum * 0.25);
        }
    }
}

void ChLinkMasked::UpdateCqw() {
    if (!Cq1 || !Cq2)
        return;

    ChLinkMasked::Transform_Cq_to_Cqw(Cq1, Cqw1, Body1);
    ChLinkMasked::Transform_Cq_to_Cqw(Cq2, Cqw2, Body2);
}

/////////   5-   UPDATE FORCES
/////////

void ChLinkMasked::UpdateForces(double mytime) {
    ChLinkMarkers::UpdateForces(mytime);

    // ** Child class can inherit this method. The parent implementation must
    //    be called _before_ adding further custom forces.

    Vector m_force = VNULL;   // initialize to zero the m1-m2 force/torque
    Vector m_torque = VNULL;  // 'intuitive' vectors (can be transformed&added later into Qf)

    // COMPUTE THE FORCES IN THE LINK, FOR EXAMPLE
    // CAUSED BY SPRINGS
    // NOTE!!!!!   C_force and C_torque   are considered in the reference coordsystem
    // of marker2  (the MAIN marker), and their application point is considered the
    // origin of marker1 (the SLAVE marker)

    // 1)========== the generic spring-damper

    if (force_D && force_D->Get_active()) {
        double dfor;
        dfor = force_D->Get_Force((dist - d_restlength), dist_dt, ChTime);
        m_force = Vmul(Vnorm(relM.pos), dfor);

        C_force = Vadd(C_force, m_force);
    }

    // 2)========== the generic torsional spring / torsional damper

    if (force_R && force_R->Get_active()) {
        double tor;
        // 1) the tors. spring
        tor = force_R->Get_Force(relAngle, 0, ChTime);
        m_torque = Vmul(relAxis, tor);
        C_torque = Vadd(C_torque, m_torque);
        // 2) the tors. damper
        double angle_dt = Vlength(relWvel);
        tor = force_R->Get_Force(0, angle_dt, ChTime);
        m_torque = Vmul(Vnorm(relWvel), tor);
        C_torque = Vadd(C_torque, m_torque);
    }

    // 3)========== the XYZ forces

    m_force = VNULL;

    if (force_X && force_X->Get_active()) {
        m_force.x() = force_X->Get_Force(relM.pos.x(), relM_dt.pos.x(), ChTime);
    }

    if (force_Y && force_Y->Get_active()) {
        m_force.y() = force_Y->Get_Force(relM.pos.y(), relM_dt.pos.y(), ChTime);
    }

    if (force_Z && force_Z->Get_active()) {
        m_force.z() = force_Z->Get_Force(relM.pos.z(), relM_dt.pos.z(), ChTime);
    }

    C_force = Vadd(C_force, m_force);

    // 4)========== the RxRyRz forces (torques)

    m_torque = VNULL;

    if (force_Rx && force_Rx->Get_active()) {
        m_torque.x() = force_Rx->Get_Force(relRotaxis.x(), relWvel.x(), ChTime);
    }

    if (force_Ry && force_Ry->Get_active()) {
        m_torque.y() = force_Ry->Get_Force(relRotaxis.y(), relWvel.y(), ChTime);
    }

    if (force_Rz && force_Rz->Get_active()) {
        m_torque.z() = force_Rz->Get_Force(relRotaxis.z(), relWvel.z(), ChTime);
    }

    C_torque = Vadd(C_torque, m_torque);
}

/////////
/////////   COMPLETE UPDATE
/////////
/////////

void ChLinkMasked::Update(double time, bool update_assets) {
    // 1 -
    UpdateTime(time);

    // 2 -
    UpdateRelMarkerCoords();

    // 3 -
    UpdateState();

    // 3b-
    UpdateCqw();

    // 4 -
    UpdateForces(time);
    
    // Inherit time changes of parent class (ChLinkMarkers)
    ChLinkMarkers::Update(time, update_assets);
}

// Define some  link-specific flags for backward compatibility

#define LF_INACTIVE (1L << 0)
#define LF_BROKEN (1L << 2)
#define LF_DISABLED (1L << 4)

void ChLinkMasked::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMasked>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);

    // serialize all member data:
    // marchive << CHNVP(mask); // to do? needed?
    marchive << CHNVP(d_restlength);
    marchive << CHNVP(force_D);
    marchive << CHNVP(force_R);
    marchive << CHNVP(force_X);
    marchive << CHNVP(force_Y);
    marchive << CHNVP(force_Z);
    marchive << CHNVP(force_Rx);
    marchive << CHNVP(force_Ry);
    marchive << CHNVP(force_Rz);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMasked::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkMasked>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);

    // deserialize all member data:
    // if (mask) delete (mask); marchive >> CHNVP(mask); // to do? needed?
    marchive >> CHNVP(d_restlength);
    marchive >> CHNVP(force_D);
    marchive >> CHNVP(force_R);
    marchive >> CHNVP(force_X);
    marchive >> CHNVP(force_Y);
    marchive >> CHNVP(force_Z);
    marchive >> CHNVP(force_Rx);
    marchive >> CHNVP(force_Ry);
    marchive >> CHNVP(force_Rz);
}

}  // end namespace chrono
