// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/motion_functions/ChFunctionRotation_ABCfunctions.h"
#include "chrono/motion_functions/ChFunctionPosition_XYZfunctions.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkMotionImposed)

ChLinkMotionImposed::ChLinkMotionImposed() : ChLinkMateGeneric(true, true, true, true, true, true) {
    // default motion and rotation: no rotation no translation
	position_function = chrono_types::make_shared<ChFunctionPosition_XYZfunctions>();
	rotation_function = chrono_types::make_shared<ChFunctionRotation_ABCfunctions>();
}

ChLinkMotionImposed::ChLinkMotionImposed(const ChLinkMotionImposed& other) : ChLinkMateGeneric(other) {
    position_function = other.position_function;
	rotation_function = other.rotation_function;
}

ChLinkMotionImposed::~ChLinkMotionImposed() {}

void ChLinkMotionImposed::Update(double mytime, bool update_assets) {
    // Inherit parent class:
    ChLinkMateGeneric::Update(mytime, update_assets);

    // Override the rotational jacobian [Cq] and the rotational residual C,
    // by assuming an additional hidden frame that rotates about frame2:

    if (this->Body1 && this->Body2) {
        ChFrame<> frame1W = this->frame1 >> (*this->Body1);
        ChFrame<> frame2W = this->frame2 >> (*this->Body2);

		frameM2.SetRot(rotation_function->Get_q(mytime));
		frameM2.SetPos(position_function->Get_p(mytime));
		
		frameMb2 = frameM2 >> this->frame2;

		ChFrame<> frameMW = frameM2 >> frame2W;

        ChFrame<> frame1M;
        frameMW.TransformParentToLocal(frame1W, frame1M);

        ChMatrix33<> planeMW = frameMW.GetA();

        ChMatrix33<> Jw1 = planeMW.transpose() * Body1->GetA();
        ChMatrix33<> Jw2 = -planeMW.transpose() * Body2->GetA();

		ChMatrix33<> Jx1 = planeMW.transpose();
        ChMatrix33<> Jx2 = -planeMW.transpose();

        ChMatrix33<> Jr1 = -Jw1 * ChStarMatrix33<>(frame1.GetPos());
        ChMatrix33<> Jr2 = -Jw2 * ChStarMatrix33<>(frameMb2.GetPos());

		//???? not needed, because frame1W and frameRW should always coincide when x y z translations are constrained
        //ChVector<> p2p1_base2 = Body2->GetA().transpose() * (p1_abs - p2_abs); 
        //Jr2 += frame2.GetA().transpose() * ChStarMatrix33<>(p2p1_base2);

        // Premultiply by Jw1 and Jw2 by  0.5*[Fp(q_resid)]' to get residual as imaginary part of a quaternion.
        ChStarMatrix33<> mtempM(frame1M.GetRot().GetVector() * 0.5);
        mtempM(0, 0) = 0.5 * frame1M.GetRot().e0();
        mtempM(1, 1) = 0.5 * frame1M.GetRot().e0();
        mtempM(2, 2) = 0.5 * frame1M.GetRot().e0();

        ChMatrix33<> mtempQ;
        mtempQ = mtempM.transpose() * Jw1;
        Jw1 = mtempQ;
        mtempQ = mtempM.transpose() * Jw2;
        Jw2 = mtempQ;

        int nc = 0;

        if (c_x) {
            C(nc) = frame1M.GetPos().x();
            mask.Constr_N(nc).Get_Cq_a().segment(0, 3) = Jx1.row(0);
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jr1.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(0, 3) = Jx2.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jr2.row(0);
            nc++;
        }
        if (c_y) {
            C(nc) = frame1M.GetPos().y();
            mask.Constr_N(nc).Get_Cq_a().segment(0, 3) = Jx1.row(1);
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jr1.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(0, 3) = Jx2.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jr2.row(1);
            nc++;
        }
        if (c_z) {
            C(nc) = frame1M.GetPos().z();
            mask.Constr_N(nc).Get_Cq_a().segment(0, 3) = Jx1.row(2);
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jr1.row(2);
            mask.Constr_N(nc).Get_Cq_b().segment(0, 3) = Jx2.row(2);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jr2.row(2);
            nc++;
        }
        if (c_rx) {
            C(nc) = frame1M.GetRot().e1();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(0);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(0);
            nc++;
        }
        if (c_ry) {
            C(nc) = frame1M.GetRot().e2();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(1);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(1);
            nc++;
        }
        if (c_rz) {
            C(nc) = frame1M.GetRot().e3();
            mask.Constr_N(nc).Get_Cq_a().setZero();
            mask.Constr_N(nc).Get_Cq_b().setZero();
            mask.Constr_N(nc).Get_Cq_a().segment(3, 3) = Jw1.row(2);
            mask.Constr_N(nc).Get_Cq_b().segment(3, 3) = Jw2.row(2);
            nc++;
        }
    }
}

void ChLinkMotionImposed::IntLoadConstraint_Ct(const unsigned int off_L, ChVectorDynamic<>& Qc, const double c) {
    
	double T = this->GetChTime();
	ChVector<> mv = -position_function->Get_p_ds(T);
	ChVector<> mw_loc = -rotation_function->Get_w_loc(T);
	ChVector<> mv_rot = rotation_function->Get_q(T).RotateBack(mv); // need velocity in local rotated system

	int nc = 0;
    if (c_x) {
        if (mask.Constr_N(nc).IsActive())
			Qc(off_L + nc) += c * mv_rot.x();
        nc++;
    }
    if (c_y) {
        if (mask.Constr_N(nc).IsActive())
            Qc(off_L + nc) += c * mv_rot.y();
        nc++;
    }
    if (c_z) {
        if (mask.Constr_N(nc).IsActive())
            Qc(off_L + nc) += c * mv_rot.z();
        nc++;
    }
    if (c_rx) {
        if (mask.Constr_N(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.x();
        nc++;
    }
    if (c_ry) {
        if (mask.Constr_N(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.y();
        nc++;
    }
    if (c_rz) {
        if (mask.Constr_N(nc).IsActive())
            Qc(off_L + nc) += c * 0.5 * mw_loc.z();
        nc++;
    }

}

// OLD
void ChLinkMotionImposed::ConstraintsBiLoad_Ct(double factor) {
    if (!this->IsActive())
        return;

    double T = this->GetChTime();
	ChVector<> mv = -position_function->Get_p_ds(T);
	ChVector<> mw_loc = -rotation_function->Get_w_loc(T);
	ChVector<> mv_rot = rotation_function->Get_q(T).RotateBack(mv); // need velocity in local rotated system

    if (mask.Constr_N(0).IsActive()) {
        mask.Constr_N(0).Set_b_i(mask.Constr_N(0).Get_b_i() + factor * mv_rot.x());
    }
	if (mask.Constr_N(1).IsActive()) {
        mask.Constr_N(1).Set_b_i(mask.Constr_N(1).Get_b_i() + factor * mv_rot.y());
    }
	if (mask.Constr_N(2).IsActive()) {
        mask.Constr_N(2).Set_b_i(mask.Constr_N(2).Get_b_i() + factor * mv_rot.z());
    }
	if (mask.Constr_N(3).IsActive()) {
        mask.Constr_N(3).Set_b_i(mask.Constr_N(3).Get_b_i() + factor * 0.5 * mw_loc.x());
    }
	if (mask.Constr_N(4).IsActive()) {
        mask.Constr_N(4).Set_b_i(mask.Constr_N(4).Get_b_i() + factor * 0.5 * mw_loc.y());
    }
	if (mask.Constr_N(5).IsActive()) {
        mask.Constr_N(5).Set_b_i(mask.Constr_N(5).Get_b_i() + factor * 0.5 * mw_loc.z());
    }
}

void ChLinkMotionImposed::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkMotionImposed>();

    // serialize parent class
    ChLinkMateGeneric::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(position_function);
	marchive << CHNVP(rotation_function);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkMotionImposed::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkMotionImposed>();

    // deserialize parent class
    ChLinkMateGeneric::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(position_function);
	marchive >> CHNVP(rotation_function);
}

}  // end namespace chrono
