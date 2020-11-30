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
// Authors: Simone Benatti
// =============================================================================
//
// Structures for links, contacts, body properties in PBD systems and their lists
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/PBD/ChPBDutils.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h"
#include <Eigen/Core>


namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkPBD)

ChLinkPBD::ChLinkPBD() : p_dir(ChVector<double>(0, 0, 0)), r_dir(ChVector<double>(0, 0, 0)), p_free(false), r_free(false) {}

void ChLinkPBD::SolvePositions() {
	assert(!Body1->GetBodyFixed() || !Body2->GetBodyFixed());

	double invm1 = (1 / Body1->GetMass());
	double invm2 = (1 / Body2->GetMass());
	ChMatrix33<> Inv_I1 = Body1->GetInvInertia();
	ChMatrix33<> Inv_I2 = Body2->GetInvInertia();
	if (Body1->GetBodyFixed()) {
		Inv_I1.setZero();
		invm1 = 0;
	}
	if (Body2->GetBodyFixed()) {
		Inv_I2.setZero();
		invm2 = 0;
	}
	
	// if position free skip completely
	if (!r_free) {
		ChVector<> nr = getQdelta();
		double theta = nr.Length();
		if (nr.Normalize()) {
			auto w1r = nr.eigen().transpose() * Inv_I1 * nr.eigen();
			auto w2r = nr.eigen().transpose() * Inv_I2 * nr.eigen();
			double w1_rot = w1r(0, 0);
			double w2_rot = w2r(0, 0);

			double delta_lambda_t = -(theta + alpha * lambda_t) / (w1_rot + w2_rot + alpha);
			lambda_t += delta_lambda_t;

			ChVector<> pr = delta_lambda_t * nr;

			ChQuaternion<double> dq1, dq2;

			ChVector<> Rot1 = Inv_I1 * pr.eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq1.Qdt_from_Wabs(Rot1, Body1->GetRot());
			// q1 = q0 + dq/dt * h
			Body1->SetRot((Body1->GetRot() + dq1).Normalize());

			ChVector<> Rot2 = Inv_I2 * pr.eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq2.Qdt_from_Wabs(Rot2, Body2->GetRot());
			// q1 = q0 + dq/dt * h
			Body2->SetRot((Body2->GetRot() - dq2).Normalize());
		}
	}

	// if position free skip completely
	if (!p_free) {
		// Position violation in abs coord. This is the distance FROM the desired point TO the constraint frame.
		// It's easy to check using alpha=0 and r1,r2 parallel to n in eq 2,3,4,6: p (correction) is oppsoed to n (due to - in 4)
		// Constraint dist in absolute reference 
		ChVector<> r1 = Body1->GetRot().Rotate(f1.coord.pos);
		ChVector<> r2 = Body2->GetRot().Rotate(f2.coord.pos);
		//ChVector<> n0 = Body2->TransformPointLocalToParent(f2.coord.pos) - Body1->TransformPointLocalToParent(f1.coord.pos);
		ChVector<> n0 = Body1->GetPos() + r1 - (Body2->GetPos() + r2);
		// Rotation of the link frame w.r.t. global frame
		ChQuaternion<> q = Body1->GetRot() * f1.coord.rot;
		// get rid of unconstrained directions by element wise multiplication in the link frame reference
		ChVector<> n_loc = (q.RotateBack(n0))*p_dir;
		// Now we bring the violation back in global coord and normaize it after saving its length
		ChVector<> n = q.Rotate(n_loc);
		double C = n.Length();
		n *= 1 / C;
		if (n.Normalize()) {

			auto Ii1 = ((r1.Cross(n).eigen()).transpose()) * Inv_I1 * (r1.Cross(n).eigen());
			auto Ii2 = ((r2.Cross(n).eigen()).transpose()) * Inv_I2 * (r2.Cross(n).eigen());
			assert(Ii1.cols() * Ii1.rows() * Ii2.cols() * Ii2.rows() == 1);
			double w1 = invm1 + Ii1(0, 0);
			double w2 = invm2 + Ii2(0, 0);

			double delta_lambda_f = -(C + alpha * lambda_f) / (w1 + w2 + alpha);
			lambda_f += delta_lambda_f;
			ChVector<> p = delta_lambda_f * n;

			Body1->SetPos(Body1->GetPos() + p *  invm1);
			Body2->SetPos(Body2->GetPos() - p *  invm2);

			ChQuaternion<double> dq1, dq2;

			ChVector<> Rot1 = Inv_I1 * r1.Cross(p).eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq1.Qdt_from_Wabs(Rot1, Body1->GetRot());
			// q1 = q0 + dq/dt * h
			Body1->SetRot((Body1->GetRot() + dq1).Normalize());

			ChVector<> Rot2 = Inv_I2 * r2.Cross(p).eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq2.Qdt_from_Wabs(Rot2, Body2->GetRot());
			// q1 = q0 + dq/dt * h
			ChQuaternion<> q0 = Body2->GetRot();
			ChQuaternion<> qnew = (q0 - dq2);
			qnew.Normalize();
			Body2->SetRot(qnew);
		}
	}
}

void ChLinkPBD::findRDOF() {
	if (mask[3] & mask[4] & mask[5]) {
		r_dof = NONE;
	}
	else if (mask[3] & mask[4] & !mask[5]) {
		r_dof = Z;
		a = VECT_Z;
	}
	else if (mask[3] & !mask[4] & mask[5]) {
		r_dof = Y;
		a = VECT_Y;
	}
	else if (!mask[3] & mask[4] & mask[5]) {
		r_dof = X;
		a = VECT_X;
	}
}

ChVector<> ChLinkPBD::getQdelta() {
	// Orientation of the 2 link frames
	ChQuaternion<> ql1 = Body1->GetRot() * f1.GetCoord().rot;
	ChQuaternion<> ql2 = Body2->GetRot() * f2.GetCoord().rot;
	if (r_dof == NONE) {
		// TODO: check this, eq 18 says q1 q2^(-1), my calculation say q1^(-1)*q2
		ql1.GetConjugate();
		ChQuaternion<> qdelta =  ql1 * ql2 ;
		return qdelta.GetVector() * 2;
	}
	else  {
		// eq 20: get the rotational DOF directions in the abs frame
		ChVector<> a1 = ql1.Rotate(a) ;
		ChVector<> a2 = ql2.Rotate(a);
		ChVector<> deltarot = a1 % a2;
		return deltarot;
	}
}

ChLinkPBDLock::ChLinkPBDLock(ChLinkLock* alink) : ChLinkPBD() {
	link = alink;
	Body1 = dynamic_cast<ChBody*>(link->GetBody1());
	Body2 = dynamic_cast<ChBody*>(link->GetBody2());
	f1 = ChFrame<>(link->GetMarker1()->GetCoord());
	f2 = ChFrame<>(link->GetMarker2()->GetCoord());
	ChLinkMask& p_mask = link->GetMask();
	ChLinkMaskLF* m_lf = dynamic_cast<ChLinkMaskLF*>(&p_mask);
	mask[0] = m_lf->Constr_X().GetMode() == CONSTRAINT_LOCK;
	mask[1] = m_lf->Constr_Y().GetMode() == CONSTRAINT_LOCK;
	mask[2] = m_lf->Constr_Z().GetMode() == CONSTRAINT_LOCK;
	//mask[3] = m_lf->Constr_E0().GetMode() == CONSTRAINT_LOCK;
	mask[3] = m_lf->Constr_E1().GetMode() == CONSTRAINT_LOCK;
	mask[4] = m_lf->Constr_E2().GetMode() == CONSTRAINT_LOCK;
	mask[5] = m_lf->Constr_E3().GetMode() == CONSTRAINT_LOCK;
	p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
	p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
	r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
	findRDOF();
}



ChLinkPBDMate::ChLinkPBDMate(ChLinkMateGeneric* alink) : ChLinkPBD() {
	MGlink = alink;
	Body1 = dynamic_cast<ChBody*>(MGlink->GetBody1());
	Body2 = dynamic_cast<ChBody*>(MGlink->GetBody2());
	f1 = ChFrame<double>(MGlink->GetFrame1());
	f2 = ChFrame<double>(MGlink->GetFrame2());
	mask[0] = MGlink->IsConstrainedX();
	mask[1] = MGlink->IsConstrainedY();
	mask[2] = MGlink->IsConstrainedZ();
	mask[3] = MGlink->IsConstrainedRx();
	mask[4] = MGlink->IsConstrainedRy();
	mask[5] = MGlink->IsConstrainedRz();
	p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
	p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
	r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
	findRDOF();
}

/*ChLinkPBD::ChLinkPBD(const ChLinkPBD& other) :  p_dir(ChVector<double>(0, 0, 0)), r_dir(ChVector<double>(0, 0, 0)), f1(ChFrame<double>(VNULL)), f2(ChFrame<double>(VNULL)), p_free(false), r_free(false) {
	ChLinkPBD(other.link);
}*/

/*
void ChSystemNSC::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystemNSC>();

    // serialize parent class
    ChSystem::ArchiveOUT(marchive);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void ChSystemNSC::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSystemNSC>();

    // deserialize parent class
    ChSystem::ArchiveIN(marchive);

    // stream in all member data:
}
*/
}  // end namespace chrono
