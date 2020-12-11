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
// TODO: these initialization are probably useless
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
			lambda_t_dir = nr;
			auto w1r = nr.eigen().transpose() * Inv_I1 * nr.eigen();
			auto w2r = nr.eigen().transpose() * Inv_I2 * nr.eigen();
			double w1_rot = w1r(0, 0);
			double w2_rot = w2r(0, 0);

			double delta_lambda_t = -(theta + alpha * lambda_t) / (w1_rot + w2_rot + alpha);
			lambda_t += delta_lambda_t;

			ChVector<> pr = delta_lambda_t * nr;

			ChQuaternion<double> dq1, dq2, qnew1, qnew2;

			ChVector<> Rot1 = Inv_I1 * pr.eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq1.Qdt_from_Wabs(Rot1, Body1->GetRot());
			// q1 = q0 + dq/dt * h
			qnew1 = Body1->GetRot() + dq1;
			qnew1.Normalize();
			Body1->SetRot(qnew1);

			ChVector<> Rot2 = Inv_I2 * pr.eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq2.Qdt_from_Wabs(Rot2, Body2->GetRot());
			// q1 = q0 + dq/dt * h
			qnew2 = Body2->GetRot() - dq2;
			qnew2.Normalize();
			Body2->SetRot(qnew2);
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
		
		if (n.Normalize()) {
			lambda_f_dir = n;
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
			ChQuaternion<> q01 = Body1->GetRot();
			ChQuaternion<> qnew1 = (q01 + dq1);
			qnew1.Normalize();
			Body1->SetRot(qnew1);

			ChVector<> Rot2 = Inv_I2 * r2.Cross(p).eigen();
			// {q_dt} = 1/2 {0,w}*{q}
			dq2.Qdt_from_Wabs(Rot2, Body2->GetRot());
			// q1 = q0 + dq/dt * h
			ChQuaternion<> q02 = Body2->GetRot();
			ChQuaternion<> qnew2 = (q02 - dq2);
			qnew2.Normalize();
			Body2->SetRot(qnew2);
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

ChContactPBD::ChContactPBD(ChBody* body1, ChBody* body2, ChFrame<>& frame1, ChFrame<>& frame2, double frict_d) : ChLinkPBD() {

	Body1 = body1;
	Body2 = body2;
	f1 = frame1;
	f2 = frame2;
	mu_d = frict_d;
	mask[0] = false;
	mask[1] = false;
	mask[2] = false;
	mask[3] = false;
	mask[4] = false;
	mask[5] = false;
	//p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
	// If the contact is violated it is NOT free, is dist > 0 we skip the correction completely
	p_free = false;

	// Rotations are not constrained by contacts. Plus, we don't provide a rolling friction model yet.
	r_free = true;
	//findRDOF();
	//r_dir.Set(int(mask[3]), int(mask[4]), int(mask[5]));
}

// Adjust tangential velocity of bodies 
void ChContactPBD::SolveContactPositions(double h) {
	ChVector<double> p1 = Body1->GetPos() + Body1->GetRot().Rotate(f1.coord.pos);
	ChVector<double> p2 = Body2->GetPos() + Body2->GetRot().Rotate(f2.coord.pos);
	ChVector<double> dist = p2 - p1;
	ChQuaternion<double> q = f1.coord.rot * Body1->GetRot();
	// n is the X axis of the contact "link"
	ChVector<double> n = q.Rotate(VECT_X);
	d = (dist) ^ n;
	
	v_rel = Body2->GetPos_dt() + Body2->GetWvel_par().Cross(f2.coord.pos) - Body1->GetPos_dt() + Body1->GetWvel_par().Cross(f1.coord.pos);
	v_rel_t = v_rel - n * (v_rel^n);
	// If the distance is positive, just skip
	if (d > 0) {
		return;
	}
	else {
		double lam_n = lambda_f_dir ^ n;
		ChVector<double> lambda_contact_t = lambda_f_dir - lam_n^n;
		// 
		mask[0] = true;
		// Check if static -> dynamic
		// TODO: use static friction here!!
		if (!is_dynamic && lambda_contact_t.Length() > mu_d*lam_n ) {
			is_dynamic = true;
			mask[1] = false;
			mask[2] = false;
		}
		// Check if dynamic -> static
		// kinetic energy < friction work in the substep, aka the body would stop within the substep
		// 0.5*m*v^2 < mu_d * f * v * h
		else if (is_dynamic && (Body1->GetMass() + Body2->GetMass())*v_rel_t.Length() < 4*mu_d*lambda_contact_t.Length()*h) {
			is_dynamic = false;
			mask[1] = true;
			mask[2] = true;
		}

		// Treat the contact as a link
		p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
		SolvePositions();
	}
}

// Adjust tangential velocity of bodies 
void ChContactPBD::SolveVelocity() {
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
