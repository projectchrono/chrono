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

#include "chrono/physics/ChPBDLinks.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h"
#include "chrono/physics/ChSystemPBD.h"
#include <Eigen/Core>


namespace chrono {

	void ChLinkPBD::EvalMasses() {
		invm1 = (1 / Body1->GetMass());
		invm2 = (1 / Body2->GetMass());
		Inv_I1 = Body1->GetInvInertia();
		Inv_I2 = Body2->GetInvInertia();
		if (Body1->GetBodyFixed()) {
			Inv_I1.setZero();
			invm1 = 0;
		}
		if (Body2->GetBodyFixed()) {
			Inv_I2.setZero();
			invm2 = 0;
		}
	}
	void ChLinkPBD::SolvePositions() {
		assert(!Body1->GetBodyFixed() || !Body2->GetBodyFixed());

		// if position free skip completely
		if (!r_free) {
			ChVector<> nr = getQdelta();
			double theta = nr.Length();
			if (nr.Normalize()) {
				ChVector<> nr1 = Body1->TransformDirectionParentToLocal(nr);
				ChVector<> nr2 = Body2->TransformDirectionParentToLocal(nr);
				lambda_t_dir = nr;
				Eigen::Matrix<double, 1, 1> w1r = nr1.eigen().transpose() * Inv_I1 * nr1.eigen();
				Eigen::Matrix<double, 1, 1> w2r = nr2.eigen().transpose() * Inv_I2 * nr2.eigen();
				w1_rot = w1r(0, 0);
				w2_rot = w2r(0, 0);

				double delta_lambda_t = -(theta + alpha * lambda_t) / (w1_rot + w2_rot + alpha);
				lambda_t += delta_lambda_t;

				ChVector<> pr = delta_lambda_t * nr;

				//ChQuaternion<> dq1, dq2, qnew1, qnew2;

				ChQuaternion<> Rot1(0, Inv_I1 * Body1->TransformDirectionParentToLocal(pr).eigen());
				// {q_dt} = 1/2 {0,w}*{q}
				//dq1.Qdt_from_Wrel(Rot1, Body1->GetRot());
				// q1 = q0 + dq/dt * h
				ChQuaternion<> qnew1 = Body1->GetRot() + Body1->GetRot()*Rot1*0.5;
				qnew1.Normalize();
				Body1->SetRot(qnew1);

				ChQuaternion<> Rot2(0, Inv_I2 * Body2->TransformDirectionParentToLocal(pr).eigen());
				// {q_dt} = 1/2 {0,w}*{q}
				//dq2.Qdt_from_Wrel(Rot2, Body2->GetRot());
				// q1 = q0 + dq/dt * h
				ChQuaternion<> qnew2 = Body2->GetRot() - Body2->GetRot()*Rot2*0.5;
				qnew2.Normalize();
				Body2->SetRot(qnew2);
			}
		}

		// if position free skip completely
		if (!p_free || is_displ_limited || displ_actuated) {
			// Position violation in abs coord. This is the distance FROM the desired point TO the constraint frame.
			// It's easy to check using alpha=0 and r1,r2 parallel to n in eq 2,3,4,6: p (correction) is oppsoed to n (due to - in 4)
			// Constraint dist in absolute reference 
			ChVector<> r1 = Body1->TransformDirectionLocalToParent(f1.coord.pos);
			ChVector<> r2 = Body2->TransformDirectionLocalToParent(f2.coord.pos);
			ChVector<> n0 = Body1->GetPos() + r1 - (Body2->GetPos() + r2);

			// Rotation of the link frame w.r.t. global frame
			//ChQuaternion<> q = f1.coord.rot * Body1->GetRot();
			ChMatrix33<> M = f1.GetA() * Body1->GetA();
			// get rid of unconstrained directions by element wise multiplication in the link frame reference
			ChVector<> n_loc = (M.transpose()*n0)*p_dir;
			// Add the correction due to limit violation to non dist constr
			if (is_displ_limited || (displ_actuated && !dist_constr)) n_loc -= ApplyDisplLimAct(M.transpose()*n0);
			// Now we bring the violation back in global coord and normaize it after saving its length
			ChVector<> nt = M*n_loc;
			double C = nt.Length();
			if (dist_constr){
				if (displ_actuated)
					dist = motor_func->Get_y(PBDsys->T);
				C -= dist;
				if (abs(C) < 1E-12) {
					return;
					}
				nt *= (C/abs(C));
				C = abs(C);
			}

			if (nt.Normalize()) {
				lambda_f_dir = nt;
				ChVector<> n1 = Body1->TransformDirectionParentToLocal(nt);
				ChVector<> n2 = Body2->TransformDirectionParentToLocal(nt);
				Eigen::Matrix<double, 1, 1> Ii1 = ((f1.coord.pos.Cross(n1).eigen()).transpose()) * Inv_I1 * (f1.coord.pos.Cross(n1).eigen());
				Eigen::Matrix<double, 1, 1> Ii2 = ((f2.coord.pos.Cross(n2).eigen()).transpose()) * Inv_I2 * (f2.coord.pos.Cross(n2).eigen());
				w1 = invm1 + Ii1(0, 0);
				w2 = invm2 + Ii2(0, 0);

				double delta_lambda_f = -(C + alpha * lambda_f) / (w1 + w2 + alpha);
				lambda_f += delta_lambda_f;
				ChVector<> p = delta_lambda_f * nt;

				Body1->SetPos(Body1->GetPos() + p *  invm1);
				Body2->SetPos(Body2->GetPos() - p *  invm2);

				ChVector<> Rot1 = Inv_I1 * Body1->TransformDirectionParentToLocal(r1.Cross(p)).eigen();
				ChQuaternion<> q01 = Body1->GetRot();
				ChQuaternion<> dq1(0, Rot1);
				ChQuaternion<> qnew1 = (q01 + q01*dq1*0.5);
				qnew1.Normalize();
				Body1->SetRot(qnew1);

				ChVector<> Rot2 = Inv_I2 * Body2->TransformDirectionParentToLocal(r2.Cross(p)).eigen();
				ChQuaternion<> q02 = Body2->GetRot();
				ChQuaternion<> dq2(0, Rot2);
				ChQuaternion<> qnew2 = (q02 - q02*dq2*0.5);
				qnew2.Normalize();
				Body2->SetRot(qnew2);
			}
		}
	}

	void ChLinkPBD::findRDOF() {
		if (mask[3] & mask[4] & mask[5]) {
			r_locked = true;
		}
		else if (mask[3] & mask[4] & !mask[5]) {
			a = VECT_Z;
		}
		else if (mask[3] & !mask[4] & mask[5]) {
			a = VECT_Y;
		}
		else if (!mask[3] & mask[4] & mask[5]) {
			a = VECT_X;
		}
	}

	ChVector<> ChLinkPBD::getQdelta() {
		// Orientation of the 2 link frames
		if (r_locked) {
			ChQuaternion<> ql1 = Body1->GetRot()*f1.GetCoord().rot;
			ChQuaternion<> ql2 = Body2->GetRot()*f2.GetCoord().rot;
			// eq. 18 PBD paper
			ChQuaternion<> qdelta = ql1*ql2.GetInverse();
			// For angle and speed rotational actuators: the target q is rotated of the target angle "alpha" about the rot axis "a"
			if (rot_actuated) {
				double alpha = motor_func->Get_y(PBDsys->T);
				if (speed_actuated) {
					alpha *= PBDsys->h;
					alpha += old_val;
					old_val = alpha;
				}
				ChQuaternion<> q_act;
				q_act.Q_from_AngAxis(alpha, a);
				qdelta = ql1*q_act*ql2.GetInverse();
			}
			return qdelta.GetVector() * 2;
		}
		else {
			ChMatrix33<> M1 = Body1->GetA()*f1.GetA();
			ChMatrix33<> M2 = Body2->GetA()*f2.GetA();
			// eq 20: get the rotational DOF directions in the abs frame
			ChVector<> a1 = M1*a;
			ChVector<> a2 = M2*a;
			ChVector<> deltarot = a2 % a1;
			return deltarot;
		}
	}

	ChVector<> ChLinkPBD::ApplyDisplLimAct(ChVector<> local_disp) {
		ChVector<> correction;
		if (displ_actuated) {
			double targ = motor_func->Get_y(PBDsys->T);
			if (speed_actuated) {
				// Correct of the amount necessary to acheive the target speed by the end of the substep
				double relspeed = Body1->TransformDirectionParentToLocal(Body2->GetPos_dt() - Body1->GetPos_dt())[actuation_dir];
				double corr = -(targ-relspeed)*PBDsys->h;
				correction.Set(corr,0,0);
			}
			else{
				correction[actuation_dir] = targ - local_disp[actuation_dir];
			};
		}
		if (is_displ_limited) {
			for (unsigned i = 0; i < 3; i++) {
				if (displ_lims[i]) {
					correction[i] += ChMin(displ_lims_high[i] - local_disp[i], 0.0);
					correction[i] += ChMax(displ_lims_low[i] - local_disp[i], 0.0);
				}
			}
		}
		return correction;
	}

	ChLinkPBDLock::ChLinkPBDLock(ChLinkLock* alink, ChSystemPBD* sys) : ChLinkPBD(sys) {
		link = alink;
		Body1 = dynamic_cast<ChBody*>(link->GetBody2());
		Body2 = dynamic_cast<ChBody*>(link->GetBody1());
		f1 = ChFrame<>(link->GetMarker2()->GetCoord());
		f2 = ChFrame<>(link->GetMarker1()->GetCoord());
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
		p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
		r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
		findRDOF();
		EvalMasses();
		SetLimits();
	}

	void ChLinkPBDLock::SetLimits() {
		ChLinkLimit& limx = link->GetLimit_X();
		ChLinkLimit& limy = link->GetLimit_Y();
		ChLinkLimit& limz = link->GetLimit_Z();
		ChLinkLimit& limRx = link->GetLimit_Rx();
		ChLinkLimit& limRy = link->GetLimit_Ry();
		ChLinkLimit& limRz = link->GetLimit_Rz();
		if (limx.IsActive()) {
			is_displ_limited = true;
			displ_lims[0] = true;
			displ_lims_low[0] = limx.GetMin();
			displ_lims_high[0] = limx.GetMax();
		}
		if (limy.IsActive()) {
			is_displ_limited = true;
			displ_lims[1] = true;
			displ_lims_low[1] = limy.GetMin();
			displ_lims_high[1] = limy.GetMax();
		}
		if (limz.IsActive()) {
			is_displ_limited = true;
			displ_lims[2] = true;
			displ_lims_low[2] = limz.GetMin();
			displ_lims_high[2] = limz.GetMax();
		}
		if (limRx.IsActive()) {
			is_displ_limited = true;
			rot_lims_low[0] = limRx.GetMin();
			rot_lims_high[0] = limRx.GetMax();
		}
		if (limRy.IsActive()) {
			is_displ_limited = true;
			rot_lims_low[1] = limRy.GetMin();
			rot_lims_high[1] = limRy.GetMax();
		}
		if (limRz.IsActive()) {
			is_displ_limited = true;
			rot_lims_low[2] = limRz.GetMin();
			rot_lims_high[2] = limRz.GetMax();
		}
	}


	ChLinkPBDMate::ChLinkPBDMate(ChLinkMateGeneric* alink, ChSystemPBD* sys) : ChLinkPBD(sys) {
		MGlink = alink;
		Body1 = dynamic_cast<ChBody*>(MGlink->GetBody2());
		Body2 = dynamic_cast<ChBody*>(MGlink->GetBody1());
		f1 = ChFrame<>(MGlink->GetFrame2());
		f2 = ChFrame<>(MGlink->GetFrame1());
		mask[0] = MGlink->IsConstrainedX();
		mask[1] = MGlink->IsConstrainedY();
		mask[2] = MGlink->IsConstrainedZ();
		mask[3] = MGlink->IsConstrainedRx();
		mask[4] = MGlink->IsConstrainedRy();
		mask[5] = MGlink->IsConstrainedRz();
		p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
		p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
		r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
		findRDOF();
		EvalMasses();
	}

	ChLinkPBDMotor::ChLinkPBDMotor(ChLinkMotor* alink, ChSystemPBD* sys) : ChLinkPBDMate(alink, sys) {

		/// The constraint part is managed by ChLinkMate. Here we only add the actuation here.
		/// We only manage speed and position actuators 
		/// force/torque actuators come "for free".
		if (dynamic_cast<const ChLinkMotorRotation*>(alink) != nullptr) {
			ChLinkMotorRotation* motor = dynamic_cast<ChLinkMotorRotation*>(alink);
			rot_actuated = true;
			// Torque motors are revolute mates + torque
			if (dynamic_cast<const ChLinkMotorRotationTorque*>(alink) != nullptr) {
				mask[5] = false;
				r_locked = false;
				findRDOF();
			}
			motor_func = motor->GetMotorFunction();
			a = VECT_Z;
			if (dynamic_cast<const ChLinkMotorRotationSpeed*>(alink) != nullptr) { speed_actuated = true; }
		}
		else if (dynamic_cast<const ChLinkMotorLinear*>(alink) != nullptr) {
			ChLinkMotorLinear* motor = dynamic_cast<ChLinkMotorLinear*>(alink);
			mask[0] = false;
			if (dynamic_cast<const ChLinkMotorLinearForce*>(alink) != nullptr) {
				// nothing to do besides unlocking x for lin force actuator
				return;
			}
			displ_actuated = true;
			actuation_dir = 0;
			p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
			motor_func = motor->GetMotorFunction();
			if (dynamic_cast<const ChLinkMotorLinearSpeed*>(alink) != nullptr) { speed_actuated = true; alpha=0; }
			if (dynamic_cast<const ChLinkMotorLinearForce*>(alink) != nullptr) {
				mask[0] = false;
			}
		}
	}


	ChContactPBD::ChContactPBD(ChBody* body1, ChBody* body2, ChSystemPBD* sys, ChFrame<>& frame1, ChFrame<>& frame2, double frict_d) : ChLinkPBD(sys) {

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
		EvalMasses();
		//p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
		// If the contact is violated it is NOT free, is dist > 0 we skip the correction completely
		p_free = false;
		p_dir.Set(1, 0, 0);
		// Rotations are not constrained by contacts. Plus, we don't provide a rolling friction model yet.
		r_free = true;
		//findRDOF();
		// TODO: set properly alpha according to http://blog.mmacklin.com/
		// rmember to eval alpha_hat = alpha/(h^2)
		alpha = 100;
	}

	// Adjust tangential velocity of bodies 
	void ChContactPBD::SolveContactPositions() {
		ChVector<> p1 = Body1->GetPos() + Body1->TransformDirectionLocalToParent(f1.coord.pos);
		ChVector<> p2 = Body2->GetPos() + Body2->TransformDirectionLocalToParent(f2.coord.pos);
		ChVector<> dist = p2 - p1;
		ChMatrix33<> M = f1.GetA() * Body1->GetA();
		// n is the X axis of the contact "link"
		n = M*VECT_X;
		d = dist ^ n;
		// If the distance is positive, just skip
		if (d > 0) {
			//lambda_contact_tf = 0;
			//lambda_f = 0;
			return;
		}
		else {
			SolvePositions();
			ChVector<> v_rel = (Body1->GetPos_dt() + Body1->TransformDirectionLocalToParent(Body1->GetWvel_loc().Cross(f1.coord.pos)) - Body2->GetPos_dt() - Body2->TransformDirectionLocalToParent(Body2->GetWvel_loc().Cross(f2.coord.pos)));
			// normal velocity before velocity update. Will be used in 
			v_n_old = v_rel^n;

			// Treat the contact as a link	
			if (!is_dynamic) {

				// project static friction
				ChVector<> r1 = Body1->TransformDirectionLocalToParent(f1.coord.pos);
				ChVector<> r2 = Body2->TransformDirectionLocalToParent(f2.coord.pos);
				
				// As in paper: use tangential disp wrt previous s.step:
				//ChVector<> n0 = ( (Body1->GetPos() + r1 - p1_old) - ((Body2->GetPos() + r2) - p2_old ));
				//double n0_n = n0^n;
				//ChVector<> n0_t = n0 - n * n0_n;
				//ChVector<> n_tf = n0_t;

				// Alternative: tg displ wrt contact points. Exact only for the 1sr sstep, does not require storing old pos
				ChVector<> n_tf = Body1->GetPos() + r1 - (Body2->GetPos() + r2);
				// we do not need to project the constraint in the z-y plane since the x violation is already 0 from the previous SolvePositions

				double C = n_tf.Length();
				if (n_tf.Normalize()) {

					ChVector<> n1 = Body1->TransformDirectionParentToLocal(n_tf);
					ChVector<> n2 = Body2->TransformDirectionParentToLocal(n_tf);
					Eigen::Matrix<double, 1, 1> Ii1 = ((f1.coord.pos.Cross(n1).eigen()).transpose()) * Inv_I1 * (f1.coord.pos.Cross(n1).eigen());
					Eigen::Matrix<double, 1, 1> Ii2 = ((f2.coord.pos.Cross(n2).eigen()).transpose()) * Inv_I2 * (f2.coord.pos.Cross(n2).eigen());
					w1_tf = invm1 + Ii1(0, 0);
					w2_tf = invm2 + Ii2(0, 0);

					double delta_lambda_tf = -(C + alpha * lambda_contact_tf) / (w1_tf + w2_tf + alpha);
					ChVector<> p = delta_lambda_tf * n_tf;
					lambda_contact_tf += delta_lambda_tf;
					lambda_tf_dir = n_tf;

					// TODO: use static friction here!!
					if (abs(lambda_contact_tf) > mu_d*abs(lambda_f)) {
						lambda_contact_tf = (lambda_contact_tf / abs(lambda_contact_tf))  * mu_d*abs(lambda_f);
						is_dynamic = true;
					}
					else {
						Body1->SetPos(Body1->GetPos() + p *  invm1);
						Body2->SetPos(Body2->GetPos() - p *  invm2);

						ChVector<> Rot1 = Inv_I1 * Body1->TransformDirectionParentToLocal(r1.Cross(p)).eigen();
						ChQuaternion<> q01 = Body1->GetRot();
						ChQuaternion<> dq1(0, Rot1);
						ChQuaternion<> qnew1 = (q01 + q01*dq1*0.5);
						qnew1.Normalize();
						Body1->SetRot(qnew1);

						ChVector<> Rot2 = Inv_I2 * Body2->TransformDirectionParentToLocal(r2.Cross(p)).eigen();
						ChQuaternion<> q02 = Body2->GetRot();
						ChQuaternion<> dq2(0, Rot2);
						ChQuaternion<> qnew2 = (q02 - q02*dq2*0.5);
						qnew2.Normalize();
						Body2->SetRot(qnew2);
					}
				}
			}
		}
	}

	// Adjust tangential velocity of bodies 
	void ChContactPBD::SolveVelocity() {
		// We do NOT re-evaluate the distance, since after the pos update it will not be negative anymore.
		// We want to correct the valocities of the contct we solved previously, so we keep the old d as discriminator
		if (d > 0) {
			return;
		}
		else
		{
			double h = PBDsys->h;
			ChVector<> v_rel = Body1->GetPos_dt() + Body1->TransformDirectionLocalToParent(Body1->GetWvel_loc().Cross(f1.coord.pos)) - Body2->GetPos_dt() - Body2->TransformDirectionLocalToParent(Body2->GetWvel_loc().Cross(f2.coord.pos));
			double v_rel_n = v_rel^n;
			ChVector<> v_rel_t = v_rel - n * v_rel_n;
			double vt = v_rel_t.Length();
			ChVector<> delta_vn, delta_vt, p;

			// do not compute tangential velocity correction if there is none
			if (is_dynamic) {
				double ft = abs(lambda_contact_tf) / (h*h);
				// similar to eq. 30 but this is different from the paper (which does not make sense dimensionally)
				double threshold = h * ft * (w1_tf + w2_tf);
				if (vt < threshold) {
					is_dynamic = false;
					delta_vt = -v_rel_t;
				}
				else {
					delta_vt = -v_rel_t.GetNormalized() * threshold;
				}
				p += (delta_vt) / (w1_tf + w2_tf);
			}

			// normal speed restitution
			// TODO: make restitution coefficient settable
			double e = (abs(v_rel_n) < 2 * 9.8 * h) ? 0 : 0.005;
			delta_vn = -n * (v_rel_n + ChMax(e*v_n_old, 0.0));

			// apply speed impulses to bodies
			p += delta_vn / (w1 + w2);
			ChVector<> v1 = Body1->GetPos_dt();
			ChVector<> v2 = Body2->GetPos_dt();
			Body1->SetPos_dt(v1 + p*invm1);
			Body2->SetPos_dt(v2 - p*invm2);

			ChVector<> omega1 = Body1->GetWvel_loc();
			ChVector<> omega2 = Body2->GetWvel_loc();
			ChVector<> delta_omega1 = Inv_I1 * f1.coord.pos.Cross(Body1->TransformDirectionParentToLocal(p)).eigen();
			ChVector<> delta_omega2 = Inv_I2 * f2.coord.pos.Cross(Body2->TransformDirectionParentToLocal(p)).eigen();
			Body1->SetWvel_loc(omega1 + delta_omega1);
			Body2->SetWvel_loc(omega2 - delta_omega2);

			//p1_old = Body1->GetPos() + Body1->TransformDirectionLocalToParent(f1.coord.pos);
			//p2_old = Body2->GetPos() + Body2->TransformDirectionLocalToParent(f2.coord.pos);
		}
	}

	ChLinkPBDUniversal::ChLinkPBDUniversal(ChLinkUniversal* alink, ChSystemPBD* sys) : ChLinkPBD(sys) {
		Ulink = alink;
		Body1 = dynamic_cast<ChBody*>(alink->GetBody1());
		Body2 = dynamic_cast<ChBody*>(alink->GetBody2());
		f1 = ChFrame<>(alink->GetFrame1Rel());
		f2 = ChFrame<>(alink->GetFrame2Rel());
		// The universal link constraints all 3 displ DOF
		mask[0] = true;
		mask[1] = true;
		mask[2] = true;
		// The rotation is custom, these are only placeholders.
		mask[3] = false;
		mask[4] = false;
		mask[5] = false;
		p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
		p_free = false;
		r_free = false;
		a = VECT_X;
		EvalMasses();
	}

	ChVector<> ChLinkPBDUniversal::getQdelta() {
		// Orientation of the 2 link frames
		ChQuaternion<> ql1 = f1.GetCoord().rot * Body1->GetRot();
		ChQuaternion<> ql2 = f2.GetCoord().rot * Body2->GetRot();
		// eq 20: get the rotational DOF directions in the abs frame
		// ChLinkUniversal constraints 2 y to be othogonal to 1 x. 
		// In this case 2 y (a) is the normalized projection of y on the 1 z-y plane
		//ChVector<> a1 = (ql2.Rotate(a).Dot(ql1.Rotate(VECT_Y)))*ql1.Rotate(VECT_Y) + (ql2.Rotate(a).Dot(ql1.Rotate(VECT_Z)))*ql1.Rotate(VECT_Z);
		ChVector<> a1 = ql2.Rotate(a) - (ql2.Rotate(a).Dot(ql1.Rotate(VECT_Y))*ql1.Rotate(VECT_Y));
		a1.Normalize();
		ChVector<> a2 = ql2.Rotate(a);
		//ChVector<> a1 = ql1.Rotate(VECT_X);
		ChVector<> deltarot = -a1 % a2;
		return deltarot;
	}


	ChLinkPBDLinActuator::ChLinkPBDLinActuator(ChLinkLinActuator* linact, ChSystemPBD* sys) : ChLinkPBD(sys) {
        link = linact;
		Body1 = dynamic_cast<ChBody*>(link->GetBody2());
		Body2 = dynamic_cast<ChBody*>(link->GetBody1());
        f1 = ChFrame<>(link->GetMarker2()->GetCoord());
        f2 = ChFrame<>(link->GetMarker1()->GetCoord());
		mask[0] = true;
		mask[1] = true;
		mask[2] = true;
		mask[3] = false;
		mask[4] = false;
		mask[5] = false;
		p_dir.Set(1, 1, 1);
		p_free = false;
		r_free = true;
		EvalMasses();
		dist_constr = true;
        displ_actuated = true;
        motor_func = link->Get_dist_funct();
		//displ_lims_low[0] = d - 1E-4;
		//displ_lims_high[0] = d + 1E-4;
	}

	ChLinkPBDDistance::ChLinkPBDDistance(ChLinkDistance* alink, ChSystemPBD* sys) : ChLinkPBD(sys) {
        link = alink;
        Body1 = dynamic_cast<ChBody*>(link->GetBody1());
        Body2 = dynamic_cast<ChBody*>(link->GetBody2());
        f1 = ChFrame<>(link->GetEndPoint1Rel());
        f2 = ChFrame<>(link->GetEndPoint2Rel());
        mask[0] = true;
        mask[1] = true;
        mask[2] = true;
        mask[3] = false;
        mask[4] = false;
        mask[5] = false;
        p_dir.Set(1, 1, 1);
        p_free = false;
        r_free = true;
        EvalMasses();
        dist_constr = true;
        dist = link->GetImposedDistance();
    }
	ChLinkPBD::ChLinkPBD(ChBody* body1, ChBody* body2, ChFrame<>& fr1, ChFrame<>& fr2, 
						bool mmask[6], ChSystemPBD* sys, double impdist, std::shared_ptr<ChFunction> motfun) : ChLinkPBD(sys) {
		//link = alink;
		Body1 = body1;
		Body2 = body2;
		f1 = fr1;
		f2 = fr2;
		mask[0] = mmask[0];
		mask[1] = mmask[1];
		mask[2] = mmask[2];
		mask[3] = mmask[3];
		mask[4] = mmask[4];
		mask[5] = mmask[5];
		EvalMasses();
		if (impdist > 0){
			dist_constr = true;
			dist = impdist;
			}
		if (motfun) {
			displ_actuated = true;
			motor_func = motfun;
		}
		p_dir.Set(int(mask[0]), int(mask[1]), int(mask[2]));
		p_free = (int(mask[0]) + int(mask[1]) + int(mask[2]) == 0) ? true : false;
		r_free = (int(mask[3]) + int(mask[4]) + int(mask[5]) == 0) ? true : false;
		findRDOF();

}

	/*ChLinkPBD::ChLinkPBD(const ChLinkPBD& other) :  p_dir(ChVector<>(0, 0, 0)), r_dir(ChVector<>(0, 0, 0)), f1(ChFrame<double>(VNULL)), f2(ChFrame<double>(VNULL)), p_free(false), r_free(false) {
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
