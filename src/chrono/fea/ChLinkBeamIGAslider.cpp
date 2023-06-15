// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/fea/ChLinkBeamIGAslider.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkBeamIGAslider)

ChLinkBeamIGAslider::ChLinkBeamIGAslider() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkBeamIGAslider::ChLinkBeamIGAslider(const ChLinkBeamIGAslider& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
}

ChCoordsys<> ChLinkBeamIGAslider::GetLinkAbsoluteCoords() {
    if (m_body) {
        ChCoordsys<> linkcsys = m_csys >> (*m_body);
        return linkcsys;
    }
    return CSYSNORM;
}

void ChLinkBeamIGAslider::UpdateNodes() {
    m_nodes.clear();

    this->order = (int)m_beams[0]->GetNodes().size() - 1;

    // fetch the element and the tau
    this->tau = 0;
    this->active_element = 0;
    int nnodes = (int)this->m_nodes.size();
    ChVector<> point = VNULL;
    ChVector<> goodpoint = VNULL;
    ChVectorDynamic<> N(nnodes);
    int nsamples = 6;  //***TODO*** search via NR
    double mdist = 1e30;
    ChVector<> absoutlet = this->m_body->TransformPointLocalToParent(this->m_csys.pos);

    for (size_t iel = 0; iel < m_beams.size(); ++iel) {
        double u1 = m_beams[iel]->GetU1();
        double u2 = m_beams[iel]->GetU2();

        for (int ip = 0; ip < nsamples; ++ip) {
            double u = u1 + (u2 - u1) * (double)ip / ((double)(nsamples - 1));
            double eta = (2.0 * (u - u1) / (u2 - u1)) - 1.0;
            m_beams[iel]->EvaluateSectionPoint(eta, point);

            double trydist = (point - absoutlet).Length();
            if (trydist < mdist) {
                mdist = trydist;
                this->tau = u;
                this->active_element = iel;
                goodpoint = point;
            }
        }
    }
    //***TEST
    // double u1 = m_beams[this->active_element]->GetU1();
    // double u2 = m_beams[this->active_element]->GetU2();
    // double eta = (2.0*(this->tau - u1) / (u2 - u1)) - 1.0;
    // m_beams[this->active_element]->EvaluateSectionPoint(eta, point);
    // GetLog() << "Update tau=" << this->tau << " eta=" << eta << " dist=" << mdist << " elem n." <<
    // this->active_element << " on " << m_beams.size() << " pos=" << goodpoint << " outlet=" << absoutlet << "\n";

    m_nodes = m_beams[this->active_element]->GetNodes();

    // update variables pointers
    std::vector<ChVariables*> mvars;
    for (auto& i : this->m_nodes) {
        mvars.push_back(&(i->Variables()));
    }
    mvars.push_back(&(m_body->Variables()));

    // constraint1.SetVariables(mvars);
    constraint2.SetVariables(mvars);
    constraint3.SetVariables(mvars);
}

int ChLinkBeamIGAslider::Initialize(std::vector<std::shared_ptr<fea::ChElementBeamIGA>>& melements,
                                    std::shared_ptr<ChBodyFrame> body,
                                    ChVector<>* pos) {
    assert(body);

    m_beams = melements;
    m_body = body;

    UpdateNodes();

    ChVector<> pos_abs = pos ? *pos : m_body->GetPos();
    SetAttachPositionInAbsoluteCoords(pos_abs);

    return true;
}

void ChLinkBeamIGAslider::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(mytime, update_assets);

    // update class data
    this->UpdateNodes();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkBeamIGAslider::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // L(off_L + 0) = m_react.x();
    L(off_L + 0) = m_react.y();
    L(off_L + 1) = m_react.z();
}

void ChLinkBeamIGAslider::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // m_react.x() = L(off_L + 0);
    m_react.y() = L(off_L + 0);
    m_react.z() = L(off_L + 1);
}

void ChLinkBeamIGAslider::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                              ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                              const ChVectorDynamic<>& L,  // the L vector
                                              const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    // constraint1.MultiplyTandAdd(R, L(off_L + 0) * c);
    constraint2.MultiplyTandAdd(R, L(off_L + 0) * c);
    constraint3.MultiplyTandAdd(R, L(off_L + 1) * c);
}

void ChLinkBeamIGAslider::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                              ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                              const double c,            // a scaling factor
                                              bool do_clamp,             // apply clamping to c*C?
                                              double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector<> splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);
    ////GetLog() << "active_element = " << active_element << "\n";
    ////GetLog() << "tau = " << tau << "\n";
    ////GetLog() << "u1 = " << u1 << "\n";
    ////GetLog() << "u2 = " << u2 << "\n";
    ////GetLog() << "eta = " << eta << "\n";
    ////GetLog() << "point = " << splinepoint << "\n";
    ChVector<> res = Arw.transpose() * (splinepoint - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVector<> cres = res * c;
    ////GetLog() << "res = " << res << "\n";
    if (do_clamp) {
        cres.x() = ChMin(ChMax(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = ChMin(ChMax(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = ChMin(ChMax(cres.z(), -recovery_clamp), recovery_clamp);
    }
    ////GetLog() << "cres = " << cres << "\n";
    ////Qc(off_L + 0) += cres.x();
    ////Qc(off_L + 0) += cres.y();
    ////Qc(off_L + 1) += cres.z();
}

void ChLinkBeamIGAslider::IntToDescriptor(const unsigned int off_v,
                                          const ChStateDelta& v,
                                          const ChVectorDynamic<>& R,
                                          const unsigned int off_L,
                                          const ChVectorDynamic<>& L,
                                          const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    // constraint1.Set_l_i(L(off_L + 0));
    constraint2.Set_l_i(L(off_L + 0));
    constraint3.Set_l_i(L(off_L + 1));

    // constraint1.Set_b_i(Qc(off_L + 0));
    constraint2.Set_b_i(Qc(off_L + 0));
    constraint3.Set_b_i(Qc(off_L + 1));
}

void ChLinkBeamIGAslider::IntFromDescriptor(const unsigned int off_v,
                                            ChStateDelta& v,
                                            const unsigned int off_L,
                                            ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    // L(off_L + 0) = constraint1.Get_l_i();
    L(off_L + 0) = constraint2.Get_l_i();
    L(off_L + 1) = constraint3.Get_l_i();
}

// SOLVER INTERFACES

void ChLinkBeamIGAslider::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    // mdescriptor.InsertConstraint(&constraint1);
    mdescriptor.InsertConstraint(&constraint2);
    mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkBeamIGAslider::ConstraintsBiReset() {
    // constraint1.Set_b_i(0.);
    constraint2.Set_b_i(0.);
    constraint3.Set_b_i(0.);
}

void ChLinkBeamIGAslider::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_body)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector<> splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);

    ChVector<> res = Arw.transpose() * (splinepoint - m_body->TransformPointLocalToParent(m_csys.pos));

    // constraint1.Set_b_i(constraint1.Get_b_i() + factor * res.x());
    constraint2.Set_b_i(constraint2.Get_b_i() + factor * res.y());
    constraint3.Set_b_i(constraint3.Get_b_i() + factor * res.z());
}

void ChLinkBeamIGAslider::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkBeamIGAslider::ConstraintsLoadJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> ArwT = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChVector<> splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);

    ChStarMatrix33<> atilde(Aow.transpose() * (splinepoint - m_body->GetPos()));
    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    int nspan = this->order;

    ChVectorDynamic<> N((int)this->m_nodes.size());
    geometry::ChBasisToolsBspline::BasisEvaluate(this->order, nspan, this->tau,
                                                 this->m_beams[this->active_element]->GetKnotSequence(),
                                                 N);  ///< here return  in N

    ChMatrix33<> ArwT_N;
    for (int i = 0; i < this->m_nodes.size(); ++i) {
        ArwT_N = ArwT * N(i);
        constraint2.Get_Cq_N(i).segment(0, 3) = ArwT_N.row(1);
        constraint3.Get_Cq_N(i).segment(0, 3) = ArwT_N.row(2);
        ////GetLog() << "N" << i << "=" << N(i) << "\n";
    }

    constraint2.Get_Cq_N(this->m_nodes.size()).segment(0, 3) = Jxb.row(1);
    constraint3.Get_Cq_N(this->m_nodes.size()).segment(0, 3) = Jxb.row(2);

    constraint2.Get_Cq_N(this->m_nodes.size()).segment(3, 3) = Jrb.row(1);
    constraint3.Get_Cq_N(this->m_nodes.size()).segment(3, 3) = Jrb.row(2);
}

void ChLinkBeamIGAslider::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    // m_react.x() = constraint1.Get_l_i() * factor;
    m_react.y() = constraint2.Get_l_i() * factor;
    m_react.z() = constraint3.Get_l_i() * factor;
}

// FILE I/O

void ChLinkBeamIGAslider::ArchiveOut(ChArchiveOut& marchive) {
    //// TODO
}

void ChLinkBeamIGAslider::ArchiveIn(ChArchiveIn& marchive) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
