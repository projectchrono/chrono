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

#include "chrono/fea/ChLinkBeamIGAFrame.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkBeamIGAFrame)

ChLinkBeamIGAFrame::ChLinkBeamIGAFrame() : m_react(VNULL), m_csys(CSYSNORM) {}

ChLinkBeamIGAFrame::ChLinkBeamIGAFrame(const ChLinkBeamIGAFrame& other) : ChLinkBase(other) {
    m_csys = other.m_csys;
    m_react = other.m_react;
}

ChFrame<> ChLinkBeamIGAFrame::GetFrameBodyAbs() const {
    return ChFrame<>(m_csys >> *m_body);
}

void ChLinkBeamIGAFrame::UpdateNodes() {
    m_nodes.clear();

    this->order = (int)m_beams[0]->GetNodes().size() - 1;

    // fetch the element and the tau
    this->tau = 0;
    this->active_element = 0;
    int nnodes = (int)this->m_nodes.size();
    ChVector3d point = VNULL;
    ChVector3d goodpoint = VNULL;
    ChVectorDynamic<> N(nnodes);
    int nsamples = 6;  //// TODO  search via NR
    double mdist = 1e30;
    ChVector3d absoutlet = this->m_body->TransformPointLocalToParent(this->m_csys.pos);

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
    //// TEST
    // double u1 = m_beams[this->active_element]->GetU1();
    // double u2 = m_beams[this->active_element]->GetU2();
    // double eta = (2.0*(this->tau - u1) / (u2 - u1)) - 1.0;
    // m_beams[this->active_element]->EvaluateSectionPoint(eta, point);
    // std::cout << "Update tau=" << this->tau << " eta=" << eta << " dist=" << mdist << " elem n." <<
    // this->active_element << " on " << m_beams.size() << " pos=" << goodpoint << " outlet=" << absoutlet << std::endl;

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

int ChLinkBeamIGAFrame::Initialize(std::vector<std::shared_ptr<fea::ChElementBeamIGA>>& melements,
                                   std::shared_ptr<ChBodyFrame> body,
                                   ChVector3d* pos) {
    assert(body);

    m_beams = melements;
    m_body = body;

    UpdateNodes();

    ChVector3d pos_abs = pos ? *pos : m_body->GetPos();
    SetAttachPositionInAbsoluteCoords(pos_abs);

    return true;
}

void ChLinkBeamIGAFrame::Update(double time, bool update_assets) {
    // Inherit time changes of parent class
    ChPhysicsItem::Update(time, update_assets);

    // update class data
    this->UpdateNodes();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkBeamIGAFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // L(off_L + 0) = m_react.x();
    L(off_L + 0) = m_react.y();
    L(off_L + 1) = m_react.z();
}

void ChLinkBeamIGAFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // m_react.x() = L(off_L + 0);
    m_react.y() = L(off_L + 0);
    m_react.z() = L(off_L + 1);
}

void ChLinkBeamIGAFrame::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                             ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                             const ChVectorDynamic<>& L,  // the L vector
                                             const double c               // a scaling factor
) {
    if (!IsActive())
        return;

    // constraint1.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint2.AddJacobianTransposedTimesScalarInto(R, L(off_L + 0) * c);
    constraint3.AddJacobianTransposedTimesScalarInto(R, L(off_L + 1) * c);
}

void ChLinkBeamIGAFrame::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                             ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                             const double c,            // a scaling factor
                                             bool do_clamp,             // apply clamping to c*C?
                                             double recovery_clamp      // value for min/max clamping of c*C
) {
    if (!IsActive())
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());

    ChVector3d splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);
    ////std::cout << "active_element = " << active_element << std::endl;
    ////std::cout << "tau = " << tau << std::endl;
    ////std::cout << "u1 = " << u1 << std::endl;
    ////std::cout << "u2 = " << u2 << std::endl;
    ////std::cout << "eta = " << eta << std::endl;
    ////std::cout << "point = " << splinepoint << std::endl;
    ChVector3d res = Arw.transpose() * (splinepoint - m_body->TransformPointLocalToParent(m_csys.pos));
    ChVector3d cres = res * c;
    ////std::cout << "res = " << res << std::endl;
    if (do_clamp) {
        cres.x() = std::min(std::max(cres.x(), -recovery_clamp), recovery_clamp);
        cres.y() = std::min(std::max(cres.y(), -recovery_clamp), recovery_clamp);
        cres.z() = std::min(std::max(cres.z(), -recovery_clamp), recovery_clamp);
    }
    ////std::cout << "cres = " << cres << std::endl;
    ////Qc(off_L + 0) += cres.x();
    ////Qc(off_L + 0) += cres.y();
    ////Qc(off_L + 1) += cres.z();
}

void ChLinkBeamIGAFrame::IntToDescriptor(const unsigned int off_v,
                                         const ChStateDelta& v,
                                         const ChVectorDynamic<>& R,
                                         const unsigned int off_L,
                                         const ChVectorDynamic<>& L,
                                         const ChVectorDynamic<>& Qc) {
    if (!IsActive())
        return;

    // constraint1.SetLagrangeMultiplier(L(off_L + 0));
    constraint2.SetLagrangeMultiplier(L(off_L + 0));
    constraint3.SetLagrangeMultiplier(L(off_L + 1));

    // constraint1.SetRightHandSide(Qc(off_L + 0));
    constraint2.SetRightHandSide(Qc(off_L + 0));
    constraint3.SetRightHandSide(Qc(off_L + 1));
}

void ChLinkBeamIGAFrame::IntFromDescriptor(const unsigned int off_v,
                                           ChStateDelta& v,
                                           const unsigned int off_L,
                                           ChVectorDynamic<>& L) {
    if (!IsActive())
        return;

    // L(off_L + 0) = constraint1.GetLagrangeMultiplier();
    L(off_L + 0) = constraint2.GetLagrangeMultiplier();
    L(off_L + 1) = constraint3.GetLagrangeMultiplier();
}

// SOLVER INTERFACES

void ChLinkBeamIGAFrame::InjectConstraints(ChSystemDescriptor& descriptor) {
    // if (!IsActive())
    //	return;

    // descriptor.InsertConstraint(&constraint1);
    descriptor.InsertConstraint(&constraint2);
    descriptor.InsertConstraint(&constraint3);
}

void ChLinkBeamIGAFrame::ConstraintsBiReset() {
    // constraint1.SetRightHandSide(0.);
    constraint2.SetRightHandSide(0.);
    constraint3.SetRightHandSide(0.);
}

void ChLinkBeamIGAFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    if (!m_body)
        return;

    ChMatrix33<> Arw(m_csys.rot >> m_body->GetRot());
    ChVector3d splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);

    ChVector3d res = Arw.transpose() * (splinepoint - m_body->TransformPointLocalToParent(m_csys.pos));

    // constraint1.SetRightHandSide(constraint1.GetRightHandSide() + factor * res.x());
    constraint2.SetRightHandSide(constraint2.GetRightHandSide() + factor * res.y());
    constraint3.SetRightHandSide(constraint3.GetRightHandSide() + factor * res.z());
}

void ChLinkBeamIGAFrame::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChLinkBeamIGAFrame::LoadConstraintJacobians() {
    // compute jacobians
    ChMatrix33<> Aro(m_csys.rot);
    ChMatrix33<> Aow(m_body->GetRot());
    ChMatrix33<> Arw = Aow * Aro;

    ChMatrix33<> ArwT = Arw.transpose();

    ChMatrix33<> Jxb = -Arw.transpose();

    ChVector3d splinepoint;
    double u1 = m_beams[this->active_element]->GetU1();
    double u2 = m_beams[this->active_element]->GetU2();
    double eta = (2.0 * (this->tau - u1) / (u2 - u1)) - 1.0;
    m_beams[this->active_element]->EvaluateSectionPoint(eta, splinepoint);

    ChStarMatrix33<> atilde(Aow.transpose() * (splinepoint - m_body->GetPos()));
    ChMatrix33<> Jrb = Aro.transpose() * atilde;

    int nspan = this->order;

    ChVectorDynamic<> N((int)this->m_nodes.size());
    ChBasisToolsBSpline::BasisEvaluate(this->order, nspan, this->tau,
                                       this->m_beams[this->active_element]->GetKnotSequence(),
                                       N);  ///< here return  in N

    ChMatrix33<> ArwT_N;
    for (int i = 0; i < this->m_nodes.size(); ++i) {
        ArwT_N = ArwT * N(i);
        constraint2.Get_Cq_N(i).segment(0, 3) = ArwT_N.row(1);
        constraint3.Get_Cq_N(i).segment(0, 3) = ArwT_N.row(2);
        ////std::cout << "N" << i << "=" << N(i) << std::endl;
    }

    constraint2.Get_Cq_N(this->m_nodes.size()).segment(0, 3) = Jxb.row(1);
    constraint3.Get_Cq_N(this->m_nodes.size()).segment(0, 3) = Jxb.row(2);

    constraint2.Get_Cq_N(this->m_nodes.size()).segment(3, 3) = Jrb.row(1);
    constraint3.Get_Cq_N(this->m_nodes.size()).segment(3, 3) = Jrb.row(2);
}

void ChLinkBeamIGAFrame::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    // m_react.x() = constraint1.GetLagrangeMultiplier() * factor;
    m_react.y() = constraint2.GetLagrangeMultiplier() * factor;
    m_react.z() = constraint3.GetLagrangeMultiplier() * factor;
}

// FILE I/O

void ChLinkBeamIGAFrame::ArchiveOut(ChArchiveOut& archive_out) {
    //// TODO
}

void ChLinkBeamIGAFrame::ArchiveIn(ChArchiveIn& archive_in) {
    //// TODO
}

}  // end namespace fea
}  // end namespace chrono
