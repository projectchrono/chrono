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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

//#define BEAM_VERBOSE
#include "chrono/core/ChQuadrature.h"
#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"

namespace chrono {
namespace fea {
ChElementBeamTaperedTimoshenkoFPM::ChElementBeamTaperedTimoshenkoFPM() : guass_order(4) {
    q_refrotA = QUNIT;
    q_refrotB = QUNIT;
    q_element_abs_rot = QUNIT;
    q_element_ref_rot = QUNIT;
    force_symmetric_stiffness = false;
    disable_corotate = false;
    use_geometric_stiffness = true;
    use_Rc = true;
    use_Rs = true;

    nodes.resize(2);

    Km.setZero(this->GetNdofs(), this->GetNdofs());
    Kg.setZero(this->GetNdofs(), this->GetNdofs());
    M.setZero(this->GetNdofs(), this->GetNdofs());
    Rm.setZero(this->GetNdofs(), this->GetNdofs());
    Ri.setZero(this->GetNdofs(), this->GetNdofs());
    Ki.setZero(this->GetNdofs(), this->GetNdofs());

    T.setZero(this->GetNdofs(), this->GetNdofs());
    Rs.setIdentity(6, 6);
    Rc.setIdentity(6, 6);
}

void ChElementBeamTaperedTimoshenkoFPM::ShapeFunctionsTimoshenkoFPM(ShapeFunctionGroupFPM& NB, double eta) {
    // The shape functions have referenced two papers below, especially the first one:
    // Alexander R.Stäblein,and Morten H.Hansen.
    // "Timoshenko beam element with anisotropic cross-sectional properties."
    // ECCOMAS Congress 2016, VII European Congress on Computational Methods in Applied Sciences and Engineering.
    // Crete Island, Greece, 5 - 10 June 2016
    //
    // Taeseong Kim, Anders M.Hansen,and Kim Branner.
    // "Development of an anisotropic beam finite element for composite wind turbine blades in multibody system."
    // Renewable Energy 59(2013) : 172 - 183.

    // eta = 2 * x/L;
    // x = (-L/2, L/2),  hence eta = (-1, 1)
    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;
    double eta1 = (eta + 1) / 2.0;
    double eta2 = eta1 * eta1;
    double eta3 = eta2 * eta1;

    ChMatrixNM<double, 6, 14> Ax;
    Ax.setZero();
    ChMatrixNM<double, 6, 14> dAx;
    dAx.setZero();
    ChMatrixNM<double, 14, 14> Ex;
    Ex.setZero();
    ChMatrixNM<double, 14, 14> Ex_inv;
    Ex_inv.setZero();
    ChMatrixNM<double, 6, 12> Nx;
    Nx.setZero();
    ChMatrixNM<double, 6, 12> dNx;
    dNx.setZero();

    // The coefficient matrix of the displacements and rotations with respect to the shape function coefficient vector
    // c_v note: the shape function coefficient vector is as below: c_v = [c1 c2 c3 c4 c5 c6 c7 c8 c9 c10 c13 c14 c11
    // c12].';  // notice the order of c11 c12 c13 c14
    Ax.row(0) << L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ax.row(1) << 0, 0, LLL * eta3, LL * eta2, L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ax.row(2) << 0, 0, 0, 0, 0, 0, LLL * eta3, LL * eta2, L * eta1, 1, 0, 0, 0, 0;
    Ax.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, L * eta1, 1;
    Ax.row(4) << 0, 0, 0, 0, 0, 0, -3 * LL * eta2, -2 * L * eta1, -1, 0, 1, 0, 0, 0;
    Ax.row(5) << 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;

    // The derivative of Ax
    dAx.row(0) << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    dAx.row(1) << 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    dAx.row(2) << 0, 0, 0, 0, 0, 0, 3 * LL * eta2, 2 * L * eta1, 1, 0, 0, 0, 0, 0;
    dAx.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    dAx.row(4) << 0, 0, 0, 0, 0, 0, -6 * L * eta1, -2, 0, 0, 0, 0, 0, 0;
    dAx.row(5) << 0, 0, 6 * L * eta1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // A temporary transformation matrix
    ChMatrixNM<double, 14, 12> Ttemp;
    Ttemp.setZero();
    Ttemp.block<12, 12>(2, 0).setIdentity();

    // the cross-sectional material stiffness matrix Klaw along beam element may be variant
    // due to different Klaw at two ends of tapered-sectional beam
    ChMatrixNM<double, 6, 6> Klaw_point = this->tapered_section_fpm->GetKlawAtPoint(eta);
    // double k11 = Klaw_point(0, 0);
    double k12 = Klaw_point(0, 1);
    double k13 = Klaw_point(0, 2);
    // double k14 = Klaw_point(0, 3);
    // double k15 = Klaw_point(0, 4);
    // double k16 = Klaw_point(0, 5);
    double k22 = Klaw_point(1, 1);
    double k23 = Klaw_point(1, 2);
    double k24 = Klaw_point(1, 3);
    double k25 = Klaw_point(1, 4);
    double k26 = Klaw_point(1, 5);
    double k33 = Klaw_point(2, 2);
    double k34 = Klaw_point(2, 3);
    double k35 = Klaw_point(2, 4);
    double k36 = Klaw_point(2, 5);
    // double k44 = Klaw_point(3, 3);
    // double k45 = Klaw_point(3, 4);
    // double k46 = Klaw_point(3, 5);
    double k55 = Klaw_point(4, 4);
    double k56 = Klaw_point(4, 5);
    double k66 = Klaw_point(5, 5);

    // The coefficient matrix of the equilibrium and compatibility equations
    // with respect to the shape function coefficient vector c_v
    Ex.row(0) << -k13, 0, 6 * k56 - 3 * L * k36 - 3 * L * eta * k36, -2 * k36, 0, 0,
        3 * L * k35 - 6 * k55 + 3 * L * eta * k35, 2 * k35, 0, 0, -k33, -k23, -k34, 0;
    Ex.row(1) << k12, 0, 6 * k66 + 3 * L * k26 + 3 * L * eta * k26, 2 * k26, 0, 0,
        -6 * k56 - 3 * L * k25 - 3 * L * eta * k25, -2 * k25, 0, 0, k23, k22, k24, 0;
    Ex.row(2) << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(3) << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    Ex.row(5) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    Ex.row(6) << 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0;
    Ex.row(7) << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;
    Ex.row(8) << L, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(9) << 0, 0, LLL, LL, L, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    Ex.row(10) << 0, 0, 0, 0, 0, 0, LLL, LL, L, 1, 0, 0, 0, 0;
    Ex.row(11) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, L, 1;
    Ex.row(12) << 0, 0, 0, 0, 0, 0, -3 * LL, -2 * L, -1, 0, 1, 0, 0, 0;
    Ex.row(13) << 0, 0, 3 * LL, 2 * L, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0;

    // The inverse of Ex
    Ex_inv = Ex.inverse();

    // The shape function matrix at dimensionless position eta
    Nx = Ax * Ex_inv * Ttemp;
    // and its derivative
    dNx = dAx * Ex_inv * Ttemp;  // DO NOT add Ax * dEx_inv * Ttemp, see the first paper please.

    // A temporary matrix
    ChMatrixNM<double, 6, 6> TNtemp;
    TNtemp.setZero();
    TNtemp(1, 5) = -1.0;
    TNtemp(2, 4) = 1.0;

    // The strain displacement matrix at dimensionless position eta
    ChMatrixNM<double, 6, 12> Bx = dNx + TNtemp * Nx;

    // return result
    NB = std::make_tuple(Nx, Bx);
}

/// This class defines the calculations for the Guass integrand of
/// the cross-sectional stiffness/damping/mass matrices
class BeamTaperedTimoshenkoFPM : public ChIntegrable1D<ChMatrixNM<double, 12, 12>> {
  public:
    BeamTaperedTimoshenkoFPM(ChElementBeamTaperedTimoshenkoFPM* element, const int option)
        : m_element(element), m_choice_KiRiMi(option) {}
    ~BeamTaperedTimoshenkoFPM() {}

    // Which one matrix is evaluated? stiffness/damping/mass matrices
    // - 0: stiffness matrix
    // - 1: damping matrix
    // - 2: mass matix
    void SetChoiceKiRiMi(int mv) { m_choice_KiRiMi = mv; }
    int GetChoiceKiRiMi() { return m_choice_KiRiMi; }

  private:
    ChElementBeamTaperedTimoshenkoFPM* m_element;
    // 0: stiffness matrix
    // 1: damping matrix
    // 2: mass matix
    int m_choice_KiRiMi = 0;

    virtual void Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) override;
};

void BeamTaperedTimoshenkoFPM::Evaluate(ChMatrixNM<double, 12, 12>& result, const double x) {
    double eta = x;

    ChElementBeamTaperedTimoshenkoFPM::ShapeFunctionGroupFPM NxBx;
    m_element->ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // shape function matrix
    ChMatrixNM<double, 6, 12> Nx = std::get<0>(NxBx);
    // strain-displacement relation matrix
    ChMatrixNM<double, 6, 12> Bx = std::get<1>(NxBx);

    auto tapered_section_fpm = m_element->GetTaperedSection();
    ChMatrixNM<double, 6, 6> Klaw_point;
    ChMatrixNM<double, 6, 6> Rlaw_point;
    ChMatrixNM<double, 6, 6> Mlaw_point;

    switch (m_choice_KiRiMi) {
        case 0:
            Klaw_point = tapered_section_fpm->GetKlawAtPoint(eta);
            result = Bx.transpose() * Klaw_point * Bx;
            break;
        case 1:
            Rlaw_point = tapered_section_fpm->GetRlawAtPoint(eta);
            result = Bx.transpose() * Rlaw_point * Bx;  // modified Rayleigh damping model
            break;
        case 2:
            Mlaw_point = tapered_section_fpm->GetMlawAtPoint(eta);
            result = Nx.transpose() * Mlaw_point * Nx;
            break;
        default:
            std::cout << "Please input the correct option: 0,1,2" << std::endl;
            return;
    }
};

void ChElementBeamTaperedTimoshenkoFPM::ComputeStiffnessMatrix() {
    // Calculate the local element stiffness matrix via Guass integration
    this->Km.setZero();
    BeamTaperedTimoshenkoFPM myformula(this, 0);  // 0: stiffness matrix
    ChMatrixNM<double, 12, 12> TempStiffnessMatrix;
    TempStiffnessMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempStiffnessMatrix,  // result of integration will go there
                                                          myformula,            // formula to integrate
                                                          -1, 1,                // x limits
                                                          guass_order           // order of integration
    );
    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    TempStiffnessMatrix *= this->length / 2.0;  // need to multiple detJ
    this->Km = this->T.transpose() * TempStiffnessMatrix * this->T;
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeDampingMatrix() {
    // Calculate the local element damping matrix via Guass integration
    this->Rm.setZero();
    BeamTaperedTimoshenkoFPM myformula(this, 1);  // 1: damping matrix
    ChMatrixNM<double, 12, 12> TempDampingMatrix;
    TempDampingMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempDampingMatrix,  // result of integration will go there
                                                          myformula,          // formula to integrate
                                                          -1, 1,              // x limits
                                                          guass_order         // order of integration
    );
    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    TempDampingMatrix *= this->length / 2.0;  // need to multiple detJ
    this->Rm = this->T.transpose() * TempDampingMatrix * this->T;

    // The mass-proportional term
    double rdamping_alpha = this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.alpha;
    if (this->tapered_section->GetLumpedMassMatrixType()) {
        double node_multiplier_fact = 0.5 * this->length;
        Rm += rdamping_alpha * this->M * node_multiplier_fact;
    } else {
        Rm += rdamping_alpha * this->M;
    }
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeConsistentMassMatrix() {
    // Calculate the local element mass matrix via Guass integration
    this->M.setZero();
    BeamTaperedTimoshenkoFPM myformula(this, 2);  // 2: mass matrix
    ChMatrixNM<double, 12, 12> TempMassMatrix;
    TempMassMatrix.setZero();
    ChQuadrature::Integrate1D<ChMatrixNM<double, 12, 12>>(TempMassMatrix,  // result of integration will go there
                                                          myformula,       // formula to integrate
                                                          -1, 1,           // x limits
                                                          guass_order      // order of integration
    );
    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    TempMassMatrix *= this->length / 2.0;  // need to multiple detJ
    this->M = TempMassMatrix;
    // If the cross-sectional mass properties are given at the mass center,
    // then it should be transformed to the centerline firstly,
    // this is handled in the Class ChBeamSectionTimoshenkoAdvancedGenericFPM. NOT HERE.
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeMassMatrix() {
    // Compute local mass matrix of element
    // It could be lumped or consistent mass matrix, depends on SetLumpedMassMatrix(true/false)
    if (this->tapered_section_fpm->GetLumpedMassMatrixType()) {
        // If it is lumped mass matrix, you need to multiple 0.5 * length to obtain the final mass matrix
        // For consistent mass matrix, don't need to multiple anything.
        this->tapered_section_fpm->ComputeInertiaMatrix(this->M);
    } else {
        // If the consistent mass matrix is used, you need to compute the ave_sec_par firstly.
        this->tapered_section_fpm->ComputeAverageSectionParameters();
        ComputeConsistentMassMatrix();
    }
}

void ChElementBeamTaperedTimoshenkoFPM::SetupInitial(ChSystem* system) {
    assert(tapered_section_fpm);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
    this->mass = 0.5 * this->length * this->tapered_section_fpm->GetSectionA()->GetMassPerUnitLength() +
                 0.5 * this->length * this->tapered_section_fpm->GetSectionB()->GetMassPerUnitLength();

    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector<> myele = nodes[0]->GetX0().GetA().Get_A_Yaxis();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();

    // Compute transformation matrix
    ComputeTransformMatrix();

    // Compute local mass matrix:
    ComputeMassMatrix();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();

    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    ComputeGeometricStiffnessMatrix();

    // Compute local damping matrix:
    ComputeDampingMatrix();
}

void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionDisplacement(const double eta,
                                                                    ChVector<>& u_displ,
                                                                    ChVector<>& u_rotaz) {
    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);
    // No transformation for the displacement of two nodes,
    // so the section displacement is evaluated at the centerline of beam

    ShapeFunctionGroupFPM NxBx;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // the shape function matrix
    ChMatrixNM<double, 6, 12> Nx = std::get<0>(NxBx);

    // the displacements and rotations, as a vector
    ChVectorDynamic<> u_vector = Nx * displ;

    u_displ.x() = u_vector(0);
    u_displ.y() = u_vector(1);
    u_displ.z() = u_vector(2);
    u_rotaz.x() = u_vector(3);
    u_rotaz.y() = u_vector(4);
    u_rotaz.z() = u_vector(5);
}

void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionForceTorque(const double eta,
                                                                   ChVector<>& Fforce,
                                                                   ChVector<>& Mtorque) {
    assert(tapered_section_fpm);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    // transform the displacement of two nodes to elastic axis
    ChVectorDynamic<> displ_ec = this->T * displ;

    ShapeFunctionGroupFPM NxBx;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // the strain displacement matrix B:
    ChMatrixNM<double, 6, 12> Bx = std::get<1>(NxBx);

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek = Bx * displ_ec;

    // 6*6 fully populated constitutive matrix of the beam:
    ChMatrixNM<double, 6, 6> Klaw_d = this->tapered_section_fpm->GetKlawAtPoint(eta);

    ChMatrixDynamic<> Teta;
    ComputeTransformMatrixAtPoint(Teta, eta);

    // ..unrolled rotated constitutive matrix..
    ChMatrixNM<double, 6, 6> Klaw_r;
    Klaw_r.setZero();
    Klaw_r = Teta.transpose() * Klaw_d;

    // .. compute wrench = Klaw_r * sect_ek
    ChVectorN<double, 6> wrench = Klaw_r * sect_ek;
    Fforce = wrench.segment(0, 3);
    Mtorque = wrench.segment(3, 3);
}

void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionStrain(const double eta,
                                                              ChVector<>& StrainV_trans,
                                                              ChVector<>& StrainV_rot) {
    assert(tapered_section_fpm);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    // transform the displacement of two nodes to elastic axis
    ChVectorDynamic<> displ_ec = this->T * displ;

    ShapeFunctionGroupFPM NxBx;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    // the strain displacement matrix B:
    ChMatrixNM<double, 6, 12> Bx = std::get<1>(NxBx);

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek = Bx * displ_ec;

    StrainV_trans = sect_ek.segment(0, 3);
    StrainV_rot = sect_ek.segment(3, 3);
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeNF(const double U,
                                                  ChVectorDynamic<>& Qi,
                                                  double& detJ,
                                                  const ChVectorDynamic<>& F,
                                                  ChVectorDynamic<>* state_x,
                                                  ChVectorDynamic<>* state_w) {
    ShapeFunctionGroupFPM NxBx;
    // the shape function matrix
    ChMatrixNM<double, 6, 12> Nx;

    double eta = -1;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    Nx = std::get<0>(NxBx);
    Qi.head(6) = Nx.transpose() * F;

    eta = 1;
    ShapeFunctionsTimoshenkoFPM(NxBx, eta);
    Nx = std::get<0>(NxBx);
    Qi.tail(6) = Nx.transpose() * F;

    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    detJ = this->GetRestLength() / 2.0;
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeNF(const double U,
                                                  const double V,
                                                  const double W,
                                                  ChVectorDynamic<>& Qi,
                                                  double& detJ,
                                                  const ChVectorDynamic<>& F,
                                                  ChVectorDynamic<>* state_x,
                                                  ChVectorDynamic<>* state_w) {
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}

}  // end namespace fea
}  // end namespace chrono
