// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Template for a multibody deformable tire.
//
// =============================================================================

//// TODO:
//// - implement CalculateInertiaProperties
//// - add false coloring support

#include <algorithm>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChMBTire.h"

// =============================================================================

#define FORCE_SEQUENTIAL
////#define FORCE_OMP_ALL_AT_ONCE
////#define FORCE_OMP_CRITICAL_SECTION

// =============================================================================

namespace chrono {
namespace vehicle {

ChMBTire::ChMBTire(const std::string& name) : ChDeformableTire(name) {
    m_model = chrono_types::make_shared<MBTireModel>();
    m_model->m_tire = this;
    m_model->m_stiff = false;
    m_model->m_force_jac = false;
}

void ChMBTire::SetTireGeometry(const std::vector<double>& ring_radii,
                               const std::vector<double>& ring_offsets,
                               int num_divs,
                               double rim_radius) {
    assert(ring_radii.size() > 1);
    assert(ring_radii.size() == ring_offsets.size());

    m_model->m_radii = ring_radii;
    m_model->m_offsets = ring_offsets;

    m_model->m_num_rings = (int)ring_radii.size();
    m_model->m_num_divs = num_divs;

    m_model->m_rim_radius = rim_radius;
}

void ChMBTire::SetTireMass(double mass) {
    m_mass = mass;
}

void ChMBTire::SetMeshSpringCoefficients(double kC, double cC, double kT, double cT) {
    m_model->m_kC = kC;
    m_model->m_cC = cC;
    m_model->m_kT = kT;
    m_model->m_cT = cT;
}

void ChMBTire::SetBendingSpringCoefficients(double kB, double cB) {
    m_model->m_kB = kB;
    m_model->m_cB = cB;
}

void ChMBTire::SetRadialSpringCoefficients(double kR, double cR) {
    m_model->m_kR = kR;
    m_model->m_cR = cR;
}

void ChMBTire::IsStiff(bool val) {
    m_model->m_stiff = val;
}

void ChMBTire::ForceJacobianCalculation(bool val) {
    m_model->m_force_jac = val;
}

void ChMBTire::SetTireContactMaterial(const ChContactMaterialData& mat_data) {
    m_contact_mat_data = mat_data;
}

double ChMBTire::GetRadius() const {
    return *std::max_element(m_model->m_radii.begin(), m_model->m_radii.end());
}

double ChMBTire::GetRimRadius() const {
    return m_model->m_rim_radius;
}

double ChMBTire::GetWidth() const {
    return m_model->m_offsets.back() - m_model->m_offsets.front();
}

void ChMBTire::CreateContactMaterial() {
    m_contact_mat =
        std::static_pointer_cast<ChContactMaterialSMC>(m_contact_mat_data.CreateMaterial(ChContactMethod::SMC));
}

void ChMBTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    ChSystem* system = wheel->GetSpindle()->GetSystem();
    assert(system);

    // Add the underlying MB tire model (as a PhysicsItem) to the system *before* its construction.
    // This way, all its components will have an associated system during construction.
    system->Add(m_model);

    // Set internal tire pressure (if enabled)
    if (m_pressure_enabled && m_pressure <= 0)
        m_pressure = GetDefaultPressure();

    // Set contact material properties (if enabled)
    if (m_contact_enabled)
        CreateContactMaterial();

    // Construct the underlying tire model, attached to the wheel spindle body
    m_model->m_wheel = wheel->GetSpindle();
    m_model->Construct(m_contact_surface_type, m_contact_surface_dim, m_collision_family);
}

void ChMBTire::Synchronize(double time, const ChTerrain& terrain) {
    //// TODO
}

void ChMBTire::Advance(double step) {
    //// TODO
}

TerrainForce ChMBTire::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce terrain_force;
    terrain_force.force = m_model->m_wheel_force;
    terrain_force.moment = m_model->m_wheel_torque;
    terrain_force.point = m_wheel->GetPos();
    return terrain_force;
}

TerrainForce ChMBTire::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ChMBTire::ReportTireForceLocal not implemented." << std::endl;
    throw std::runtime_error("ChMBTire::ReportTireForceLocal not implemented.");
}

void ChMBTire::AddVisualizationAssets(VisualizationType vis) {
    m_model->AddVisualizationAssets(vis);
}

void ChMBTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_model);
}

void ChMBTire::InitializeInertiaProperties() {
    if (!IsInitialized())
        return;

    ChVector3d com;
    m_model->CalculateInertiaProperties(com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

void ChMBTire::UpdateInertiaProperties() {
    if (!IsInitialized())
        return;

    ChVector3d com;
    m_model->CalculateInertiaProperties(com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

std::vector<std::shared_ptr<fea::ChNodeFEAxyz>>& ChMBTire::GetGridNodes() const {
    return m_model->m_nodes;
}

std::vector<std::shared_ptr<fea::ChNodeFEAxyz>>& ChMBTire::GetRimNodes() const {
    return m_model->m_rim_nodes;
}

// =============================================================================

MBTireModel::MBTireModel()
    : m_num_pressure_loads(0),
      m_num_grid_lin_springs(0),
      m_num_edge_lin_springs(0),
      m_num_grid_rot_springs(0),
      m_num_edge_rot_springs(0),
      m_num_loads(0) {}

int MBTireModel::NodeIndex(int ir, int id) const {
    // If ring index out-of-bounds, return -1
    if (ir < 0 || ir >= m_num_rings)
        return -1;

    // Make sure to use a positive value of the division index
    while (id < 0)
        id += m_num_divs;

    // Wrap around circumference if needed
    return ir * m_num_divs + (id % m_num_divs);
}

int MBTireModel::RimNodeIndex(int ir, int id) const {
    // If ring index out-of-bounds, return -1
    if (ir != 0 && ir != m_num_rings - 1)
        return -1;

    // Make sure to use a positive value of the division index
    while (id < 0)
        id += m_num_divs;

    // Wrap around circumference if needed
    int ir_local = (ir == 0) ? 0 : 1;
    return ir_local * m_num_divs + (id % m_num_divs);
}

// Calculate total tire area as sum of side areas of truncated cones
double MBTireModel::CalculateArea() const {
    double area = 0;

    for (int ir = 0; ir < m_num_rings - 1; ir++) {
        double l2 = (m_radii[ir + 1] - m_radii[ir]) * (m_radii[ir + 1] - m_radii[ir]) +
                    (m_offsets[ir + 1] - m_offsets[ir]) * (m_offsets[ir + 1] - m_offsets[ir]);
        area += CH_PI * (m_radii[ir + 1] + m_radii[ir]) * std::sqrt(l2);
    }

    return area;
}

void MBTireModel::CalculateNormal(int ir, int id, ChVector3d& normal, double& area) {
    // Get locations of previous and next nodes in the two directions
    const auto& posS = m_nodes[NodeIndex(ir, id - 1)]->GetPos();
    const auto& posN = m_nodes[NodeIndex(ir, id + 1)]->GetPos();
    const auto& posE = (ir == 0) ? m_rim_nodes[RimNodeIndex(0, id)]->GetPos()  //
                                 : m_nodes[NodeIndex(ir - 1, id)]->GetPos();
    const auto& posW = (ir == m_num_rings - 1) ? m_rim_nodes[RimNodeIndex(m_num_rings - 1, id)]->GetPos()  //
                                               : m_nodes[NodeIndex(ir + 1, id)]->GetPos();

    // Note: the normal could be approximated better, by averaging the normals of the 4 incident triangular faces
    ChVector3d dir = Vcross(posN - posS, posE - posW);
    area = 0.25 * dir.Length();
    normal = dir / dir.Length();  // normal vector
}

// -----------------------------------------------------------------------------
// Implementation of MBTire load classes (Forces and Jacobians)
//
// Jacobian matrices are linear combination of the form:
//    Kfactor * [K] + Rfactor * [R]
// where
// - [K] is the partial derivative wrt position-level states ("stiffness")
// - [R] is the partial derivative wrt velocity-level states ("damping")

// Constant threshold for checking zero length vectors
static constexpr double zero_length = 1e-6;

// Constant threshold for checking zero angles
static constexpr double zero_angle = 1e-3;

// Perturbation for FD Jacobian approximation
static constexpr double FD_step = 1e-3;

void MBTireModel::NodePressure::Initialize(bool stiff) {
    if (stiff) {
        std::vector<ChVariables*> vars;
        vars.push_back(&node->Variables());
        vars.push_back(&wheel->Variables());
        KRM.SetVariables(vars);
    }
}

void MBTireModel::NodePressure::CalculateForce() {
    const auto& pos = node->GetPos();     // node position
    const auto& w_pos = wheel->GetPos();  // wheel body position

    //// RADU TODO ---> a better approximation of the normal?

    auto n = pos - w_pos;     // vector from wheel center to node
    double l = n.Length();    //
    assert(l > zero_length);  //
    n /= l;                   // unit normal approximation at node
    force = p_times_a * n;    // nodal force due to inner pressure
}

// Calculate a 3x3 block used in assembling Jacobians for SpringAir
ChMatrix33<> MBTireModel::NodePressure::CalculateJacobianBlock(double Kfactor, double Rfactor) {
    const auto& pos = node->GetPos();
    ChVector3d w_pos = wheel->GetPos();  // wheel body position

    auto n = pos - w_pos;     // vector from wheel center to node
    double l = n.Length();    //
    assert(l > zero_length);  //
    n /= l;                   // unit normal approximation at node

    ChVectorN<double, 3> n_vec = n.eigen();
    ChMatrix33<> N = n_vec * n_vec.transpose();
    ChMatrix33<> A = Kfactor * p_times_a / l * (ChMatrix33<>::Identity() - N);

    return A;
}

void MBTireModel::NodePressure::CalculateJacobian(double Kfactor, double Rfactor) {
    ChMatrixRef J = KRM.GetMatrix();
    if (Kfactor == 0 && Rfactor == 0) {
        J.setZero();
        return;
    }

    ////std::cout << "Node pressure " << inode;
    ////std::cout << "  J size: " << J.rows() << "x" << J.cols() << std::endl;

    auto A = CalculateJacobianBlock(Kfactor, Rfactor);

    J.setZero();
    J.block(0, 0, 3, 3) = A;   // block for F and node
    J.block(0, 3, 3, 3) = -A;  // block for F and wheel translational states

    /*
    CalculateJacobianFD(Kfactor, Rfactor);
    std::cout << "NodePressure " << inode << std::endl;
    std::cout << "-------\n";
    std::cout << J << std::endl;
    std::cout << "-------\n";
    std::cout << J_fd << std::endl;
    std::cout << "-------\n";
    std::cout << "err_2 = " << (J_fd - J).norm() / J.norm() << "\t";
    std::cout << "err_1 = " << (J_fd - J).lpNorm<1>() / J.lpNorm<1>() << "\t";
    std::cout << "err_inf = " << (J_fd - J).lpNorm<Eigen::Infinity>() / J.lpNorm<Eigen::Infinity>() << "\n";
    std::cout << "=======" << std::endl;
    */
}

void MBTireModel::NodePressure::CalculateJacobianFD(double Kfactor, double Rfactor) {
    ChMatrixNM<double, 9, 9> K;

    K.setZero();

    ChVector3d w_pos = wheel->GetPos();
    ChVector3d pos = node->GetPos();

    CalculateForce();
    auto force_0 = force;

    // node states (columms 1,2,3)
    for (int i = 0; i < 3; i++) {
        pos[i] += FD_step;
        node->SetPos(pos);
        CalculateForce();
        K.col(i).segment(0, 3) = (force.eigen() - force_0.eigen()) / FD_step;
        pos[i] -= FD_step;
        node->SetPos(pos);
    }

    // wheel states (columms 3,4,5)
    for (int i = 0; i < 3; i++) {
        w_pos[i] += FD_step;
        wheel->SetPos(w_pos);
        CalculateForce();
        K.col(3 + i).segment(0, 3) = (force.eigen() - force_0.eigen()) / FD_step;
        w_pos[i] -= FD_step;
        wheel->SetPos(w_pos);
    }

    J_fd = Kfactor * K;
}

void MBTireModel::Spring2::CalculateForce() {
    const auto& pos1 = node1->GetPos();
    const auto& pos2 = node2->GetPos();
    const auto& vel1 = node1->GetPosDt();
    const auto& vel2 = node2->GetPosDt();

    auto d = pos2 - pos1;
    double l = d.Length();
    assert(l > zero_length);
    d /= l;
    double ld = Vdot(vel2 - vel1, d);

    double f = k * (l - l0) + c * ld;

    ChVector3d vforce = f * d;
    force1 = +vforce;
    force2 = -vforce;
}

void MBTireModel::Spring2::Initialize() {
    const auto& pos1 = node1->GetPos();
    const auto& pos2 = node2->GetPos();

    l0 = (pos2 - pos1).Length();
}

void MBTireModel::GridSpring2::Initialize(bool stiff) {
    Spring2::Initialize();
    if (stiff) {
        std::vector<ChVariables*> vars;
        vars.push_back(&node1->Variables());
        vars.push_back(&node2->Variables());
        KRM.SetVariables(vars);
    }
}

void MBTireModel::EdgeSpring2::Initialize(bool stiff) {
    Spring2::Initialize();
    if (stiff) {
        std::vector<ChVariables*> vars;
        vars.push_back(&wheel->Variables());
        vars.push_back(&node2->Variables());
        KRM.SetVariables(vars);
    }
}

// Calculate a 3x3 block used in assembling Jacobians for Spring2
ChMatrix33<> MBTireModel::Spring2::CalculateJacobianBlock(double Kfactor, double Rfactor) {
    const auto& pos1 = node1->GetPos();
    const auto& pos2 = node2->GetPos();
    const auto& vel1 = node1->GetPosDt();
    const auto& vel2 = node2->GetPosDt();

    auto d = pos2 - pos1;
    double l = d.Length();
    assert(l > zero_length);
    d /= l;
    double ld = Vdot(vel2 - vel1, d);
    auto dd = (vel2 - vel1 - ld * d) / l;

    ChVectorN<double, 3> d_vec = d.eigen();
    ChVectorN<double, 3> dd_vec = dd.eigen();

    ChMatrix33<> D = d_vec * d_vec.transpose();
    ChMatrix33<> DD = d_vec * dd_vec.transpose();

    double f = k * (l - l0) + c * ld;

    ChMatrix33<> A = (Kfactor * k + Rfactor * c - Kfactor * f / l) * D +  //
                     (Kfactor * f / l) * ChMatrix33<>::Identity() +       //
                     Kfactor * c * DD;

    return A;
}

// For a linear spring connecting two grid nodes, the Jacobian is a 6x6 matrix:
//   d[f1;f2]/d[n1;n2]
// where f1, f2 are the nodal forces and n1, n2 are the states of the 2 nodes.
void MBTireModel::GridSpring2::CalculateJacobian(double Kfactor, double Rfactor) {
    ChMatrixRef J = KRM.GetMatrix();
    if (Kfactor == 0 && Rfactor == 0) {
        J.setZero();
        return;
    }

    ////std::cout << "Grid spring2 " << inode1 << "-" << inode2;
    ////std::cout << "  J size: " << J.rows() << "x" << J.cols() << std::endl;

    auto A = CalculateJacobianBlock(Kfactor, Rfactor);

    J.block(0, 0, 3, 3) = -A;  // block for F1 and node1
    J.block(0, 3, 3, 3) = A;   // block for F1 and node2
    J.block(3, 0, 3, 3) = A;   // block for F2 and node1
    J.block(3, 3, 3, 3) = -A;  // block for F2 and node2

    /*
    CalculateJacobianFD(Kfactor, Rfactor);
    std::cout << "Grid spring2 " << inode1 << "-" << inode2 << std::endl;
    std::cout << "-------\n";
    std::cout << J << std::endl;
    std::cout << "-------\n";
    std::cout << J_fd << std::endl;
    std::cout << "-------\n";
    std::cout << "err_2 = " << (J_fd - J).norm() / J.norm() << "\t";
    std::cout << "err_1 = " << (J_fd - J).lpNorm<1>() / J.lpNorm<1>() << "\t";
    std::cout << "err_inf = " << (J_fd - J).lpNorm<Eigen::Infinity>() / J.lpNorm<Eigen::Infinity>() << "\n";
    std::cout << "=======" << std::endl;
    */
}

void MBTireModel::GridSpring2::CalculateJacobianFD(double Kfactor, double Rfactor) {
    ChMatrixNM<double, 6, 6> K;
    ChMatrixNM<double, 6, 6> R;

    K.setZero();
    R.setZero();

    ChVector3d pos1 = node1->GetPos();
    ChVector3d pos2 = node2->GetPos();
    ChVector3d vel1 = node1->GetPosDt();
    ChVector3d vel2 = node2->GetPosDt();

    CalculateForce();
    auto force1_0 = force1;
    auto force2_0 = force2;

    // node1 states (columns 0,1,2)
    for (int i = 0; i < 3; i++) {
        pos1[i] += FD_step;
        node1->SetPos(pos1);
        CalculateForce();
        K.col(i).segment(0, 3) = (force1.eigen() - force1_0.eigen()) / FD_step;
        K.col(i).segment(3, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        pos1[i] -= FD_step;
        node1->SetPos(pos1);

        vel1[i] += FD_step;
        node1->SetPosDt(vel1);
        CalculateForce();
        R.col(i).segment(0, 3) = (force1.eigen() - force1_0.eigen()) / FD_step;
        R.col(i).segment(3, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        vel1[i] -= FD_step;
        node1->SetPosDt(vel1);
    }

    // node2 states (columms 3,4,5)
    for (int i = 0; i < 3; i++) {
        pos2[i] += FD_step;
        node2->SetPos(pos2);
        CalculateForce();
        K.col(3 + i).segment(0, 3) = (force1.eigen() - force1_0.eigen()) / FD_step;
        K.col(3 + i).segment(3, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        pos2[i] -= FD_step;
        node2->SetPos(pos2);

        vel2[i] += FD_step;
        node2->SetPosDt(vel2);
        CalculateForce();
        R.col(3 + i).segment(0, 3) = (force1.eigen() - force1_0.eigen()) / FD_step;
        R.col(3 + i).segment(3, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        vel2[i] -= FD_step;
        node2->SetPosDt(vel2);
    }

    J_fd = Kfactor * K + Rfactor * R;
}

void MBTireModel::EdgeSpring2::CalculateForce() {
    Spring2::CalculateForce();

    // Convert the nodal force at node1 (fixed on the wheel) into a wrench expressed in global frame
    // Calculate force_wheel and torque_wheel, with the torque expressed in body frame
    auto wrench = wheel->AppliedForceParentToWrenchParent(force1, node1->GetPos());
    force_wheel = wrench.force;
    torque_wheel = wheel->TransformDirectionParentToLocal(wrench.torque);
}

// For a linear spring connecting a rim node and a grid node, the Jacobian is a 9x9 matrix.
// Note the order of generalized forces and states is:
//    wheel state (6)
//    node2 state (3)

ChMatrix33<> MBTireModel::EdgeSpring2::JacobianRotatedVector() {
    const auto& pos1 = node1->GetPos();
    ChVector3d p = wheel->GetPos();  // wheel body position
    ChStarMatrix33<> Jac_Af_m(p - pos1);
    return Jac_Af_m;
}

void MBTireModel::EdgeSpring2::CalculateJacobian(double Kfactor, double Rfactor) {
    ChMatrixRef J = KRM.GetMatrix();
    if (Kfactor == 0 && Rfactor == 0) {
        J.setZero();
        return;
    }

    ////std::cout << "Edge spring2 " << inode1 << "-" << inode2;
    ////std::cout << "  J size: " << J.rows() << "x" << J.cols() << std::endl;

    auto A = CalculateJacobianBlock(Kfactor, Rfactor);

    auto Jac_Af_m = JacobianRotatedVector();
    ChVector3d rp = wheel->TransformPointParentToLocal(node1->GetPos());
    ChStarMatrix33<> skew_rp(rp);

    J.block(0, 0, 3, 3) = -A;
    J.block(0, 3, 3, 3) = -A * Jac_Af_m;
    J.block(0, 6, 3, 3) = A;

    J.block(3, 0, 3, 9) = skew_rp * J.block(0, 0, 3, 9);  // block of torque
    J.block(6, 0, 3, 9) = -J.block(0, 0, 3, 9);           // block for F2 and node1

    /*
    CalculateJacobianFD(Kfactor, Rfactor);
    std::cout << "Edge spring2 " << inode1 << "-" << inode2 << std::endl;
    std::cout << "-------\n";
    std::cout << J << std::endl;
    std::cout << "-------\n";
    std::cout << J_fd << std::endl;
    std::cout << "-------\n";
    std::cout << "err_2   = " << (J_fd - J).norm() / J.norm() << "\n";
    std::cout << "err_1   = " << (J_fd - J).lpNorm<1>() / J.lpNorm<1>() << "\n";
    std::cout << "err_inf = " << (J_fd - J).lpNorm<Eigen::Infinity>() / J.lpNorm<Eigen::Infinity>() << "\n";
    std::cout << "=======" << std::endl;
    */
}

void MBTireModel::EdgeSpring2::CalculateJacobianFD(double Kfactor, double Rfactor) {
    ChMatrixNM<double, 9, 9> K;
    ChMatrixNM<double, 9, 9> R;

    K.setZero();
    R.setZero();

    ChVector3d posW = wheel->GetPos();
    ChQuaterniond rotW = wheel->GetRot();
    ChVector3d velW = wheel->GetPosDt();
    ChVector3d omgW = wheel->GetAngVelParent();

    ChVector3d pos2 = node2->GetPos();
    ChVector3d vel2 = node2->GetPosDt();

    CalculateForce();
    auto force_wheel_0 = force_wheel;
    auto torque_wheel_0 = torque_wheel;
    auto force2_0 = force2;

    // wheel states (columns 0...5 in K)
    // we must "bake in" the velocity transform: perturb at velocity level and let the body increment state.
    // for position states (first 3 wheel states), this is the same as directly perturbing posW components.
    ChState wheel_state(7, nullptr);
    wheel_state.segment(0, 3) = posW.eigen();
    wheel_state.segment(3, 4) = rotW.eigen();
    ChState wheel_state_perturbed(7, nullptr);
    ChStateDelta state_delta(6, nullptr);
    state_delta.setZero(6, nullptr);

    for (int i = 0; i < 6; i++) {
        state_delta(i) += FD_step;
        wheel->LoadableStateIncrement(0, wheel_state_perturbed, wheel_state, 0, state_delta);
        wheel->SetPos(ChVector3d(wheel_state_perturbed.segment(0, 3)));
        wheel->SetRot(ChQuaterniond(wheel_state_perturbed.segment(3, 4)));
        node1->SetPos(wheel->TransformPointLocalToParent(local_pos));

        CalculateForce();
        K.col(0 + i).segment(0, 3) = (force_wheel.eigen() - force_wheel_0.eigen()) / FD_step;
        K.col(0 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        K.col(0 + i).segment(6, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;

        state_delta(i) -= FD_step;
        wheel->LoadableStateIncrement(0, wheel_state_perturbed, wheel_state, 0, state_delta);
        wheel->SetPos(ChVector3d(wheel_state_perturbed.segment(0, 3)));
        wheel->SetRot(ChQuaterniond(wheel_state_perturbed.segment(3, 4)));
        node1->SetPos(wheel->TransformPointLocalToParent(local_pos));
    }

    // wheel state derivatives (columns 0,1,2 in R)
    for (int i = 0; i < 3; i++) {
        velW[i] += FD_step;
        wheel->SetPosDt(velW);
        node1->SetPosDt(wheel->PointSpeedLocalToParent(local_pos));

        CalculateForce();
        R.col(0 + i).segment(0, 3) = (force_wheel.eigen() - force_wheel_0.eigen()) / FD_step;
        R.col(0 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        R.col(0 + i).segment(6, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;

        velW[i] -= FD_step;
        wheel->SetPosDt(velW);
        node1->SetPosDt(wheel->PointSpeedLocalToParent(local_pos));
    }

    // wheel state derivatives (columns 3,4,5 in R)
    for (int i = 0; i < 3; i++) {
        omgW[i] += FD_step;
        wheel->SetAngVelLocal(omgW);
        node1->SetPosDt(wheel->PointSpeedLocalToParent(local_pos));

        CalculateForce();
        R.col(3 + i).segment(0, 3) = (force_wheel.eigen() - force_wheel_0.eigen()) / FD_step;
        R.col(3 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        R.col(3 + i).segment(6, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;

        omgW[i] -= FD_step;
        wheel->SetAngVelLocal(omgW);
        node1->SetPosDt(wheel->PointSpeedLocalToParent(local_pos));
    }

    // node2 states and state derivatives (columms 6,7,8 in K and R)
    for (int i = 0; i < 3; i++) {
        pos2[i] += FD_step;
        node2->SetPos(pos2);
        CalculateForce();
        K.col(6 + i).segment(0, 3) = (force_wheel.eigen() - force_wheel_0.eigen()) / FD_step;
        K.col(6 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        K.col(6 + i).segment(6, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        pos2[i] -= FD_step;
        node2->SetPos(pos2);

        vel2[i] += FD_step;
        node2->SetPosDt(vel2);
        CalculateForce();
        R.col(6 + i).segment(0, 3) = (force_wheel.eigen() - force_wheel_0.eigen()) / FD_step;
        R.col(6 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        R.col(6 + i).segment(6, 3) = (force2.eigen() - force2_0.eigen()) / FD_step;
        vel2[i] -= FD_step;
        node2->SetPosDt(vel2);
    }

    J_fd = Kfactor * K + Rfactor * R;
}

void MBTireModel::Spring3::Initialize() {
    const auto& pos_p = node_p->GetPos();
    const auto& pos_c = node_c->GetPos();
    const auto& pos_n = node_n->GetPos();

    auto dir_p = (pos_c - pos_p).GetNormalized();
    auto dir_n = (pos_n - pos_c).GetNormalized();

    double cosA = Vdot(dir_p, dir_n);
    a0 = std::acos(cosA);
}

void MBTireModel::GridSpring3::Initialize(bool stiff) {
    Spring3::Initialize();
    if (stiff) {
        std::vector<ChVariables*> vars;
        vars.push_back(&node_p->Variables());
        vars.push_back(&node_c->Variables());
        vars.push_back(&node_n->Variables());
        KRM.SetVariables(vars);
    }
}

void MBTireModel::EdgeSpring3::Initialize(bool stiff) {
    Spring3::Initialize();
    if (stiff) {
        std::vector<ChVariables*> vars;
        vars.push_back(&wheel->Variables());
        vars.push_back(&node_c->Variables());
        vars.push_back(&node_n->Variables());
        KRM.SetVariables(vars);
    }
}

void MBTireModel::Spring3::CalculateForce() {
    const auto& pos_p = node_p->GetPos();
    const auto& pos_c = node_c->GetPos();
    const auto& pos_n = node_n->GetPos();
    ////const auto& vel_p = node_p->GetPos_dt();
    ////const auto& vel_c = node_c->GetPos_dt();
    ////const auto& vel_n = node_n->GetPos_dt();

    auto d_p = pos_c - pos_p;
    auto d_n = pos_n - pos_c;
    double l_p = d_p.Length();
    double l_n = d_n.Length();
    assert(l_p > zero_length);
    assert(l_n > zero_length);
    d_p /= l_p;
    d_n /= l_n;

    double cosA = Vdot(d_p, d_n);
    double a = std::acos(cosA);

    if (std::abs(a - a0) < zero_angle) {
        force_p = VNULL;
        force_c = VNULL;
        force_n = VNULL;
        return;
    }

    auto cross = Vcross(d_p, d_n);
    double length_cross = cross.Length();
    if (length_cross > zero_length) {
        cross /= length_cross;
    } else {  // colinear points
        cross = wheel->TransformDirectionLocalToParent(t0);
    }

    auto F_p = k * ((a - a0) / l_p) * Vcross(cross, d_p);
    auto F_n = k * ((a - a0) / l_n) * Vcross(cross, d_n);

    force_p = -F_p;
    force_c = F_p + F_n;
    force_n = -F_n;
}

ChMatrixNM<double, 6, 9> MBTireModel::Spring3::CalculateJacobianBlockJ1(double Kfactor) {
    const auto& pos_p = node_p->GetPos();
    const auto& pos_c = node_c->GetPos();
    const auto& pos_n = node_n->GetPos();

    auto dp = pos_c - pos_p;
    auto dn = pos_n - pos_c;

    ChStarMatrix33<> skew_dp(dp);
    ChStarMatrix33<> skew_dn(dn);

    double lp = dp.Length();
    double ln = dn.Length();

    auto np = dp / lp;
    auto nn = dn / ln;

    auto t = Vcross(np, nn);
    double lt = t.Length();
    if (lt < zero_length) {
        t = wheel->TransformDirectionLocalToParent(t0);
        lt = 1;
    }

    double cosA = Vdot(np, nn);
    double a = std::acos(cosA);
    double sinA = std::sin(a);

    // Calculate D_pp, D_nn, D_pn
    ChMatrix33<> D_pp = np.eigen() * np.eigen().transpose();
    ChMatrix33<> D_nn = nn.eigen() * nn.eigen().transpose();

    // multiplier
    double scale = k / (lt * lp * ln);
    scale *= Kfactor;
    double scale2 = (sinA * cosA - (a - a0)) / (lt * lt);

    auto Bp = 1 / lp * Vcross(t, np);
    auto Bn = 1 / ln * Vcross(t, nn);

    // common vector
    ChVectorN<double, 6> B;
    B.block(0, 0, 3, 1) = scale * Bp.eigen();
    B.block(3, 0, 3, 1) = scale * Bn.eigen();

    ChMatrix33<> dA1 = skew_dn * (ChMatrix33<>::Identity() - D_pp);
    ChMatrix33<> dA3 = skew_dp * (ChMatrix33<>::Identity() - D_nn);

    ChVectorN<double, 3> dA_dalp_13 =
        scale2 * dA1.transpose() * t.eigen() + sinA * (ChMatrix33<>::Identity() - D_pp).transpose() * dn.eigen();
    ChVectorN<double, 3> dA_dalp_79 =
        scale2 * dA3.transpose() * t.eigen() - sinA * (ChMatrix33<>::Identity() - D_nn).transpose() * dp.eigen();
    ChVectorN<double, 3> dA_dalp_46 = -dA_dalp_13 - dA_dalp_79;

    ChVectorN<double, 9> dA_dalp;
    dA_dalp.block<3, 1>(0, 0) = dA_dalp_13;
    dA_dalp.block<3, 1>(3, 0) = dA_dalp_46;
    dA_dalp.block<3, 1>(6, 0) = dA_dalp_79;
    ChMatrixNM<double, 6, 9> J1;
    J1 = B * dA_dalp.transpose();

    return J1;
}

ChMatrixNM<double, 6, 9> MBTireModel::Spring3::CalculateJacobianBlockJ2(double Kfactor) {
    const auto& pos_p = node_p->GetPos();
    const auto& pos_c = node_c->GetPos();
    const auto& pos_n = node_n->GetPos();

    auto dp = pos_c - pos_p;
    auto dn = pos_n - pos_c;

    double lp = dp.Length();
    double ln = dn.Length();
    assert(lp > zero_length);
    assert(ln > zero_length);
    auto np = dp / lp;
    auto nn = dn / ln;

    auto t = Vcross(np, nn);
    double lt = t.Length();
    if (lt < zero_length) {
        t = wheel->TransformDirectionLocalToParent(t0);
        lt = 1;
    }

    double cosA = Vdot(np, nn);
    double a = std::acos(cosA);

    // Calculate D_pp, D_nn, D_pn
    ChMatrix33<> D_pp = np.eigen() * np.eigen().transpose();
    ChMatrix33<> D_nn = nn.eigen() * nn.eigen().transpose();
    ChMatrix33<> D_pn = np.eigen() * nn.eigen().transpose();

    // multiplier
    double scale = k * (a - a0) / (lt * lp * ln);
    scale *= Kfactor;

    ChMatrix33<> dBn_dalp_1 = (ChMatrix33<>::Identity() - D_nn) * (ChMatrix33<>::Identity() - D_pp);
    ChMatrix33<> dBn_dalp_3 = (lp / ln) * (D_pn + D_pn.transpose() + cosA * (ChMatrix33<>::Identity() - 3 * D_nn));
    ChMatrix33<> dBn_dalp_2 = -dBn_dalp_1 - dBn_dalp_3;

    ChMatrix33<> dBp_dalp_1 = (ln / lp) * (D_pn + D_pn.transpose() + cosA * (ChMatrix33<>::Identity() - 3 * D_pp));
    ChMatrix33<> dBp_dalp_3 = dBn_dalp_1.transpose();
    ChMatrix33<> dBp_dalp_2 = -dBp_dalp_1 - dBp_dalp_3;

    ChMatrixNM<double, 6, 9> J2;
    J2.block(0, 0, 3, 3) = scale * dBp_dalp_1;
    J2.block(0, 3, 3, 3) = scale * dBp_dalp_2;
    J2.block(0, 6, 3, 3) = scale * dBp_dalp_3;

    J2.block(3, 0, 3, 3) = scale * dBn_dalp_1;
    J2.block(3, 3, 3, 3) = scale * dBn_dalp_2;
    J2.block(3, 6, 3, 3) = scale * dBn_dalp_3;

    return J2;
}

// For a rotational spring connecting three grid nodes, the Jacobian is a 9x9 matrix:
//   d[f_p;f_c;f_n]/d[n_p;n_c;n_n]
// where f_p, f_c, and f_n are the nodal forces and n_p, n_c, and n_n are the states of the 3 nodes.

// Note: Rfactor not used here (no damping)
void MBTireModel::GridSpring3::CalculateJacobian(double Kfactor, double Rfactor) {
    ChMatrixRef J = KRM.GetMatrix();
    if (Kfactor == 0) {
        J.setZero();
        return;
    }

    ////std::cout << "Grid spring3 " << inode_p << "-" << inode_c << "-" << inode_n;
    ////std::cout << "  J size: " << J.rows() << "x" << J.cols() << std::endl;

    auto J1 = CalculateJacobianBlockJ1(Kfactor);
    auto J2 = CalculateJacobianBlockJ2(Kfactor);

    ChMatrixNM<double, 3, 9> Jp = J1.topRows(3) + J2.topRows(3);
    ChMatrixNM<double, 3, 9> Jn = J1.bottomRows(3) + J2.bottomRows(3);

    // assemble Jacobian matrix
    //    force_p = -F_p;
    //    force_c = +F_p + F_n;
    //    force_n = -F_n;
    J.block(0, 0, 3, 9) = -Jp;
    J.block(3, 0, 3, 9) = Jp + Jn;
    J.block(6, 0, 3, 9) = -Jn;

    /*
    CalculateJacobianFD(Kfactor);
    std::cout << "Grid spring3 " << inode_p << "-" << inode_c << "-" << inode_n << std::endl;
    std::cout << "-------\n";
    std::cout << J << std::endl;
    std::cout << "-------\n";
    std::cout << J_fd << std::endl;
    std::cout << "-------\n";
    std::cout << "err_2   = " << (J_fd - J).norm() / J.norm() << "\n";
    std::cout << "err_1   = " << (J_fd - J).lpNorm<1>() / J.lpNorm<1>() << "\n";
    std::cout << "err_inf = " << (J_fd - J).lpNorm<Eigen::Infinity>() / J.lpNorm<Eigen::Infinity>() << "\n";
    std::cout << "=======" << std::endl;
    */
}

// Note: Rfactor not used here (no damping)
void MBTireModel::GridSpring3::CalculateJacobianFD(double Kfactor, double Rfactor) {
    ChMatrixNM<double, 9, 9> K;

    K.setZero();

    ChVector3d pos_p = node_p->GetPos();
    ChVector3d pos_c = node_c->GetPos();
    ChVector3d pos_n = node_n->GetPos();

    CalculateForce();
    auto force_p_0 = force_p;
    auto force_c_0 = force_c;
    auto force_n_0 = force_n;

    // node_p states (columns 0,1,2)
    for (int i = 0; i < 3; i++) {
        pos_p[i] += FD_step;
        node_p->SetPos(pos_p);
        CalculateForce();
        K.col(i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(i).segment(3, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(i).segment(6, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;
        pos_p[i] -= FD_step;
        node_p->SetPos(pos_p);
    }

    // node_c states (columns 3,4,5)
    for (int i = 0; i < 3; i++) {
        pos_c[i] += FD_step;
        node_c->SetPos(pos_c);
        CalculateForce();
        K.col(3 + i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(3 + i).segment(3, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(3 + i).segment(6, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;
        pos_c[i] -= FD_step;
        node_c->SetPos(pos_c);
    }

    // node_n states (columns 6,7,8)
    for (int i = 0; i < 3; i++) {
        pos_n[i] += FD_step;
        node_n->SetPos(pos_n);
        CalculateForce();
        K.col(6 + i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(6 + i).segment(3, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(6 + i).segment(6, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;
        pos_n[i] -= FD_step;
        node_n->SetPos(pos_n);
    }

    J_fd = Kfactor * K;
}

void MBTireModel::EdgeSpring3::CalculateForce() {
    Spring3::CalculateForce();

    // Convert the nodal force at node_p (fixed on the wheel) into a wrench expressed in global frame
    // Calculate force_wheel and torque_wheel, with the torque expressed in body frame
    auto wrench = wheel->AppliedForceParentToWrenchParent(force_p, node_p->GetPos());
    force_wheel = wrench.force;
    torque_wheel = wheel->TransformDirectionParentToLocal(wrench.torque);
}

// For a rotational spring connecting a rim node and 2 grid nodes, the Jacobian is a 12x12 matrix.
// Note the order of generalized forces and states is:
//    wheel state (6)
//    node_c state (3)
//    node_n state (3)

ChMatrix33<> MBTireModel::EdgeSpring3::JacobianRotatedVector() {
    const auto& pos_p = node_p->GetPos();
    ChVector3d p = wheel->GetPos();  // wheel body position
    ChStarMatrix33<> Jac_Af_m(p - pos_p);
    return Jac_Af_m;
}

// Note: Rfactor not used here (no damping)
void MBTireModel::EdgeSpring3::CalculateJacobian(double Kfactor, double Rfactor) {
    ChMatrixRef J = KRM.GetMatrix();
    if (Kfactor == 0) {
        J.setZero();
        return;
    }

    ////std::cout << "Edge spring3 " << inode_p << "-" << inode_c << "-" << inode_n;
    ////std::cout << "  J size: " << J.rows() << "x" << J.cols() << std::endl;

    auto J1 = CalculateJacobianBlockJ1(Kfactor);
    auto J2 = CalculateJacobianBlockJ2(Kfactor);

    auto Jac_Af_m = JacobianRotatedVector();
    ChVector3d rp = wheel->TransformPointParentToLocal(node_p->GetPos());
    ChStarMatrix33<> skew_rp(rp);

    ChMatrixNM<double, 9, 12> TransitionMatrix;
    TransitionMatrix.setZero();
    TransitionMatrix.block(0, 0, 3, 3) = ChMatrix33<>::Identity();
    TransitionMatrix.block(0, 3, 3, 3) = Jac_Af_m;
    TransitionMatrix.block(3, 6, 3, 3) = ChMatrix33<>::Identity();
    TransitionMatrix.block(6, 9, 3, 3) = ChMatrix33<>::Identity();

    ChMatrixNM<double, 6, 12> J1_new = J1 * TransitionMatrix;
    ChMatrixNM<double, 6, 12> J2_new = J2 * TransitionMatrix;
    ChMatrixNM<double, 3, 12> Jp = J1_new.topRows(3) + J2_new.topRows(3);
    ChMatrixNM<double, 3, 12> Jn = J1_new.bottomRows(3) + J2_new.bottomRows(3);

    J.block(0, 0, 3, 12) = -Jp;
    J.block(3, 0, 3, 12) = -skew_rp * Jp;  // block of torque
    J.block(6, 0, 3, 12) = Jp + Jn;
    J.block(9, 0, 3, 12) = -Jn;

    /*
    CalculateJacobianFD(Kfactor);
    std::cout << "Edge spring3 " << inode_p << "-" << inode_c << "-" << inode_n << std::endl;
    std::cout << "-------\n";
    std::cout << J << std::endl;
    std::cout << "-------\n";
    std::cout << J_fd << std::endl;
    std::cout << "-------\n";
    std::cout << "err_2   = " << (J_fd - J).norm() / J.norm() << "\n";
    std::cout << "err_1   = " << (J_fd - J).lpNorm<1>() / J.lpNorm<1>() << "\n";
    std::cout << "err_inf = " << (J_fd - J).lpNorm<Eigen::Infinity>() / J.lpNorm<Eigen::Infinity>() << "\n";
    std::cout << "=======" << std::endl;
    */
}

// Note: Rfactor not used here (no damping)
void MBTireModel::EdgeSpring3::CalculateJacobianFD(double Kfactor, double Rfactor) {
    ChMatrixNM<double, 12, 12> K;  // +3 due to the torque

    K.setZero();

    ChVector3d posW = wheel->GetPos();
    ChQuaterniond rotW = wheel->GetRot();

    ChVector3d pos_c = node_c->GetPos();
    ChVector3d pos_n = node_n->GetPos();

    CalculateForce();
    auto force_p_0 = force_p;
    auto force_wheel_0 = force_wheel;
    auto torque_wheel_0 = torque_wheel;
    auto force_c_0 = force_c;
    auto force_n_0 = force_n;

    ChState wheel_state(7, nullptr);
    wheel_state.segment(0, 3) = posW.eigen();
    wheel_state.segment(3, 4) = rotW.eigen();
    ChState wheel_state_perturbed(7, nullptr);
    ChStateDelta state_delta(6, nullptr);
    state_delta.setZero(6, nullptr);

    // node_p states
    for (int i = 0; i < 6; i++) {
        state_delta(i) += FD_step;
        wheel->LoadableStateIncrement(0, wheel_state_perturbed, wheel_state, 0, state_delta);
        wheel->SetPos(ChVector3d(wheel_state_perturbed.segment(0, 3)));
        wheel->SetRot(ChQuaterniond(wheel_state_perturbed.segment(3, 4)));
        node_p->SetPos(wheel->TransformPointLocalToParent(local_pos));

        CalculateForce();
        K.col(i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        K.col(i).segment(6, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(i).segment(9, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;

        state_delta(i) -= FD_step;
        wheel->LoadableStateIncrement(0, wheel_state_perturbed, wheel_state, 0, state_delta);
        wheel->SetPos(ChVector3d(wheel_state_perturbed.segment(0, 3)));
        wheel->SetRot(ChQuaterniond(wheel_state_perturbed.segment(3, 4)));
        node_p->SetPos(wheel->TransformPointLocalToParent(local_pos));
    }

    // node_c states
    for (int i = 0; i < 3; i++) {
        pos_c[i] += FD_step;
        node_c->SetPos(pos_c);
        CalculateForce();
        K.col(6 + i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(6 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        K.col(6 + i).segment(6, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(6 + i).segment(9, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;
        pos_c[i] -= FD_step;
        node_c->SetPos(pos_c);
    }

    // node_n states
    for (int i = 0; i < 3; i++) {
        pos_n[i] += FD_step;
        node_n->SetPos(pos_n);
        CalculateForce();
        K.col(9 + i).segment(0, 3) = (force_p.eigen() - force_p_0.eigen()) / FD_step;
        K.col(9 + i).segment(3, 3) = (torque_wheel.eigen() - torque_wheel_0.eigen()) / FD_step;
        K.col(9 + i).segment(6, 3) = (force_c.eigen() - force_c_0.eigen()) / FD_step;
        K.col(9 + i).segment(9, 3) = (force_n.eigen() - force_n_0.eigen()) / FD_step;
        pos_n[i] -= FD_step;
        node_n->SetPos(pos_n);
    }

    J_fd = Kfactor * K;
}

// -----------------------------------------------------------------------------

void MBTireModel::Construct(ChTire::ContactSurfaceType surface_type, double surface_dim, int collision_family) {
    m_num_rim_nodes = 2 * m_num_divs;
    m_num_nodes = m_num_rings * m_num_divs;
    m_num_faces = 2 * (m_num_rings - 1) * m_num_divs;

    m_node_mass = m_tire->GetMass() / m_num_nodes;

    // Create the visualization shape and get accessors to the underling trimesh
    m_trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    auto trimesh = m_trimesh_shape->GetMesh();
    auto& vertices = trimesh->GetCoordsVertices();
    auto& normals = trimesh->GetCoordsNormals();
    auto& idx_vertices = trimesh->GetIndicesVertexes();
    auto& idx_normals = trimesh->GetIndicesNormals();
    ////auto& uv_coords = trimesh->GetCoordsUV();
    auto& colors = trimesh->GetCoordsColors();

    // ------------ Nodes

    // Create the FEA nodes (with positions expressed in the global frame) and load the mesh vertices.
    m_nodes.resize(m_num_nodes);
    vertices.resize(m_num_nodes);
    double dphi = CH_2PI / m_num_divs;
    int k = 0;
    for (int ir = 0; ir < m_num_rings; ir++) {
        double y = m_offsets[ir];
        double r = m_radii[ir];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;
            double x = r * std::cos(phi);
            double z = r * std::sin(phi);
            vertices[k] = m_wheel->TransformPointLocalToParent(ChVector3d(x, y, z));
            m_nodes[k] = chrono_types::make_shared<fea::ChNodeFEAxyz>(vertices[k]);
            m_nodes[k]->SetMass(m_node_mass);
            m_nodes[k]->m_TotalMass = m_node_mass;
            k++;
        }
    }

    // Create the FEA nodes attached to the rim
    m_rim_nodes.resize(m_num_rim_nodes);
    k = 0;
    {
        double y = m_offsets[0];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;
            double x = m_rim_radius * std::cos(phi);
            double z = m_rim_radius * std::sin(phi);
            auto loc = m_wheel->TransformPointLocalToParent(ChVector3d(x, y, z));
            m_rim_nodes[k] = chrono_types::make_shared<fea::ChNodeFEAxyz>(loc);
            m_rim_nodes[k]->SetMass(m_node_mass);
            m_rim_nodes[k]->m_TotalMass = m_node_mass;
            k++;
        }
    }
    {
        double y = m_offsets[m_num_rings - 1];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;
            double x = m_rim_radius * std::cos(phi);
            double z = m_rim_radius * std::sin(phi);
            auto loc = m_wheel->TransformPointLocalToParent(ChVector3d(x, y, z));
            m_rim_nodes[k] = chrono_types::make_shared<fea::ChNodeFEAxyz>(loc);
            m_rim_nodes[k]->SetMass(m_node_mass);
            m_rim_nodes[k]->m_TotalMass = m_node_mass;
            k++;
        }
    }

    // ------------ Pressure loads

    double tire_area = CalculateArea();
    double node_area = tire_area / (m_num_rings * m_num_divs);
    double p_times_a = m_tire->GetPressure() * node_area;

    for (int id = 0; id < m_num_divs; id++) {
        for (int ir = 0; ir < m_num_rings; ir++) {
            int inode = NodeIndex(ir, id);
            auto load = chrono_types::make_shared<NodePressure>();
            load->inode = inode;
            load->node = m_nodes[inode].get();
            load->wheel = m_wheel.get();
            load->p_times_a = p_times_a;

            load->Initialize(m_stiff || m_force_jac);

            m_pressure_loads.push_back(load);
            m_num_pressure_loads++;

            m_loads.push_back(load);
            m_num_loads++;
        }
    }

    // ------------ Springs

    // Create circumferential linear springs (node-node)
    for (int ir = 0; ir < m_num_rings; ir++) {
        for (int id = 0; id < m_num_divs; id++) {
            int inode1 = NodeIndex(ir, id);
            int inode2 = NodeIndex(ir, id + 1);

            auto spring = chrono_types::make_shared<GridSpring2>();
            spring->inode1 = inode1;
            spring->inode2 = inode2;
            spring->node1 = m_nodes[inode1].get();
            spring->node2 = m_nodes[inode2].get();
            spring->wheel = m_wheel.get();
            spring->k = m_kC;
            spring->c = m_cC;

            spring->Initialize(m_stiff || m_force_jac);

            m_grid_lin_springs.push_back(spring);
            m_num_grid_lin_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }
    }

    // Create transversal linear springs (node-node and node-rim)
    for (int id = 0; id < m_num_divs; id++) {
        // radial springs connected to the rim at first ring
        {
            int inode1 = RimNodeIndex(0, id);
            int inode2 = NodeIndex(0, id);

            auto spring = chrono_types::make_shared<EdgeSpring2>();
            spring->inode1 = inode1;
            spring->inode2 = inode2;
            spring->node1 = m_rim_nodes[inode1].get();
            spring->node2 = m_nodes[inode2].get();
            spring->wheel = m_wheel.get();
            spring->k = m_kR;
            spring->c = m_cR;

            spring->local_pos = m_wheel->TransformPointParentToLocal(m_rim_nodes[inode1]->GetPos());

            spring->Initialize(m_stiff || m_force_jac);

            m_edge_lin_springs.push_back(spring);
            m_num_edge_lin_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }

        // node-node springs
        for (int ir = 0; ir < m_num_rings - 1; ir++) {
            int inode1 = NodeIndex(ir, id);
            int inode2 = NodeIndex(ir + 1, id);

            auto spring = chrono_types::make_shared<GridSpring2>();
            spring->inode1 = inode1;
            spring->inode2 = inode2;
            spring->node1 = m_nodes[inode1].get();
            spring->node2 = m_nodes[inode2].get();
            spring->wheel = m_wheel.get();
            spring->k = m_kT;
            spring->c = m_cT;

            spring->Initialize(m_stiff || m_force_jac);

            m_grid_lin_springs.push_back(spring);
            m_num_grid_lin_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }

        // radial springs connected to the rim at last ring
        {
            int inode1 = RimNodeIndex(m_num_rings - 1, id);
            int inode2 = NodeIndex(m_num_rings - 1, id);

            auto spring = chrono_types::make_shared<EdgeSpring2>();
            spring->inode1 = inode1;
            spring->inode2 = inode2;
            spring->node1 = m_rim_nodes[inode1].get();
            spring->node2 = m_nodes[inode2].get();
            spring->wheel = m_wheel.get();
            spring->k = m_kR;
            spring->c = m_cR;

            spring->local_pos = m_wheel->TransformPointParentToLocal(m_rim_nodes[inode1]->GetPos());

            spring->Initialize(m_stiff || m_force_jac);

            m_edge_lin_springs.push_back(spring);
            m_num_edge_lin_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }
    }

    // Create circumferential rotational springs (node-node)
    for (int ir = 0; ir < m_num_rings; ir++) {
        for (int id = 0; id < m_num_divs; id++) {
            int inode_p = NodeIndex(ir, id - 1);
            int inode_c = NodeIndex(ir, id);
            int inode_n = NodeIndex(ir, id + 1);

            auto spring = chrono_types::make_shared<GridSpring3>();
            spring->inode_p = inode_p;
            spring->inode_c = inode_c;
            spring->inode_n = inode_n;
            spring->node_p = m_nodes[inode_p].get();
            spring->node_c = m_nodes[inode_c].get();
            spring->node_n = m_nodes[inode_n].get();
            spring->wheel = m_wheel.get();
            spring->t0 = ChVector3d(0, 1, 0);
            spring->k = m_kB;
            spring->c = m_cB;

            spring->Initialize(m_stiff || m_force_jac);

            m_grid_rot_springs.push_back(spring);
            m_num_grid_rot_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }
    }

    // Create transversal rotational springs (node-node and node-rim)
    for (int id = 0; id < m_num_divs; id++) {
        double phi = id * dphi;
        ChVector3d t0(-std::sin(phi), 0, std::cos(phi));

        // torsional springs connected to the rim at first ring
        {
            int inode_p = RimNodeIndex(0, id);
            int inode_c = NodeIndex(0, id);
            int inode_n = NodeIndex(1, id);

            auto spring = chrono_types::make_shared<EdgeSpring3>();
            spring->inode_p = inode_p;
            spring->inode_c = inode_c;
            spring->inode_n = inode_n;
            spring->node_p = m_rim_nodes[inode_p].get();
            spring->node_c = m_nodes[inode_c].get();
            spring->node_n = m_nodes[inode_n].get();
            spring->wheel = m_wheel.get();
            spring->t0 = t0;
            spring->k = m_kB;
            spring->c = m_cB;

            spring->local_pos = m_wheel->TransformPointParentToLocal(m_rim_nodes[inode_p]->GetPos());

            spring->Initialize(m_stiff || m_force_jac);

            m_edge_rot_springs.push_back(spring);
            m_num_edge_rot_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }

        // node-node torsional springs
        for (int ir = 1; ir < m_num_rings - 1; ir++) {
            int inode_p = NodeIndex(ir - 1, id);
            int inode_c = NodeIndex(ir, id);
            int inode_n = NodeIndex(ir + 1, id);

            auto spring = chrono_types::make_shared<GridSpring3>();
            spring->inode_p = inode_p;
            spring->inode_c = inode_c;
            spring->inode_n = inode_n;
            spring->node_p = m_nodes[inode_p].get();
            spring->node_c = m_nodes[inode_c].get();
            spring->node_n = m_nodes[inode_n].get();
            spring->wheel = m_wheel.get();
            spring->t0 = t0;
            spring->k = m_kB;
            spring->c = m_cB;

            spring->Initialize(m_stiff || m_force_jac);

            m_grid_rot_springs.push_back(spring);
            m_num_grid_rot_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }

        // torsional springs connected to the rim at last ring
        {
            int inode_p = RimNodeIndex(m_num_rings - 1, id);
            int inode_c = NodeIndex(m_num_rings - 1, id);
            int inode_n = NodeIndex(m_num_rings - 2, id);

            auto spring = chrono_types::make_shared<EdgeSpring3>();
            spring->inode_p = inode_p;
            spring->inode_c = inode_c;
            spring->inode_n = inode_n;
            spring->node_p = m_rim_nodes[inode_p].get();
            spring->node_c = m_nodes[inode_c].get();
            spring->node_n = m_nodes[inode_n].get();
            spring->wheel = m_wheel.get();
            spring->t0 = -t0;
            spring->k = m_kB;
            spring->c = m_cB;

            spring->local_pos = m_wheel->TransformPointParentToLocal(m_rim_nodes[inode_p]->GetPos());

            spring->Initialize(m_stiff || m_force_jac);

            m_edge_rot_springs.push_back(spring);
            m_num_edge_rot_springs++;

            m_loads.push_back(spring);
            m_num_loads++;
        }
    }

    assert(m_num_pressure_loads == m_num_rings * m_num_divs);
    assert(m_num_grid_lin_springs == (int)m_grid_lin_springs.size());
    assert(m_num_grid_rot_springs == (int)m_grid_rot_springs.size());
    assert(m_num_edge_lin_springs == (int)m_edge_lin_springs.size());
    assert(m_num_edge_rot_springs == (int)m_edge_rot_springs.size());

    // ------------ Collision and visualization meshes

    // Auxiliary face information (for possible collision mesh)
    struct FaceAuxData {
        ChVector3i nbr_node;   // neighbor (opposite) vertex/node for each face vertex
        ChVector3b owns_node;  // vertex/node owned by the face?
        ChVector3b owns_edge;  // edge owned by the face?
    };
    std::vector<FaceAuxData> auxdata(m_num_faces);

    // Create the mesh faces and define auxiliary data (for possible collision mesh)
    idx_vertices.resize(m_num_faces);
    idx_normals.resize(m_num_faces);
    k = 0;
    for (int ir = 0; ir < m_num_rings - 1; ir++) {
        bool last = ir == m_num_rings - 2;
        for (int id = 0; id < m_num_divs; id++) {
            int v1 = NodeIndex(ir, id);
            int v2 = NodeIndex(ir + 1, id);
            int v3 = NodeIndex(ir + 1, id + 1);
            int v4 = NodeIndex(ir, id + 1);
            idx_vertices[k] = {v1, v2, v3};
            idx_normals[k] = {v1, v2, v3};
            auxdata[k].nbr_node = {NodeIndex(ir + 2, id + 1), v4, NodeIndex(ir, id - 1)};
            auxdata[k].owns_node = {true, last, false};
            auxdata[k].owns_edge = {true, last, true};
            k++;
            idx_vertices[k] = {v1, v3, v4};
            idx_normals[k] = {v1, v3, v4};
            auxdata[k].nbr_node = {NodeIndex(ir + 1, id + 2), NodeIndex(ir - 1, id), v2};
            auxdata[k].owns_node = {false, false, false};
            auxdata[k].owns_edge = {false, false, true};
            k++;
        }
    }

    // Create the contact surface of the specified type and initialize it using the underlying model
    if (m_tire->IsContactEnabled()) {
        auto contact_mat = m_tire->GetContactMaterial();

        switch (surface_type) {
            case ChDeformableTire::ContactSurfaceType::NODE_CLOUD: {
                auto contact_surf = chrono_types::make_shared<fea::ChContactSurfaceNodeCloud>(contact_mat);
                contact_surf->SetPhysicsItem(this);
                contact_surf->DisableSelfCollisions(collision_family);
                for (const auto& node : m_nodes)
                    contact_surf->AddNode(node, surface_dim);
                m_contact_surf = contact_surf;
                break;
            }
            case ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH: {
                auto contact_surf = chrono_types::make_shared<fea::ChContactSurfaceMesh>(contact_mat);
                contact_surf->SetPhysicsItem(this);
                contact_surf->DisableSelfCollisions(collision_family);
                for (k = 0; k < m_num_faces; k++) {
                    const auto& node1 = m_nodes[idx_vertices[k][0]];
                    const auto& node2 = m_nodes[idx_vertices[k][1]];
                    const auto& node3 = m_nodes[idx_vertices[k][2]];
                    const auto& edge_node1 = (auxdata[k].nbr_node[0] != -1 ? m_nodes[auxdata[k].nbr_node[0]] : nullptr);
                    const auto& edge_node2 = (auxdata[k].nbr_node[1] != -1 ? m_nodes[auxdata[k].nbr_node[1]] : nullptr);
                    const auto& edge_node3 = (auxdata[k].nbr_node[2] != -1 ? m_nodes[auxdata[k].nbr_node[2]] : nullptr);
                    contact_surf->AddFace(node1, node2, node3,                                                        //
                                          edge_node1, edge_node2, edge_node3,                                         //
                                          auxdata[k].owns_node[0], auxdata[k].owns_node[1], auxdata[k].owns_node[2],  //
                                          auxdata[k].owns_edge[0], auxdata[k].owns_edge[1], auxdata[k].owns_edge[2],  //
                                          surface_dim);
                }
                m_contact_surf = contact_surf;
                break;
            }
        }
    }

    // Initialize mesh colors and vertex normals
    colors.resize(m_num_nodes, ChColor(1, 0, 0));
    normals.resize(m_num_nodes, ChVector3d(0, 0, 0));

    // Calculate face normals, accumulate vertex normals, and count number of faces adjacent to each vertex
    std::vector<int> accumulators(m_num_nodes, 0);
    for (int it = 0; it < m_num_faces; it++) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector3d nrm = Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
                                vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][0]]);
        nrm.Normalize();
        // Increment the normals of all incident vertices by the face normal
        normals[idx_normals[it][0]] += nrm;
        normals[idx_normals[it][1]] += nrm;
        normals[idx_normals[it][2]] += nrm;
        // Increment the count of all incident vertices by 1
        accumulators[idx_normals[it][0]] += 1;
        accumulators[idx_normals[it][1]] += 1;
        accumulators[idx_normals[it][2]] += 1;
    }

    // Set vertex normals to average values over all adjacent faces
    for (int in = 0; in < m_num_nodes; in++) {
        normals[in] /= (double)accumulators[in];
    }
}

void MBTireModel::CalculateInertiaProperties(ChVector3d& com, ChMatrix33<>& inertia) {
    //// TODO
}

// Set position and velocity of rim nodes from wheel/spindle state
void MBTireModel::SetRimNodeStates() {
    double dphi = CH_2PI / m_num_divs;
    int k = 0;
    {
        double y = m_offsets[0];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;

            double x = m_rim_radius * std::cos(phi);
            double z = m_rim_radius * std::sin(phi);
            auto pos_loc = ChVector3d(x, y, z);
            m_rim_nodes[k]->SetPos(m_wheel->TransformPointLocalToParent(pos_loc));
            m_rim_nodes[k]->SetPosDt(m_wheel->PointSpeedLocalToParent(pos_loc));
            k++;
        }
    }
    {
        double y = m_offsets[m_num_rings - 1];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;

            double x = m_rim_radius * std::cos(phi);
            double z = m_rim_radius * std::sin(phi);
            auto pos_loc = ChVector3d(x, y, z);
            m_rim_nodes[k]->SetPos(m_wheel->TransformPointLocalToParent(pos_loc));
            m_rim_nodes[k]->SetPosDt(m_wheel->PointSpeedLocalToParent(pos_loc));
            k++;
        }
    }
}

// Calculate and set forces at each node and accumulate wheel loads.
// Note: the positions and velocities of nodes attached to the wheel are assumed to be updated.
void MBTireModel::CalculateForces() {
    // Initialize nodal force accumulators
    std::vector<ChVector3d> nodal_forces(m_num_nodes, VNULL);

    // Initialize wheel force and moment accumulators
    m_wheel_force = VNULL;   // body force, expressed in global frame
    m_wheel_torque = VNULL;  // body torque, expressed in local frame

    // ------------ Gravitational nodal forces

    ChVector3d gforce = m_node_mass * GetSystem()->GetGravitationalAcceleration();
    for (int ir = 0; ir < m_num_rings; ir++) {
        for (int id = 0; id < m_num_divs; id++) {
            int k = NodeIndex(ir, id);
            nodal_forces[k] = gforce;
        }
    }

    // ------------ Load nodal forces

#if defined(FORCE_SEQUENTIAL)

    // Pressure loads
    if (m_tire->IsPressureEnabled()) {
        for (auto& load : m_pressure_loads) {
            load->CalculateForce();
            nodal_forces[load->inode] += load->force;
        }
    }

    // Forces in mesh linear springs (node-node)
    for (auto& spring : m_grid_lin_springs) {
        spring->CalculateForce();
        nodal_forces[spring->inode1] += spring->force1;
        nodal_forces[spring->inode2] += spring->force2;
        // std::cout << spring->force1 << std::endl;
    }

    // Forces in edge linear springs (rim node: node1)
    for (auto& spring : m_edge_lin_springs) {
        spring->CalculateForce();
        m_wheel_force += spring->force_wheel;
        m_wheel_torque += spring->torque_wheel;
        nodal_forces[spring->inode2] += spring->force2;
    }

    // Forces in mesh rotational springs (node-node)
    for (auto& spring : m_grid_rot_springs) {
        spring->CalculateForce();
        nodal_forces[spring->inode_p] += spring->force_p;
        nodal_forces[spring->inode_c] += spring->force_c;
        nodal_forces[spring->inode_n] += spring->force_n;
    }

    // Forces in edge rotational springs (rim node: node_p)
    for (auto& spring : m_edge_rot_springs) {
        spring->CalculateForce();
        m_wheel_force += spring->force_wheel;
        m_wheel_torque += spring->torque_wheel;
        nodal_forces[spring->inode_c] += spring->force_c;
        nodal_forces[spring->inode_n] += spring->force_n;
    }

#elif defined(FORCE_OMP_ALL_AT_ONCE)

    int nthreads = GetSystem()->GetNumThreadsChrono();

    // Loop through all loads in the system and let them evaluate forces
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_loads; is++) {
        m_loads[is]->CalculateForce();
    }

    // Loop through pressure loads and load nodal force
    for (auto& load : m_pressure_loads) {
        nodal_forces[load->inode] += load->force;
    }

    // Loop through each type of spring and load nodal and wheel forces
    for (auto& spring : m_grid_lin_springs) {
        nodal_forces[spring->inode1] += spring->force1;
        nodal_forces[spring->inode2] += spring->force2;
    }

    for (auto& spring : m_edge_lin_springs) {
        m_wheel_force += spring->force_wheel;
        m_wheel_torque += spring->torque_wheel;
        nodal_forces[spring->inode2] += spring->force2;
    }

    for (auto& spring : m_grid_rot_springs) {
        nodal_forces[spring->inode_p] += spring->force_p;
        nodal_forces[spring->inode_c] += spring->force_c;
        nodal_forces[spring->inode_n] += spring->force_n;
    }

    for (auto& spring : m_edge_rot_springs) {
        m_wheel_force += spring->force_wheel;
        m_wheel_torque += spring->torque_wheel;
        nodal_forces[spring->inode_c] += spring->force_c;
        nodal_forces[spring->inode_n] += spring->force_n;
    }

#elif defined(FORCE_OMP_CRITICAL_SECTION)

    int nthreads = GetSystem()->GetNumThreadsChrono();

    // Pressure forces
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_pressure_loads; is++) {
        auto& load = m_pressure_loads[is];
        load->CalculateForce();
    #pragma omp critical
        {
            nodal_forces[load->inode] += load->force;
        }
    }

    // Forces in mesh linear springs (node-node)
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_grid_lin_springs; is++) {
        auto& spring = m_grid_lin_springs[is];
        spring->CalculateForce();
    #pragma omp critical
        {
            nodal_forces[spring->inode1] += spring->force1;
            nodal_forces[spring->inode2] += spring->force2;
        }
    }

    // Forces in edge linear springs (rim node: node1)
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_edge_lin_springs; is++) {
        auto& spring = m_edge_lin_springs[is];
        spring->CalculateForce();
    #pragma omp critical
        {
            m_wheel_force += spring->force_wheel;
            m_wheel_torque += spring->torque_wheel;
            nodal_forces[spring->inode2] += spring->force2;
        }
    }

    // Forces in mesh rotational springs (node-node)
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_grid_rot_springs; is++) {
        auto& spring = m_grid_rot_springs[is];
        spring->CalculateForce();
    #pragma omp critical
        {
            nodal_forces[spring->inode_p] += spring->force_p;
            nodal_forces[spring->inode_c] += spring->force_c;
            nodal_forces[spring->inode_n] += spring->force_n;
        }
    }

    // Forces in edge rotational springs (rim node: node_p)
    #pragma omp parallel for schedule(dynamic, 4) num_threads(nthreads)
    for (int is = 0; is < m_num_edge_rot_springs; is++) {
        auto& spring = m_edge_rot_springs[is];
        spring->CalculateForce();
    #pragma omp critical
        {
            m_wheel_force += spring->force_wheel;
            m_wheel_torque += spring->torque_wheel;
            nodal_forces[spring->inode_c] += spring->force_c;
            nodal_forces[spring->inode_n] += spring->force_n;
        }
    }

#endif

    // ------------ Apply loads on FEA nodes

    for (int k = 0; k < m_num_nodes; k++) {
        m_nodes[k]->SetForce(nodal_forces[k]);
    }
}

// -----------------------------------------------------------------------------

void MBTireModel::SyncCollisionModels() {
    if (m_contact_surf)
        m_contact_surf->SyncCollisionModels();
}

void MBTireModel::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    assert(GetSystem());
    if (m_contact_surf) {
        m_contact_surf->SyncCollisionModels();
        m_contact_surf->AddCollisionModelsToSystem(coll_sys);
    }
}

void MBTireModel::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    assert(GetSystem());
    if (m_contact_surf)
        m_contact_surf->RemoveCollisionModelsFromSystem(coll_sys);
}

// -----------------------------------------------------------------------------

// Notes:
// The implementation of these functions is similar to those in ChMesh.
// It is assumed that none of the FEA nodes is fixed.

void MBTireModel::SetupInitial() {
    // Calculate DOFs and state offsets
    m_dofs = 0;
    m_dofs_w = 0;
    for (auto& node : m_nodes) {
        node->SetupInitial(GetSystem());
        m_dofs += node->GetNumCoordsPosLevelActive();
        m_dofs_w += node->GetNumCoordsVelLevelActive();
    }
}

//// TODO: Cache (normalized) locations around the rim to save sin/cos evaluations
void MBTireModel::Setup() {
    // Recompute DOFs and state offsets
    m_dofs = 0;
    m_dofs_w = 0;
    for (auto& node : m_nodes) {
        // Set node offsets in state vectors (based on the offsets of the container)
        node->NodeSetOffsetPosLevel(GetOffset_x() + m_dofs);
        node->NodeSetOffsetVelLevel(GetOffset_w() + m_dofs_w);

        // Count the actual degrees of freedom (consider only nodes that are not fixed)
        m_dofs += node->GetNumCoordsPosLevelActive();
        m_dofs_w += node->GetNumCoordsVelLevelActive();
    }

    // Update visualization mesh
    auto trimesh = m_trimesh_shape->GetMesh();
    auto& vertices = trimesh->GetCoordsVertices();
    auto& normals = trimesh->GetCoordsNormals();
    auto& colors = trimesh->GetCoordsColors();

    for (int k = 0; k < m_num_nodes; k++) {
        vertices[k] = m_nodes[k]->GetPos();
    }

    //// TODO: update vertex normals and colors
}

void MBTireModel::Update(double t, bool update_assets) {
    // Parent class update
    ChPhysicsItem::Update(t, update_assets);
}

// -----------------------------------------------------------------------------

void MBTireModel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_trimesh_shape->SetWireframe(true);
    AddVisualShape(m_trimesh_shape);

    for (const auto& node : m_rim_nodes) {
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(0.01);
        sph->SetColor(ChColor(1.0f, 0.0f, 0.0f));
        auto loc = m_wheel->TransformPointParentToLocal(node->GetPos());
        m_wheel->AddVisualShape(sph, ChFrame<>(loc));
    }

    //// TODO
    //// properly remove visualization assets added to the wheel body (requires caching the visual shapes)
}

// =============================================================================

void MBTireModel::InjectVariables(ChSystemDescriptor& descriptor) {
    for (auto& node : m_nodes)
        node->InjectVariables(descriptor);
}

void MBTireModel::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    if (!m_stiff)
        return;
    for (auto& load : m_pressure_loads)
        descriptor.InsertKRMBlock(&load->KRM);
    for (auto& spring : m_grid_lin_springs)
        descriptor.InsertKRMBlock(&spring->KRM);
    for (auto& spring : m_edge_lin_springs)
        descriptor.InsertKRMBlock(&spring->KRM);
    for (auto& spring : m_grid_rot_springs)
        descriptor.InsertKRMBlock(&spring->KRM);
    for (auto& spring : m_edge_rot_springs)
        descriptor.InsertKRMBlock(&spring->KRM);
}

void MBTireModel::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    if (!m_stiff && !m_force_jac)
        return;

    for (auto& load : m_pressure_loads)
        load->CalculateJacobian(Kfactor, Rfactor);
    for (auto& spring : m_grid_lin_springs)
        spring->CalculateJacobian(Kfactor, Rfactor);
    for (auto& spring : m_edge_lin_springs)
        spring->CalculateJacobian(Kfactor, Rfactor);
    for (auto& spring : m_grid_rot_springs)
        spring->CalculateJacobian(Kfactor, Rfactor);
    for (auto& spring : m_edge_rot_springs)
        spring->CalculateJacobian(Kfactor, Rfactor);
}

void MBTireModel::IntStateGather(const unsigned int off_x,
                                 ChState& x,
                                 const unsigned int off_v,
                                 ChStateDelta& v,
                                 double& T) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
        local_off_x += node->GetNumCoordsPosLevelActive();
        local_off_v += node->GetNumCoordsVelLevelActive();
    }

    T = GetChTime();
}

void MBTireModel::IntStateScatter(const unsigned int off_x,
                                  const ChState& x,
                                  const unsigned int off_v,
                                  const ChStateDelta& v,
                                  const double T,
                                  bool full_update) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
        local_off_x += node->GetNumCoordsPosLevelActive();
        local_off_v += node->GetNumCoordsVelLevelActive();
    }

    Update(T, full_update);
}

void MBTireModel::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGatherAcceleration(off_a + local_off_a, a);
        local_off_a += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateScatterAcceleration(off_a + local_off_a, a);
        local_off_a += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntStateIncrement(const unsigned int off_x,
                                    ChState& x_new,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
        local_off_x += node->GetNumCoordsPosLevelActive();
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntStateGetIncrement(const unsigned int off_x,
                                       const ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
        local_off_x += node->GetNumCoordsPosLevelActive();
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // Synchronize position and velocity of rim nodes with wheel/spindle state
    SetRimNodeStates();

    // Calculate spring forces:
    // - set them as applied forces on the FEA nodes
    // - accumulate force and torque on wheel/spindle body
    CalculateForces();

    // Load nodal forces into R
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntLoadResidual_F(off + local_off_v, R, c);
        local_off_v += node->GetNumCoordsVelLevelActive();
    }

    // Load wheel body forces into R
    if (m_wheel->Variables().IsActive()) {
        R.segment(m_wheel->Variables().GetOffset() + 0, 3) += c * m_wheel_force.eigen();
        R.segment(m_wheel->Variables().GetOffset() + 3, 3) += c * m_wheel_torque.eigen();
    }
}

void MBTireModel::IntLoadResidual_Mv(const unsigned int off,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& w,
                                     const double c) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntLoadResidual_Mv(off + local_off_v, R, w, c);
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntLoadLumpedMass_Md(const unsigned int off, ChVectorDynamic<>& Md, double& err, const double c) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntLoadLumpedMass_Md(off + local_off_v, Md, err, c);
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntToDescriptor(const unsigned int off_v,
                                  const ChStateDelta& v,
                                  const ChVectorDynamic<>& R,
                                  const unsigned int off_L,
                                  const ChVectorDynamic<>& L,
                                  const ChVectorDynamic<>& Qc) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntToDescriptor(off_v + local_off_v, v, R);
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

void MBTireModel::IntFromDescriptor(const unsigned int off_v,
                                    ChStateDelta& v,
                                    const unsigned int off_L,
                                    ChVectorDynamic<>& L) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntFromDescriptor(off_v + local_off_v, v);
        local_off_v += node->GetNumCoordsVelLevelActive();
    }
}

}  // end namespace vehicle
}  // end namespace chrono