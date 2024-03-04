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

//#define BEAM_VERBOSE

#include "chrono/fea/ChElementBeamIGA.h"

namespace chrono {
namespace fea {

// for testing and debugging
ChElementBeamIGA::QuadratureType ChElementBeamIGA::quadrature_type = ChElementBeamIGA::QuadratureType::FULL_EXACT;
double ChElementBeamIGA::Delta = 1e-10;
bool ChElementBeamIGA::lumped_mass = true;
bool ChElementBeamIGA::add_gyroscopic_terms = true;

ChElementBeamIGA::ChElementBeamIGA() {
    order = 3;
    nodes.resize(4);  // controllare se ordine = -> 2 nodi, 2 control points, o di pi?
    knots.resize(8);
    int_order_s = 1;
    int_order_b = 1;
}

void ChElementBeamIGA::SetNodesCubic(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                                     std::shared_ptr<ChNodeFEAxyzrot> nodeD,
                                     double knotA1,
                                     double knotA2,
                                     double knotB1,
                                     double knotB2,
                                     double knotB3,
                                     double knotB4,
                                     double knotB5,
                                     double knotB6) {
    nodes.resize(4);
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    nodes[2] = nodeC;
    nodes[3] = nodeD;
    knots.resize(8);
    knots(0) = knotA1;
    knots(1) = knotA2;
    knots(2) = knotB1;
    knots(3) = knotB2;
    knots(4) = knotB3;
    knots(5) = knotB4;
    knots(6) = knotB5;
    knots(7) = knotB6;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    mvars.push_back(&nodes[2]->Variables());
    mvars.push_back(&nodes[3]->Variables());
    Kmatr.SetVariables(mvars);

    int_order_s = 1;
    int_order_b = 1;
}

void ChElementBeamIGA::SetNodesGenericOrder(std::vector<std::shared_ptr<ChNodeFEAxyzrot>> mynodes,
                                            std::vector<double> myknots,
                                            int myorder) {
    this->order = myorder;

    nodes.resize(myorder + 1);
    for (int i = 0; i < mynodes.size(); ++i) {
        nodes[i] = mynodes[i];
    }
    knots.resize(nodes.size() + myorder + 1);
    for (int i = 0; i < myknots.size(); ++i) {
        knots(i) = myknots[i];
    }

    std::vector<ChVariables*> mvars;
    for (int i = 0; i < mynodes.size(); ++i) {
        mvars.push_back(&nodes[i]->Variables());
    }
    Kmatr.SetVariables(mvars);

    // integration for in-between elements:
    // int_order_s = 1;

    // FULL OVER INTEGRATION:
    // int_order_b = myorder+1;
    // FULL EXACT INTEGRATION:
    int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
    // REDUCED INTEGRATION:
    // int_order_b = myorder;
    // SELECTIVE INTEGRATION:
    // int_order_b = 1;

    // ensure integration order is odd, to have a centered gauss point
    // if (int_order_b % 2 == 0) {
    //	int_order_b += 1;
    //}

    bool is_middle = false;
    bool is_end_A = false;
    bool is_end_B = false;

    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    if (u1 < 0.5 && u2 >= 0.5) {
        // GetLog() << " -- IS_MIDDLE ";
        is_middle = true;
    }
    // Full integration for end elements:
    int multiplicity_a = 1;
    int multiplicity_b = 1;
    for (int im = order - 1; im >= 0; --im) {
        if (knots(im) == knots(order))  // extreme of span
            ++multiplicity_a;
    }
    for (int im = (int)knots.size() - order; im < (int)knots.size(); ++im) {
        if (knots(im) == knots(knots.size() - order - 1))  // extreme of span
            ++multiplicity_b;
    }
    if (multiplicity_a > 1)
        is_end_A = true;
    if (multiplicity_b > 1)
        is_end_B = true;

    if (this->quadrature_type == QuadratureType::FULL_EXACT) {
        int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
        int_order_s = (int)std::ceil((this->order + 1.0) / 2.0);
    }
    if (this->quadrature_type == QuadratureType::FULL_OVER) {
        int_order_b = myorder + 1;
        int_order_s = myorder + 1;
    }
    if (this->quadrature_type == QuadratureType::REDUCED) {
        int_order_b = myorder;
        int_order_s = myorder;
    }
    if (this->quadrature_type == QuadratureType::SELECTIVE) {
        int_order_b = 1;  // int_order_b = my_order;
        int_order_s = 1;
        if (is_end_A || is_end_B) {
            int_order_b = (int)std::ceil((this->order + 1.0) / 2.0);
            int_order_s = (int)std::ceil((this->order + 1.0) / 2.0);
        }
    }
    if (this->quadrature_type == QuadratureType::CUSTOM1) {
        int_order_b = 1;
        int_order_s = 1;
        if (order == 2) {
            if (is_middle) {
                int_order_s = 2;
                int_order_b = 2;
            }
        }
        if (order >= 3) {
            if (is_end_A || is_end_B) {
                int_order_b = order - 1;
                int_order_s = order - 1;
            }
        }
    }
    if (this->quadrature_type == QuadratureType::URI2) {
        int_order_b = 1;
        int_order_s = 1;
        if (order == 2) {
            if (is_end_B) {
                int_order_s = 2;
                int_order_b = 2;
            }
        }
        if (order >= 3) {
            if (is_end_A || is_end_B) {
                int_order_b = order - 1;
                int_order_s = order - 1;
            }
        }
    }

    // TRICK - FOR THE MOMENT ENSURE NO SELECTIVE CAUSE NOT YET IMPLEMENTED DIFFERENT GAUSS PTS FOR BEND/SHEAR
    int_order_s = int_order_b;

    /*
    GetLog() << "Element order p=" << this->order << ",   u in (" << u1 << ", " << u2 << ")";
    GetLog() << "   orders:  b=" << int_order_b << "   s=" << int_order_s;
    GetLog() << "\n";
    */
}

/// Set the integration points, for shear components and for bending components:

void ChElementBeamIGA::SetIntegrationPoints(int npoints_s, int npoints_b) {
    int_order_s = npoints_s;
    int_order_b = npoints_b;
}

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementBeamIGA::ComputeNodalMass() {
    for (int i = 0;i<nodes.size();++i)
        nodes[i]->m_TotalMass += this->mass / nodes.size();
}

/// Sets H as the global stiffness matrix K, scaled  by Kfactor. Optionally, also
/// superimposes global damping matrix R, scaled by Rfactor, and global mass matrix M multiplied by Mfactor.

void ChElementBeamIGA::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6 * nodes.size()) && (H.cols() == 6 * nodes.size()));
    assert(section);

    // BRUTE FORCE METHOD: USE NUMERICAL DIFFERENTIATION!

    //
    // The K stiffness matrix of this element span:
    //

    ChState state_x(this->LoadableGet_ndof_x(), nullptr);
    ChStateDelta state_w(this->LoadableGet_ndof_w(), nullptr);
    this->LoadableGetStateBlock_x(0, state_x);
    this->LoadableGetStateBlock_w(0, state_w);

    int mrows_w = this->LoadableGet_ndof_w();
    int mrows_x = this->LoadableGet_ndof_x();

    // compute Q at current speed & position, x_0, v_0
    ChVectorDynamic<> Q0(mrows_w);
    this->ComputeInternalForces_impl(Q0, state_x, state_w, true);  // Q0 = Q(x, v)

    ChVectorDynamic<> Q1(mrows_w);
    ChVectorDynamic<> Jcolumn(mrows_w);

    if (Kfactor) {
        ChMatrixDynamic<> K(mrows_w, mrows_w);

        ChState state_x_inc(mrows_x, nullptr);
        ChStateDelta state_delta(mrows_w, nullptr);

        // Compute K=-dQ(x,v)/dx by backward differentiation
        state_delta.setZero(mrows_w, nullptr);

        for (int i = 0; i < mrows_w; ++i) {
            state_delta(i) += Delta;
            this->LoadableStateIncrement(0, state_x_inc, state_x, 0,
                                         state_delta);  // exponential, usually state_x_inc(i) = state_x(i) + Delta;

            Q1.setZero(mrows_w);
            this->ComputeInternalForces_impl(Q1, state_x_inc, state_w, true);  // Q1 = Q(x+Dx, v)
            state_delta(i) -= Delta;

            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because K=-dQ/dx
            K.col(i) = Jcolumn;
        }

        // finally, store K into H:
        H.block(0, 0, mrows_w, mrows_w) = Kfactor * K;
    } else
        H.setZero();

    //
    // The R damping matrix of this element:
    //

    if (Rfactor && this->section->GetDamping()) {
        // Compute R=-dQ(x,v)/dv by backward differentiation
        ChStateDelta state_w_inc(mrows_w, nullptr);
        state_w_inc = state_w;
        ChMatrixDynamic<> R(mrows_w, mrows_w);

        for (int i = 0; i < mrows_w; ++i) {
            Q1.setZero(mrows_w);

            state_w_inc(i) += Delta;
            this->ComputeInternalForces_impl(Q1, state_x, state_w_inc, true);  // Q1 = Q(x, v+Dv)
            state_w_inc(i) -= Delta;

            Jcolumn = (Q1 - Q0) * (-1.0 / Delta);  // - sign because R=-dQ/dv
            R.col(i) = Jcolumn;
        }
        H.block(0, 0, mrows_w, mrows_w) += Rfactor * R;
    }

    // Add inertial stiffness matrix and inertial damping matrix (gyroscopic damping),
    // if enabled in section material.
    // These matrices are not symmetric.
    // A lumped version of the inertial damping/stiffness matrix computation is used here,
    // on a per-node basis. In future one could implement also a consistent version of it, optionally, depending on
    // ChElementBeamIGA::lumped_mass.
    if ((Rfactor && this->section->GetInertia()->compute_inertia_damping_matrix) ||
        (Kfactor && this->section->GetInertia()->compute_inertia_stiffness_matrix)) {
        ChMatrixNM<double, 6, 6> matr_loc;
        ChMatrixNM<double, 6, 6> KRi_loc;
        KRi_loc.setZero();

        double node_multiplier_fact_R = 0.5 * length * Rfactor;
        double node_multiplier_fact_K = 0.5 * length * Kfactor;
        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6;
            if (this->section->GetInertia()->compute_inertia_damping_matrix) {
                this->section->GetInertia()->ComputeInertiaDampingMatrix(matr_loc, nodes[i]->GetWvel_loc());
                KRi_loc += matr_loc * node_multiplier_fact_R;
            }
            if (this->section->GetInertia()->compute_inertia_stiffness_matrix) {
                this->section->GetInertia()->ComputeInertiaStiffnessMatrix(
                    matr_loc, nodes[i]->GetWvel_loc(), nodes[i]->GetWacc_loc(),
                    (nodes[i]->GetA().transpose()) * nodes[i]->GetPos_dtdt());  // assume x_dtdt in local frame!
                KRi_loc += matr_loc * node_multiplier_fact_K;
            }
            // corotate the local damping and stiffness matrices (at once, already scaled) into absolute one
            // H.block<3, 3>(stride,   stride  ) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0,0) *
            // (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(0,0) is null by construction
            H.block<3, 3>(stride + 3, stride + 3) += KRi_loc.block<3, 3>(3, 3);
            H.block<3, 3>(stride, stride + 3) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0, 3);
            // H.block<3, 3>(stride+3, stride)   +=                    KRi_loc.block<3, 3>(3,0) *
            // (nodes[i]->GetA().transpose());  // NOTE: not needed as KRi_loc.block<3, 3>(3,0) is null by construction
        }
    }

    //
    // The M mass matrix of this element span:
    //

    if (Mfactor) {
        ChMatrixDynamic<> Mloc(6 * nodes.size(), 6 * nodes.size());
        Mloc.setZero();
        ChMatrix33<> Mxw;

        if (ChElementBeamIGA::lumped_mass == true) {
            //
            // "lumped" M mass matrix
            //
            ChMatrixNM<double, 6, 6> sectional_mass;
            this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);

            double node_multiplier = length / (double)nodes.size();
            for (int i = 0; i < nodes.size(); ++i) {
                int stride = i * 6;
                // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                // hence it can be the simple (constant) expression
                //   Mloc.block<6, 6>(stride, stride) += sectional_mass * (node_multiplier * Mfactor);
                // but the more general case needs the rotations, hence:
                Mloc.block<3, 3>(stride, stride) += sectional_mass.block<3, 3>(0, 0) * (node_multiplier * Mfactor);
                Mloc.block<3, 3>(stride + 3, stride + 3) +=
                    sectional_mass.block<3, 3>(3, 3) * (node_multiplier * Mfactor);
                Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0, 3) * (node_multiplier * Mfactor);
                Mloc.block<3, 3>(stride, stride + 3) += Mxw;
                Mloc.block<3, 3>(stride + 3, stride) += Mxw.transpose();
            }
        } else {
            //
            // "consistent" M mass matrix, via Gauss quadrature
            //
            ChMatrixNM<double, 6, 6> sectional_mass;
            this->section->GetInertia()->ComputeInertiaMatrix(sectional_mass);

            double u1 = knots(order);
            double u2 = knots(knots.size() - order - 1);

            double c1 = (u2 - u1) / 2;
            double c2 = (u2 + u1) / 2;

            // Do quadrature over the Gauss points - reuse the "b" bend Gauss points

            for (int ig = 0; ig < int_order_b; ++ig) {
                // absyssa in typical -1,+1 range:
                double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
                // absyssa in span range:
                double u = (c1 * eta + c2);
                // scaling = gauss weight
                double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

                // Jacobian Jsu = ds/du
                double Jsu = this->Jacobian_b[ig];
                // Jacobian Jue = du/deta
                double Jue = c1;

                // compute the basis functions N(u) at given u:
                int nspan = order;

                ChVectorDynamic<> N((int)nodes.size());
                geometry::ChBasisToolsBspline::BasisEvaluate(this->order, nspan, u, knots, N);

                /*
                // interpolate rotation of section at given u, to compute R.
                ChQuaternion<> q_delta;
                ChVector<> da = VNULL;
                ChVector<> delta_rot_dir;
                double delta_rot_angle;
                for (int i = 0; i < nodes.size(); ++i) {
                    ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                    q_delta = nodes[0]->coord.rot.GetConjugate() * q_i;
                    q_delta.Q_to_AngAxis(delta_rot_angle, delta_rot_dir); // a_i = dir_i*angle_i (in spline local
                reference, -PI..+PI) da += delta_rot_dir * delta_rot_angle * N(i);  // a = N_i*a_i
                }
                ChQuaternion<> qda; qda.Q_from_Rotv(da);
                ChQuaternion<> qR = nodes[0]->coord.rot * qda;

                // compute the 3x3 rotation matrix R equivalent to quaternion above
                ChMatrix33<> R(qR);
                */

                // A simplification, for moderate bending, ignore the difference between beam section R
                // and R_i of nearby nodes, obtaining:

                for (int i = 0; i < nodes.size(); ++i) {
                    for (int j = 0; j < nodes.size(); ++j) {
                        int istride = i * 6;
                        int jstride = j * 6;

                        double Ni_Nj = N(i) * N(j);
                        double gmassfactor = Mfactor * w * Jsu * Jue * Ni_Nj;

                        // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                        // hence it can be the simple (constant) expression
                        //   Mloc.block<6, 6>(istride + 0, jstride + 0) += gmassfactor * sectional_mass;
                        // but the more general case needs the rotations, hence:
                        Mloc.block<3, 3>(istride, jstride) += sectional_mass.block<3, 3>(0, 0) * gmassfactor;
                        Mloc.block<3, 3>(istride + 3, jstride + 3) += sectional_mass.block<3, 3>(3, 3) * gmassfactor;
                        Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0, 3) * gmassfactor;
                        Mloc.block<3, 3>(istride, jstride + 3) += Mxw;
                        Mloc.block<3, 3>(istride + 3, jstride) += Mxw.transpose();
                    }
                }

            }  // end loop on gauss points
        }

        H.block(0, 0, Mloc.rows(), Mloc.cols()) += Mloc;
    }
}

/// Computes the internal forces (ex. the actual position of nodes is not in relaxed reference position) and set
/// values in the Fi vector.

void ChElementBeamIGA::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    ChState mstate_x(this->LoadableGet_ndof_x(), nullptr);
    ChStateDelta mstate_w(this->LoadableGet_ndof_w(), nullptr);
    this->LoadableGetStateBlock_x(0, mstate_x);
    this->LoadableGetStateBlock_w(0, mstate_w);
    ComputeInternalForces_impl(Fi, mstate_x, mstate_w);
}

void ChElementBeamIGA::ComputeInternalForces_impl(ChVectorDynamic<>& Fi,
                                                  ChState& state_x,
                                                  ChStateDelta& state_w,
                                                  bool used_for_differentiation) {
    // get two values of absyssa at extreme of span
    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    double c1 = (u2 - u1) / 2;
    double c2 = (u2 + u1) / 2;

    // zero the Fi accumulator
    Fi.setZero();

    // Do quadrature over the "s" shear Gauss points
    // (only if int_order_b != int_order_s, otherwise do a single loop later over "b" bend points also for shear)

    //***TODO*** maybe not needed: separate bending and shear integration points.

    // Do quadrature over the "b" bend Gauss points

    for (int ig = 0; ig < int_order_b; ++ig) {
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        double u = (c1 * eta + c2);
        // scaling = gauss weight
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
        // Jacobian Jue = du/deta
        double Jue = c1;

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChMatrixDynamic<> N(2, (int)nodes.size());  // row n.0 contains N, row n.1 contains dN/du

        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(this->order, nspan, u, knots,
                                                          N);  ///< here return N and dN/du

        // interpolate rotation of section at given u, to compute R.
        // Note: this is approximate.
        // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
        ChQuaternion<> q_delta;
        ChVector<> da = VNULL;
        ChVector<> delta_rot_dir;
        double delta_rot_angle;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            q_delta = nodes[0]->coord.rot.GetConjugate() * q_i;
            q_delta.Q_to_AngAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * N(0, i);  // a = N_i*a_i
        }
        ChQuaternion<> qda;
        qda.Q_from_Rotv(da);
        ChQuaternion<> qR = nodes[0]->coord.rot * qda;

        // compute the 3x3 rotation matrix R equivalent to quaternion above
        ChMatrix33<> R(qR);

        // compute abs. spline gradient r'  = dr/ds
        ChVector<> dr;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector<> r_i(state_x.segment(i * 7, 3));
            dr += r_i * N(1, i);  // dr/du = N_i'*r_i
        }
        // (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
        dr *= 1.0 / Jsu;

        // compute abs. time rate of spline gradient  dr'/dt
        ChVector<> drdt;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector<> drdt_i(state_w.segment(i * 6, 3));
            drdt += drdt_i * N(1, i);
        }
        drdt *= 1.0 / Jsu;

        // compute abs spline rotation gradient q' = dq/ds
        // But.. easier to compute local gradient of spin vector a' = da/ds
        da = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            q_delta = qR.GetConjugate() * q_i;
            q_delta.Q_to_AngAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * N(1, i);  // da/du = N_i'*a_i
        }
        // (note a= da/ds = da/du du/ds = da/du * 1/Jsu   where Jsu computed in SetupInitial)
        da *= 1.0 / Jsu;

        // compute abs rate of spline rotation gradient da'/dt
        ChVector<> dadt;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            ChVector<> wl_i(state_w.segment(i * 6 + 3, 3));  //  w in node csys
            ChVector<> w_i = q_i.Rotate(wl_i);               // w in absolute csys
            dadt += w_i * N(1, i);
        }
        dadt *= 1.0 / Jsu;

        // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
        ChVector<> astrain_e = R.transpose() * dr - VECT_X - this->strain_e_0[ig];

        // compute local time rate of strain:  strain_e_dt = R^t * dr'/dt
        ChVector<> astrain_e_dt = R.transpose() * drdt;

        // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' = a'_local
        ChVector<> astrain_k = da - this->strain_k_0[ig];

        // compute local time rate of curvature strain:
        ChVector<> astrain_k_dt = R.transpose() * dadt;

        // compute stress n  (local cut forces)
        // compute stress_m  (local cut torque)
        ChVector<> astress_n;
        ChVector<> astress_m;
        ChBeamMaterialInternalData* aplastic_data_old = nullptr;
        ChBeamMaterialInternalData* aplastic_data = nullptr;
        std::vector<std::unique_ptr<ChBeamMaterialInternalData>> foo_plastic_data;
        if (this->section->GetPlasticity()) {
            aplastic_data_old = this->plastic_data_old[ig].get();
            aplastic_data = this->plastic_data[ig].get();
            if (used_for_differentiation) {
                // Avoid updating plastic_data_new if ComputeInternalForces_impl() called for computing
                // stiffness matrix by backward differentiation. Otherwise pollutes the plastic_data integration.
                this->section->GetPlasticity()->CreatePlasticityData(1, foo_plastic_data);
                aplastic_data = foo_plastic_data[0].get();
            }
        }
        this->section->ComputeStress(astress_n, astress_m, astrain_e, astrain_k, aplastic_data, aplastic_data_old);

        if (!used_for_differentiation) {
            this->stress_n[ig] = astress_n;
            this->stress_m[ig] = astress_m;
            this->strain_e[ig] = astrain_e;
            this->strain_k[ig] = astrain_k;
        }

        // add viscous damping
        if (this->section->GetDamping()) {
            ChVector<> n_sp;
            ChVector<> m_sp;
            this->section->GetDamping()->ComputeStress(n_sp, m_sp, astrain_e_dt, astrain_k_dt);
            astress_n += n_sp;
            astress_m += m_sp;
        }

        // compute internal force, in generalized coordinates:

        ChVector<> stress_n_abs = R * astress_n;
        ChVector<> stress_m_abs = R * astress_m;

        for (int i = 0; i < nodes.size(); ++i) {
            // -Force_i = w * Jue * Jsu * Jsu^-1 * N' * R * C * (strain_e - strain_e0)
            //          = w * Jue                * N'         * stress_n_abs
            ChVector<> Force_i = stress_n_abs * N(1, i) * (-w * Jue);
            Fi.segment(i * 6, 3) += Force_i.eigen();

            // -Torque_i =   w * Jue * Jsu * Jsu^-1 * R_i^t * N'               * R * D * (strain_k - strain_k0) +
            //             + w * Jue * Jsu *          R_i^t * N * skew(r')^t   * R * C * (strain_e - strain_e0)
            //           =   w * Jue * R_i^t                * N'               * stress_m_abs +
            //             + w * Jue * Jsu * R_i^t          * N * skew(r')^t   * stress_n_abs
            ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
            ChVector<> Torque_i = q_i.RotateBack(stress_m_abs * N(1, i) * (-w * Jue) -
                                                 Vcross(dr, stress_n_abs) * N(0, i) * (-w * Jue * Jsu));
            Fi.segment(3 + i * 6, 3) += Torque_i.eigen();
        }

        // GetLog() << "     gp n." << ig <<   "  J=" << this->Jacobian[ig] << "   strain_e= " << strain_e << "\n";
        // GetLog() << "                    stress_n= " << stress_n << "\n";
    }

    // Add also inertial quadratic terms: gyroscopic and centrifugal

    if (ChElementBeamIGA::add_gyroscopic_terms == true) {
        if (ChElementBeamIGA::lumped_mass == true) {
            // CASE OF LUMPED MASS - faster
            double node_multiplier = length / (double)nodes.size();
            ChVector<> mFcent_i;
            ChVector<> mTgyro_i;
            for (int i = 0; i < nodes.size(); ++i) {
                this->section->GetInertia()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, state_w.segment(3 + i * 6, 3));
                ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                Fi.segment(i * 6, 3) -= node_multiplier * q_i.Rotate(mFcent_i).eigen();
                Fi.segment(3 + i * 6, 3) -= node_multiplier * mTgyro_i.eigen();
            }
        } else {
            // CASE OF CONSISTENT MASS
            ChVector<> mFcent_i;
            ChVector<> mTgyro_i;
            // evaluate inertial quadratic forces using same gauss points used for bending
            for (int ig = 0; ig < int_order_b; ++ig) {
                double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
                double u = (c1 * eta + c2);
                double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

                double Jsu = this->Jacobian_b[ig];
                double Jue = c1;
                int nspan = order;

                ChMatrixDynamic<> N(1, (int)nodes.size());  // row n.0 contains N, row n.1 contains dN/du
                geometry::ChBasisToolsBspline::BasisEvaluateDeriv(this->order, nspan, u, knots,
                                                                  N);  ///< here return N and dN/du

                // interpolate rotation of section at given u, to compute R.
                // Note: this is approximate.
                // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
                ChQuaternion<> q_delta;
                ChVector<> da = VNULL;
                ChVector<> delta_rot_dir;
                double delta_rot_angle;
                for (int i = 0; i < nodes.size(); ++i) {
                    ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                    q_delta = nodes[0]->coord.rot.GetConjugate() * q_i;
                    q_delta.Q_to_AngAxis(delta_rot_angle,
                                         delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
                    da += delta_rot_dir * delta_rot_angle * N(0, i);  // a = N_i*a_i
                }
                ChQuaternion<> qda;
                qda.Q_from_Rotv(da);
                ChQuaternion<> qR = nodes[0]->coord.rot * qda;

                ChVector<> w_sect;
                for (int i = 0; i < nodes.size(); ++i) {
                    ChQuaternion<> q_i(state_x.segment(i * 7 + 3, 4));
                    ChVector<> wl_i(state_w.segment(i * 6 + 3, 3));  //  w in node csys
                    ChVector<> w_i = q_i.Rotate(wl_i);               // w in absolute csys
                    w_sect += w_i * N(0, i);
                }
                w_sect = qR.RotateBack(w_sect);  // w in sectional csys

                this->section->GetInertia()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, w_sect);

                for (int i = 0; i < nodes.size(); ++i) {
                    Fi.segment(i * 6, 3) -= w * Jsu * Jue * N(0, i) * qR.Rotate(mFcent_i).eigen();
                    Fi.segment(3 + i * 6, 3) -= w * Jsu * Jue * N(0, i) * mTgyro_i.eigen();
                }
            }
        }
    }  // end quadratic inertial force terms
}

void ChElementBeamIGA::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    // no so efficient... a temporary mass matrix here:
    ChMatrixDynamic<> mM(this->GetNdofs(), this->GetNdofs());
    this->ComputeMmatrixGlobal(mM);

    // a vector of G accelerations (for translation degrees of freedom ie 3 xyz every 6 values)
    ChVectorDynamic<> mG(this->GetNdofs());
    mG.setZero();
    for (int i = 0; i < nodes.size(); ++i) {
        mG.segment(i * 6, 3) = G_acc.eigen();
    }

    // Gravity forces as M*g, always works, regardless of the way M
    // is computed (lumped or consistent, with offset center of mass or centered, etc.)
    // [Maybe one can replace this function with a faster ad-hoc implementation in case of lumped masses.]
    Fg = mM * mG;
}

/// Evaluate N'*F , where N is some type of shape function
/// evaluated at U coordinates of the line, ranging in -1..+1
/// F is a load, N'*F is the resulting generalized load
/// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.

void ChElementBeamIGA::ComputeNF(const double U,
                                 ChVectorDynamic<>& Qi,
                                 double& detJ,
                                 const ChVectorDynamic<>& F,
                                 ChVectorDynamic<>* state_x,
                                 ChVectorDynamic<>* state_w) {
    // get two values of absyssa at extreme of span
    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    double c1 = (u2 - u1) / 2;
    double c2 = (u2 + u1) / 2;

    // absyssa in span range:
    double u = (c1 * U + c2);

    // compute the basis functions N(u) at given u:
    int nspan = order;

    ChMatrixDynamic<> N(2, (int)nodes.size());  // row n.0 contains N, row n.1 contains dN/du

    geometry::ChBasisToolsBspline::BasisEvaluateDeriv(this->order, nspan, u, knots,
                                                      N);  ///< h

    ChVector<> dr0;
    for (int i = 0; i < nodes.size(); ++i) {
        dr0 += nodes[i]->GetX0ref().coord.pos * N(1, i);
    }
    detJ = dr0.Length() * c1;

    for (int i = 0; i < nodes.size(); ++i) {
        int stride = i * 6;
        Qi(stride + 0) = N(i) * F(0);
        Qi(stride + 1) = N(i) * F(1);
        Qi(stride + 2) = N(i) * F(2);
        Qi(stride + 3) = N(i) * F(3);
        Qi(stride + 4) = N(i) * F(4);
        Qi(stride + 5) = N(i) * F(5);
    }
}

/// Evaluate N'*F , where N is some type of shape function
/// evaluated at U,V,W coordinates of the volume, each ranging in -1..+1
/// F is a load, N'*F is the resulting generalized load
/// Returns also det[J] with J=[dx/du,..], that might be useful in gauss quadrature.

void ChElementBeamIGA::ComputeNF(const double U,
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

/// Initial setup. Precompute mass and matrices that do not change during the simulation. In particular, compute the
/// arc-length parametrization.

void ChElementBeamIGA::SetupInitial(ChSystem* system) {
    assert(section);

    if (this->section->GetPlasticity()) {
        this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data_old);
        this->section->GetPlasticity()->CreatePlasticityData(int_order_b, this->plastic_data);
    }

    this->length = 0;

    // get two values of absyssa at extreme of span
    double u1 = knots(order);
    double u2 = knots(knots.size() - order - 1);

    double c1 = (u2 - u1) / 2;
    double c2 = (u2 + u1) / 2;

    // Gauss points for "s" shear components:

    this->Jacobian_s.resize(int_order_s);

    for (int ig = 0; ig < int_order_s; ++ig) {
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_s - 1][ig];
        // absyssa in span range:
        double u = (c1 * eta + c2);

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChMatrixDynamic<> N(2, (int)nodes.size());  // row n.0 contains N, row n.1 contains dN/du

        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(this->order, nspan, u, knots,
                                                          N);  ///< here return N and dN/du

        // compute reference spline gradient \dot{dr_0} = dr0/du
        ChVector<> dr0;
        for (int i = 0; i < nodes.size(); ++i) {
            dr0 += nodes[i]->GetX0ref().coord.pos * N(1, i);
        }
        this->Jacobian_s[ig] = dr0.Length();  // J = |dr0/du|
    }

    // Gauss points for "b" bend components:

    this->Jacobian_b.resize(int_order_b);
    this->strain_e_0.resize(int_order_b);
    this->strain_k_0.resize(int_order_b);
    this->stress_m.resize(int_order_b);
    this->stress_n.resize(int_order_b);
    this->strain_e.resize(int_order_b);
    this->strain_k.resize(int_order_b);

    for (int ig = 0; ig < int_order_b; ++ig) {
        // absyssa in typical -1,+1 range:
        double eta = ChQuadrature::GetStaticTables()->Lroots[int_order_b - 1][ig];
        // absyssa in span range:
        double u = (c1 * eta + c2);
        // scaling = gauss weight * change of range:
        double w = ChQuadrature::GetStaticTables()->Weight[int_order_b - 1][ig];

        // compute the basis functions N(u) at given u:
        int nspan = order;

        ChMatrixDynamic<> N(2, (int)nodes.size());  // row n.0 contains N, row n.1 contains dN/du

        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(this->order, nspan, u, knots,
                                                          N);  ///< here return N and dN/du

        // compute reference spline gradient \dot{dr_0} = dr0/du
        ChVector<> dr0;
        for (int i = 0; i < nodes.size(); ++i) {
            dr0 += nodes[i]->GetX0ref().coord.pos * N(1, i);
        }
        this->Jacobian_b[ig] = dr0.Length();  // J = |dr0/du|

        // From now on, compute initial strains as in ComputeInternalForces

        // Jacobian Jsu = ds/du
        double Jsu = this->Jacobian_b[ig];
        // Jacobian Jue = du/deta
        double Jue = c1;

        // interpolate rotation of section at given u, to compute R.
        // Note: this is approximate.
        // A more precise method: use quaternion splines, as in Kim,Kim and Shin, 1995 paper.
        ChQuaternion<> q_delta;
        ChVector<> da = VNULL;
        ChVector<> delta_rot_dir;
        double delta_rot_angle;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i = nodes[i]->GetX0ref().GetRot();  // state_x.ClipQuaternion(i * 7 + 3, 0);
            q_delta = nodes[0]->GetX0ref().GetRot().GetConjugate() * q_i;
            q_delta.Q_to_AngAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * N(0, i);  // a = N_i*a_i
        }
        ChQuaternion<> qda;
        qda.Q_from_Rotv(da);
        ChQuaternion<> qR = nodes[0]->GetX0().GetRot() * qda;

        // compute the 3x3 rotation matrix R equivalent to quaternion above
        ChMatrix33<> R(qR);

        // compute abs. spline gradient r'  = dr/ds
        ChVector<> dr;
        for (int i = 0; i < nodes.size(); ++i) {
            ChVector<> r_i = nodes[i]->GetX0().GetPos();
            dr += r_i * N(1, i);  // dr/du = N_i'*r_i
        }
        // (note r'= dr/ds = dr/du du/ds = dr/du * 1/Jsu   where Jsu computed in SetupInitial)
        dr *= 1.0 / Jsu;

        // compute abs spline rotation gradient q' = dq/ds
        // But.. easier to compute local gradient of spin vector a' = da/ds
        da = VNULL;
        for (int i = 0; i < nodes.size(); ++i) {
            ChQuaternion<> q_i = nodes[i]->GetX0ref().GetRot();
            q_delta = qR.GetConjugate() * q_i;
            q_delta.Q_to_AngAxis(delta_rot_angle,
                                 delta_rot_dir);  // a_i = dir_i*angle_i (in spline local reference, -PI..+PI)
            da += delta_rot_dir * delta_rot_angle * N(1, i);  // da/du = N_i'*a_i
        }
        // (note a= da/ds = da/du du/ds = da/du * 1/Jsu   where Jsu computed in SetupInitial)
        da *= 1.0 / Jsu;

        // compute local epsilon strain:  strain_e= R^t * r' - {1, 0, 0}
        this->strain_e_0[ig] = R.transpose() * dr - VECT_X;

        // compute local curvature strain:  strain_k= 2*[F(q*)(+)] * q' = 2*[F(q*)(+)] * N_i'*q_i = R^t * a' =
        // a'_local
        this->strain_k_0[ig] = da;

        // as a byproduct, also compute length
        this->length += w * Jsu * Jue;
    }

    // as a byproduct, also compute total mass
    this->mass = this->length * this->section->GetInertia()->GetMassPerUnitLength();
}

}  // end namespace fea
}  // end namespace chrono
