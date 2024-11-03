// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Class for numerical iterative solution of N-DOF articulated robot kinematics.
//
// =============================================================================

#include "IndustrialKinematicsNdofNumerical.h"

namespace chrono {
namespace industrial {

IndustrialKinematicsNdofNumerical::IndustrialKinematicsNdofNumerical(const std::vector<ChCoordsysd>& joints_abs_coord,
                                                               const double tol,
                                                               const int max_iter,
                                                               bool dosubsteps,
                                                               const int num_substeps)
    : m_tol(tol), m_iter(0), m_max_iter(max_iter), m_h(1e-7), m_dosubsteps(dosubsteps), m_num_substeps(num_substeps) {
    m_num_joints = static_cast<int>(joints_abs_coord.size()) - 1;  // remove first coord because it's robot base in world frame
    m_HH.setIdentity(m_num_joints, m_num_joints);
    m_HH *= m_h;
    m_residual.resize(6);
    m_J.setZero(6, m_num_joints);  // 6xNj because residual=[Dtransl; Dquat123]
    SetupCoords(joints_abs_coord);
}

IndustrialKinematicsNdofNumerical::IndustrialKinematicsNdofNumerical(const IndustrialKinematicsNdofNumerical& other)
    : m_joints_abs_coord(other.m_joints_abs_coord),
      m_joints_rel_coord(other.m_joints_rel_coord),
      m_tol(other.m_tol),
      m_iter(other.m_iter),
      m_max_iter(other.m_max_iter),
      m_h(other.m_h),
      m_dosubsteps(other.m_dosubsteps),
      m_num_substeps(other.m_num_substeps) {
    m_num_joints = other.m_num_joints;
    m_HH = other.m_HH;
    m_J = other.m_J;
}

void IndustrialKinematicsNdofNumerical::SetupCoords(const std::vector<ChCoordsysd>& joints_abs_coord) {
    m_joints_abs_coord = joints_abs_coord;
    m_joints_rel_coord = joints_abs_coord;
    for (int i = 1; i < m_num_joints + 1; ++i)
        m_joints_rel_coord[i] = m_joints_abs_coord[i - 1].TransformParentToLocal(m_joints_abs_coord[i]);
}

void IndustrialKinematicsNdofNumerical::SetupIK(double tol, int max_iter, bool do_substeps, int num_substeps, double h) {
    m_tol = tol;
    m_max_iter = max_iter;
    m_dosubsteps = do_substeps;
    m_num_substeps = num_substeps;
    m_h = h;
}

ChCoordsysd IndustrialKinematicsNdofNumerical::GetFK(const ChVectorDynamic<>& u, int N) {
    ChCoordsysd coord = m_joints_rel_coord[0];  // Xw0
    for (auto i = 1; i < N + 1; ++i)
        coord = coord * (QuatFromAngleZ(u[i - 1]) * m_joints_rel_coord[i]);  // up to XwN
    return coord;
}

ChCoordsysd IndustrialKinematicsNdofNumerical::GetFK(const ChVectorDynamic<>& u) {
    return GetFK(u, m_num_joints);  // up to TCP
}

ChVectorDynamic<> IndustrialKinematicsNdofNumerical::GetResidualIK(const ChVectorDynamic<>& u) {
    ChCoordsysd ucoord = GetFK(u);                                           // FK at given u
    ChVector3d t_delta = ucoord.pos - m_targetcoord.pos;                     // translation residual
    ChQuaternion<> q_delta = ucoord.rot.GetConjugate() * m_targetcoord.rot;  // rotation residual
    m_residual << t_delta.x(), t_delta.y(), t_delta.z(), q_delta.e1(), q_delta.e2(), q_delta.e3();  // 6x1
    return m_residual;
}

void IndustrialKinematicsNdofNumerical::NumJacobian(const ChVectorDynamic<>& u) {
    for (int i = 0; i < m_num_joints; ++i)
        m_J.col(i) = (GetResidualIK(u + m_HH.col(i)) - GetResidualIK(u)) / m_h;
}

bool IndustrialKinematicsNdofNumerical::SolveNewtonRaphson(const ChVectorDynamic<>& u0) {
    m_uik = u0;  // initialize solution
    m_iter = 0;  // reset m_iter counter

    while (GetResidualIK(m_uik).norm() > m_tol && m_iter < m_max_iter) {
        NumJacobian(m_uik);                                      // update numeric jacobian
        m_du = m_J.householderQr().solve(GetResidualIK(m_uik));  // solve for increment
        // m_du = m_J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(GetResidualIK(m_uik)); // least squares
        // for under/over-determined systems m_du = m_J.bdcSvd(Eigen::ComputeThinU |
        // Eigen::ComputeThinV).solve(GetResidualIK(m_uik)); m_du = m_J.fullPivLu().solve(GetResidualIK(m_uik)); // LU
        // decomposition with complete pivoting
        m_uik = m_uik - m_du;                                                                    // update solution
        m_uik = m_uik.unaryExpr([](double x) { return std::fmod(x + CH_PI, CH_2PI) - CH_PI; });  // wrap to [-pi, pi]
        ++m_iter;  // update iteration counter
    }

    if (m_iter == m_max_iter)
        return false;  // not converged
    else
        return true;  // converged
}

ChVectorDynamic<> IndustrialKinematicsNdofNumerical::GetIK(const ChCoordsysd& targetcoord, const ChVectorDynamic<>& u0) {
    m_targetcoord = targetcoord;              // initialize target coordinates
    bool converged = SolveNewtonRaphson(u0);  // solve target (direct attempt)

    if (converged)
        return m_uik;  // converged: exit IK and return solution
    else {
        if (!m_dosubsteps) {
            std::cout << "WARNING: max number of IK iterations reached. Motion stopped. \n";
            m_uik = u0;    // revert solution to initial condition
            return m_uik;  // not converged: exit IK and stop motion
        } else {
            std::cout << "WARNING: max number of IK iterations reached. Fallback to substeps method. \n";
            m_uik = u0;                                        // re-initialize solution
            ChCoordsysd startcoord = GetFK(u0, m_num_joints);  // first substep coordinates
            auto substeps = ChVectorDynamic<>::LinSpaced(m_num_substeps, 1.0 / static_cast<double>(m_num_substeps),
                                                         1.0);  // size, min, max

            for (int i = 0; i < substeps.size(); ++i) {
                double p = substeps[i];                                                    // interp parameter
                auto intermediate_pos = targetcoord.pos * p + startcoord.pos * (1.0 - p);  // linear interp
                double tmp_theta = std::acos(startcoord.rot.Dot(targetcoord.rot));
                ChQuaternion<> intermediate_rot =
                    startcoord.rot * (std::sin((1 - p) * tmp_theta) / std::sin(tmp_theta)) +
                    targetcoord.rot * (std::sin(p * tmp_theta) / std::sin(tmp_theta));  // slerp
                m_targetcoord = ChCoordsysd(intermediate_pos, intermediate_rot);        // intermediate target
                bool sub_converged =
                    SolveNewtonRaphson(m_uik);  // solve intermediate targets with progressive warm-start

                if (!sub_converged) {
                    std::cout << "WARNING: Failed IK substeps method. Motion stopped. \n";
                    m_uik = u0;    // revert solution to initial condition
                    return m_uik;  // not converged: exit IKand stop motion
                }
            }
            return m_uik;  // converged: exit IK and return solution
        }
    }
}

}  // end namespace industrial
}  // end namespace chrono