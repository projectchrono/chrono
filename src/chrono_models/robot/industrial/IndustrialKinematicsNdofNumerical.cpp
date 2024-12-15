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

static const double FD_STEP = 1e-7;  // forward differentiation stepsize

ChQuaternion<> SLERP(const ChQuaternion<>& qa, const ChQuaternion<>& qb, double t) {
    ChQuaternion<> qdelta = qa.GetConjugate() * qb;
    return qa * QuatFromRotVec(qdelta.GetRotVec() * t);
};

IndustrialKinematicsNdofNumerical::IndustrialKinematicsNdofNumerical(const std::vector<ChCoordsysd>& joints_abs_coord,
                                                                     double tol,
                                                                     unsigned int max_iter,
                                                                     bool dosubsteps,
                                                                     unsigned int num_substeps)
    : m_tol(tol), m_iter(0), m_max_iter(max_iter), m_dosubsteps(dosubsteps), m_num_substeps(num_substeps) {
    // Neglect first csys because it represents robot base in world frame
    m_num_joints = static_cast<unsigned int>(joints_abs_coord.size()) - 1u;

    m_HH.setIdentity(m_num_joints, m_num_joints);
    m_HH *= FD_STEP;
    m_residual.resize(6);
    m_J.setZero(6, m_num_joints);  // 6xNj because residual=[Dtransl; Dquat123]

    SetupCoords(joints_abs_coord);
}

void IndustrialKinematicsNdofNumerical::SetupCoords(const std::vector<ChCoordsysd>& joints_abs_coord) {
    m_joints_abs_coord = joints_abs_coord;
    m_joints_rel_coord = joints_abs_coord;
    for (unsigned int i = 1; i < m_num_joints + 1; ++i)
        m_joints_rel_coord[i] = m_joints_abs_coord[i - 1].TransformParentToLocal(m_joints_abs_coord[i]);
}

void IndustrialKinematicsNdofNumerical::SetupIK(double tol,
                                                int max_iter,
                                                bool do_substeps,
                                                int num_substeps,
                                                double h) {
    m_tol = tol;
    m_max_iter = max_iter;
    m_dosubsteps = do_substeps;
    m_num_substeps = num_substeps;
}

ChCoordsysd IndustrialKinematicsNdofNumerical::GetFK(const ChVectorDynamic<>& u, unsigned int Nth) {
    ChCoordsysd coord = m_joints_rel_coord[0];  // Xw0
    for (unsigned int i = 1; i < Nth + 1; ++i)
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
    for (unsigned int i = 0; i < m_num_joints; ++i)
        m_J.col(i) = (GetResidualIK(u + m_HH.col(i)) - GetResidualIK(u)) / FD_STEP;
}

bool IndustrialKinematicsNdofNumerical::SolveNewtonRaphson(const ChVectorDynamic<>& u0) {
    m_uik = u0;  // initialize solution
    m_iter = 0;  // reset m_iter counter

    while (GetResidualIK(m_uik).norm() > m_tol && m_iter < m_max_iter) {
        NumJacobian(m_uik);                                      // update numeric jacobian
        m_du = m_J.householderQr().solve(GetResidualIK(m_uik));  // solve for increment
        //// Least squares for under/over-determined systems
        // m_du = m_J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(GetResidualIK(m_uik));
        //// LU decomposition with complete pivoting
        // m_du = m_J.fullPivLu().solve(GetResidualIK(m_uik));
        m_uik = m_uik - m_du;                                                    // update solution
        m_uik = m_uik.unaryExpr([](double x) { return ChWrapAngle(x, true); });  // wrap to [-PI, PI]
        ++m_iter;                                                                // update iteration counter
    }

    return m_iter != m_max_iter;  // converged : not converged
}

ChVectorDynamic<> IndustrialKinematicsNdofNumerical::GetIK(const ChCoordsysd& targetcoord,
                                                           const ChVectorDynamic<>& u0) {
    m_targetcoord = targetcoord;              // initialize target coordinates
    bool converged = SolveNewtonRaphson(u0);  // solve target (direct attempt)

    if (converged)
        return m_uik;  // converged: exit IK and return solution
    else {
        if (!m_dosubsteps) {
            if (m_verbose)
                std::cout << "WARNING: max number of IK iterations reached, revert to initial solution\n";

            m_uik = u0;    // revert solution to initial condition
            return m_uik;  // not converged: exit IK and stop motion
        } else {
            if (m_verbose)
                std::cout << "WARNING: max number of IK iterations reached, fallback to substeps method\n";

            m_uik = u0;                                        // re-initialize solution
            ChCoordsysd startcoord = GetFK(u0, m_num_joints);  // first substep coordinates

            double p = 1.0 / static_cast<double>(m_num_substeps);  // interp parameter
            for (unsigned int i = 0; i < m_num_substeps; ++i) {
                m_targetcoord.pos = (1.0 - p) * startcoord.pos + p * targetcoord.pos;  // linear interp
                m_targetcoord.rot = SLERP(startcoord.rot, targetcoord.rot, p);         // slerp

                // Solve intermediate targets with progressive warm-start
                bool sub_converged = SolveNewtonRaphson(m_uik);

                if (!sub_converged) {
                    if (m_verbose)
                        std::cout << "WARNING: failed IK substeps method, revert to initial solution\n";
                    m_uik = u0;    // revert solution to initial condition
                    return m_uik;  // not converged: exit IK and stop motion
                }

                p += 1.0 / static_cast<double>(m_num_substeps);
            }
            return m_uik;  // converged: exit IK and return solution
        }
    }
}

}  // end namespace industrial
}  // end namespace chrono