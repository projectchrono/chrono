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

#ifndef CH_INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H
#define CH_INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H

#include "IndustrialKinematics.h"

namespace chrono {
namespace industrial {

class CH_MODELS_API IndustrialKinematicsNdofNumerical : public IndustrialKinematics {
  public:
    /// Default constructor.
    IndustrialKinematicsNdofNumerical(){};

    /// Build model from joint absolute coordinate, IK tolerance and IK max iterations.
    IndustrialKinematicsNdofNumerical(
        const std::vector<ChCoordsysd>& m_joints_abs_coord,  ///< joints starting absolute coordinates
        const double tol = 1e-6,                             ///< IK tolerance
        const int max_iter = 10,                             ///< IK max iterations
        bool dosubsteps = false,  ///< if direct attempt to solve IK fails, subdivide residual in smaller substeps and
                                  ///< use them as progressive warm-start for N-R iterations (fallback)
        const int num_substeps = 10  ///< number of substeps in fallback IK method
    );

    /// Copy constructor.
    IndustrialKinematicsNdofNumerical(const IndustrialKinematicsNdofNumerical& other);

    /// Virtual destructor.
    virtual ~IndustrialKinematicsNdofNumerical(){};

    /// Set absolute and relative robot joint coordinates.
    void SetupCoords(const std::vector<ChCoordsysd>& m_joints_abs_coord);

    /// Set IK data.
    void SetupIK(double tol, int max_iter, bool do_substeps, int num_substeps, double h);

    /// Get Forward Kinematics at given input u, up to Nth link.
    virtual ChCoordsysd GetFK(const ChVectorDynamic<>& u, int Nth);

    /// Get Forward Kinematics at given input u, up to TCP.
    ChCoordsysd GetFK(const ChVectorDynamic<>& u);

    /// Get Inverse Kinematics for given target coordinates, starting from given u
    ChVectorDynamic<> GetIK(const ChCoordsysd& targetcoord, const ChVectorDynamic<>& u0);

  private:
    /// Get residual for IK Newton-Raphson iteration
    ChVectorDynamic<> GetResidualIK(const ChVectorDynamic<>& u);

    /// Numerical computation of residual Jacobian matrix, around current u
    void NumJacobian(const ChVectorDynamic<>& u);

    // Solve u with Newton-Raphson iterations and return T/F if converged
    bool SolveNewtonRaphson(const ChVectorDynamic<>& u0);

    std::vector<ChCoordsysd> m_joints_abs_coord;  ///< joints starting absolute coordinates
    std::vector<ChCoordsysd> m_joints_rel_coord;  ///< joints starting relative coordinates
    double m_tol = 0;                             ///< IK tolerance
    int m_iter = 0, m_max_iter = 0;               ///< IK current iteration and max iterations
    double m_h = 0;                               ///< increment for IK Jacobian computation
    bool m_dosubsteps = false;                    ///< perform fallback substepped IK if direct N-R does not converge
    int m_num_substeps = 0;                       ///< number of fallback IK substeps
    ChMatrixDynamic<> m_HH;                       ///< utility increment matrix for IK Jacobian computation
    ChMatrixDynamic<> m_J;                        ///< numerical IK Jacobian matrix
    ChVectorDynamic<> m_residual;                 ///< IK residual
    ChCoordsysd m_targetcoord;                    ///< IK target coordinates
    ChVectorDynamic<> m_uik, m_du;                ///< IK solution and increment
};

}  // end namespace industrial
}  // end namespace chrono

#endif  // end CH_INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H