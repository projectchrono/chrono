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

#ifndef INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H
#define INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H

#include "IndustrialKinematics.h"

namespace chrono {
namespace industrial {

/// @addtogroup robot_models_industrial
/// @{

class CH_MODELS_API IndustrialKinematicsNdofNumerical : public IndustrialKinematics {
  public:
    /// Default constructor.
    IndustrialKinematicsNdofNumerical() {}

    /// Build model from joint absolute coordinate, IK tolerance and IK max iterations.
    IndustrialKinematicsNdofNumerical(
        const std::vector<ChCoordsysd>& joints_abs_coord,  ///< joints starting absolute coordinates
        double tol = 1e-6,                                 ///< IK tolerance
        unsigned int max_iter = 10,                        ///< IK max iterations
        bool dosubsteps = false,  ///< if direct attempt to solve IK fails, subdivide residual in smaller substeps and
                                  ///< use them as progressive warm-start for N-R iterations (fallback)
        unsigned int num_substeps = 10  ///< number of substeps in fallback IK method
    );

    /// Virtual destructor.
    virtual ~IndustrialKinematicsNdofNumerical() {}

    /// Set absolute and relative robot joint coordinates.
    void SetupCoords(const std::vector<ChCoordsysd>& m_joints_abs_coord);

    /// Set IK data.
    void SetupIK(double tol, int max_iter, bool do_substeps, int num_substeps, double h);

    /// Get Forward Kinematics at given input u, up to Nth link.
    virtual ChCoordsysd GetFK(const ChVectorDynamic<>& u, unsigned int Nth);

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

    std::vector<ChCoordsysd> m_joints_abs_coord = {};  ///< joints starting absolute coordinates
    std::vector<ChCoordsysd> m_joints_rel_coord = {};  ///< joints starting relative coordinates
    double m_tol = 0;                                  ///< IK tolerance
    unsigned int m_iter = 0;                           ///< IK current iteration
    unsigned int m_max_iter = 0;                       ///< IK max iterations
    bool m_dosubsteps = false;        ///< perform fallback substepped IK if direct N-R does not converge
    unsigned int m_num_substeps = 0;  ///< number of fallback IK substeps
    ChMatrixDynamic<> m_HH;           ///< utility increment matrix for IK Jacobian computation
    ChMatrixDynamic<> m_J;            ///< numerical IK Jacobian matrix
    ChVectorDynamic<> m_residual;     ///< IK residual
    ChCoordsysd m_targetcoord;        ///< IK target coordinates
    ChVectorDynamic<> m_uik;          ///< IK solution
    ChVectorDynamic<> m_du;           ///< IK solution increment
};

/// @} robot_models_industrial

}  // end namespace industrial
}  // end namespace chrono

#endif  // end INDUSTRIAL_ROBOT_KINEMATICS_NDOF_NUMERICAL_H