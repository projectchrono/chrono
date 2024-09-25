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

#ifndef CHMODALDAMPING_H
#define CHMODALDAMPING_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/physics/ChAssembly.h"
#include <complex>

namespace chrono {
namespace modal {

/// @addtogroup modal
/// @{

// forward reference
class ChModalAssembly;

/// Base class for damping models of modal reduced assemblies.
/// Children classes provide specialized damping models.
/// These classes are used as arguments in ChModalAssembly::DoModalReduction(...) to define
/// which approach is used in order to build the damping matrix for the modal-reduced assembly.
class ChApiModal ChModalDamping {
  public:
    virtual ~ChModalDamping(){};

    // child class inherits this. They must compute the reduced R.
    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const = 0;
};

/// Class for no damping model
class ChApiModal ChModalDampingNone : public ChModalDamping {
  public:
    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const {
        modal_R.setZero(modal_M.rows(), modal_M.cols());
    }
};

/// Class for simple Rayleigh damping model
///     R^ = alpha*M^ + beta*K^
/// where M^ and K^ are the reduced matrices, both for boundary nodes and modal coords.
class ChApiModal ChModalDampingRayleigh : public ChModalDamping {
  public:
    /// Construct from the two coefficients in  R^ = alpha*M^ + beta*K^
    ChModalDampingRayleigh(double malpha, double mbeta) : alpha(malpha), beta(mbeta){};
    double alpha = 0;
    double beta = 0;

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const {
        modal_R = alpha * modal_M + beta * modal_K;
    }
};

/// Class for setting the damping via N damping factors z_i of the internal mode coordinates.
///  R^ = [Rbb Rbm ]
///       [Rmb Rmm ]
/// with Rmm=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i },  Rbb= 0,  Rbm = 0, Rmb = 0.
/// Note that for boundary nodes, zero damping is used.
class ChApiModal ChModalDampingFactorRmm : public ChModalDamping {
  public:
    /// Construct from the z_i in coefficients in  Rmm=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i }
    ChModalDampingFactorRmm(const ChVectorDynamic<>& factors) : damping_factors(factors){};

    ChVectorDynamic<> damping_factors;

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const;
};

/// Class for setting the damping via N damping factors z_i of the internal mode coordinates
/// and alpha-beta Rayleigh damping for the boundary nodes, assuming
///  R^ = [Rbb Rbm ]
///       [Rmb Rmm ]
/// with Rmm=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i },
/// Rbb= alpha*Mbb + beta*Kbb,  Rbm = 0, Rmb = 0.
class ChApiModal ChModalDampingFactorRayleigh : public ChModalDampingFactorRmm {
  public:
    /// Construct from the z_i in coefficients in  Rmm=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i }
    /// for modes, and from alpha-beta Rayleight coefficients for boundary nodes
    ChModalDampingFactorRayleigh(const ChVectorDynamic<>& factors, double malpha, double mbeta)
        : ChModalDampingFactorRmm(factors), alpha(malpha), beta(mbeta){};

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const;

    double alpha;
    double beta;
};

/// Class for setting the damping via N damping factors z_i for all the modes
/// of the subassembly, where assembly n.modes = (boundary coords+internal modes)
///  R^ = V'^-1 * Dd * V^-1
/// with Dd=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i },
/// and V = eigenvectors of (M^, K^). Note there can be more z_i than in the case of
/// ChModalDampingFactorRmm, ex. if one has 2 dynamic modes and 6 static modes, here you can have 8 z_i.
class ChApiModal ChModalDampingFactorAssembly : public ChModalDamping {
  public:
    /// Construct from the z_i in coefficients in  Rmm=diag { 2 z_1 w_1, 2 z_2 w_2, ..., 2 z_i w_i }
    /// for modes, and from alpha-beta Rayleight coefficients for boundary nodes
    ChModalDampingFactorAssembly(const ChVectorDynamic<>& factors) : damping_factors(factors) {}

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const;

    ChVectorDynamic<> damping_factors;
};

/// Class for damping as reduction of the original damping matrix via the eigenvectors of the undamped assembly,
/// i.e. the same eigenvectors used for reducing M and K as M^ = Psi'*M*Psi etc, as:
///    R^ = Psi'*R*Psi
/// where Psi contains static and real-valued eigenmodes of the undamped assembly. Simple but not always good.
class ChApiModal ChModalDampingReductionR : public ChModalDamping {
  public:
    /// Constructor for most cases, where you want to use the R matrix of the (not reduced) assembly.
    ChModalDampingReductionR(ChModalAssembly& massembly);

    /// Constructor for the case where you want to pass an R matrix from an external source: R=Rcustom.
    /// The Rcustom matrix must be of proper size, a square matrix of size (m_num_coords_vel_boundary +
    /// m_num_coords_vel_internal)
    ChModalDampingReductionR(ChSparseMatrix& Rcustom) { full_R = Rcustom; }

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const {
        modal_R = Psi.transpose() * full_R * Psi;
    }

    ChSparseMatrix full_R;
};

/// Class for damping defined with an user-defined matrix that could be obtained via external
/// tools such as Matlab or FEA. This is the most generic case.
///    R^ = Rcustom
/// where Rcustom is a square matrix of size (m_num_coords_vel_boundary + m_num_modes_coords_vel)
class ChApiModal ChModalDampingCustom : public ChModalDamping {
  public:
    /// Constructor where you pass a custom damping matrix Rcustom, related to the coordinates of the already reduced
    /// assembly. The Rcustom matrix must be of proper size, i.e. a square matrix of size (m_num_coords_vel_boundary +
    /// m_num_modes_coords_vel)
    ChModalDampingCustom(ChSparseMatrix& Rcustom) { reduced_R = Rcustom; }

    virtual void ComputeR(ChModalAssembly& assembly,
                          const ChMatrixDynamic<>& modal_M,
                          const ChMatrixDynamic<>& modal_K,
                          const ChMatrixDynamic<>& Psi,
                          ChMatrixDynamic<>& modal_R) const {
        modal_R = reduced_R;
    }

    ChSparseMatrix reduced_R;
};

/// @} modal

}  // end namespace modal
}  // end namespace chrono

#endif
