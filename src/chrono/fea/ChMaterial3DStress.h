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

#ifndef CHMATERIAL3DSTRESS_H
#define CHMATERIAL3DSTRESS_H

#include "chrono/fea/ChMaterial3DDensity.h"
#include "chrono/core/ChTensors.h"


namespace chrono {
namespace fea {

// Forward:
class ChFieldDisplacement3D;
class ChFieldDataNONE;
class ChFeaPerElementDataKRM;


/// @addtogroup chrono_fea
/// @{


/// Class for the basic properties of materials in 3D continua where stress 
/// can be computed from some deformation measure and some state. 
/// TODO: provide a more general interface, now it is like in the subcase of the hyperelastic
/// materials.

class ChMaterial3DStress : public ChMaterial3DDensity {
public:
    //using T_per_node = std::tuple<ChFieldDisplacement3D>;
    using T_per_materialpoint = ChFieldDataNONE;
    using T_per_element = ChFeaPerElementDataKRM;

    ChMaterial3DStress() {}

    virtual ~ChMaterial3DStress() {}

    /// Compute elastic stress from finite strain, passed as 3x3 deformation gradient tensor F_def.
    /// Assuming stress if Piola-Kirchhoff tensor S, in Voigt notation.
    /// TODO: provide a more general interface with more inputs, ex. strain rate, states etc.,
    /// now it is a bit like hyperelastic materials.
    virtual void ComputeStress(ChStressTensor<>& S_stress,          ///< output stress, PK2
                                const ChMatrix33d& F_def,           ///< current deformation gradient tensor
                                T_per_materialpoint* data_per_point,///< pointer to auxiliary data (ex states), if any, per quadrature point
                                T_per_element* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element point
    ) = 0;

    /// Computes the tangent modulus for a given strain, assuming it
    /// between delta of 2nd Piola-Kirchhoff S and delta of Green Lagrange E, both in Voigt notation.
    /// Assuming current strain is a 3x3 deformation gradient tensor F_def. 
    /// TODO: provide a more general interface with more inputs, ex. strain rate, states etc.,
    /// now it is a bit like hyperelastic materials.
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, ///< output C tangent modulus, as dS=C*dE
                                const ChMatrix33d& F_def,           ///< current deformation gradient tensor
                                T_per_materialpoint* data_per_point,///< pointer to auxiliary data (ex states), if any, per quadrature point
                                T_per_element* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element point
    ) = 0;

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
