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

#ifndef CHMATERIAL3DHYPERELASTIC_H
#define CHMATERIAL3DHYPERELASTIC_H

#include "chrono/fea/ChMaterial3DStress.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for hyperelastic materials, that is materials with a constitutive equation 
/// of the type  P = f(E) , where E is the Green Lagrange strain tensor, and P is the
/// 2nd Piola-Kirchhoff strain. Or in general, materials for which the stress–strain 
/// relationship derives from a strain energy density function.

class ChMaterial3DHyperelastic : public ChMaterial3DStress {
public:
    ChMaterial3DHyperelastic() {}

    virtual ~ChMaterial3DHyperelastic() {}

    /// Implement interface to lower level stress material
    virtual void ComputeStress(ChStressTensor<>& P_stress, const ChMatrix33d& F) override {

        // Green Lagrange    E = 1/2( F*F' - I)
        ChMatrix33d E_strain33 = 0.5 * (F * F.transpose() - ChMatrix33d(1));

        ChStrainTensor<> E_strain; // Green Lagrange in Voigt notation
        E_strain.ConvertFromMatrix(E_strain33);
        E_strain.XY() *= 2; E_strain.XZ() *= 2; E_strain.YZ() *= 2;
        
        // Use the hyperelastic stress computation
        ComputeElasticStress(P_stress, E_strain);
    };

    /// Implement interface to lower level stress material
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, const ChMatrix33d& F) override {
        
        // Green Lagrange    E = 1/2( F*F' - I)
        ChMatrix33d E_strain33 = 0.5 * (F * F.transpose() - ChMatrix33d(1));

        ChStrainTensor<> E_strain; // Green Lagrange in Voigt notation
        E_strain.ConvertFromMatrix(E_strain33);
        E_strain.XY() *= 2; E_strain.XZ() *= 2; E_strain.YZ() *= 2;

        // Use the hyperelastic tangent modulus computation
        ComputeElasticTangentModulus(StressStrainMatrix, E_strain);
    };


    /// Compute elastic stress from elastic strain.  Assuming the 2* factor in the last 3 values of strain Voigt notation.
    /// Assuming strain is Green-Lagrange tensor E, in Voigt notation. For small strains it coincides with espilon tensor.
    /// Assuming stress if Piola-Kirchhoff tensor S, in Voigt notation. 
    /// Children materials MUST implement this.
    virtual void ComputeElasticStress(ChStressTensor<>& S_stress, const ChStrainTensor<>& E_strain) = 0;

    /// Computes the tangent modulus for a given strain. For linear elasticity it is the constant E matrix.
    /// Assuming strain is Green-Lagrange tensor, in Voigt notation. For small strains it coincides with espilon tensor.
    /// Children materials MUST implement this.
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, const ChStrainTensor<>& strain) = 0;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
