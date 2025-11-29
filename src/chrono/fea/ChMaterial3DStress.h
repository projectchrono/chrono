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
class ChFieldData;
class ChElementData;


/// @addtogroup chrono_fea
/// @{


/// Class for the basic properties of materials in 3D continua where stress 
/// can be computed from some deformation measure and some state. 

class ChMaterial3DStress : public ChMaterial3DDensity {
public:

    ChMaterial3DStress() {}

    virtual ~ChMaterial3DStress() {}


    /// Compute elastic stress from finite strain, passed as 3x3 deformation gradient tensor F_def.
    /// Assuming stress is 2nd Piola-Kirchhoff tensor "S_stress", in Voigt notation.
    /// Some materials could also make use of spatial velocity gradient "l", ex. for damping effects,
    /// and of the "data_per_point" auxiliary structure, that could contain states like in plasticity.
    
    virtual void ComputeStress(ChStressTensor<>& S_stress,          ///< output stress, PK2
                                const ChMatrix33d& F_def,           ///< current deformation gradient tensor
                                const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

    /// Computes the tangent modulus for a given strain, assuming it
    /// between delta of 2nd Piola-Kirchhoff S and delta of Green Lagrange E, both in Voigt notation.
    /// Assuming current strain is a 3x3 deformation gradient tensor F_def. 

    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, ///< output C tangent modulus, as dS=C*dE
                                const ChMatrix33d& F_def,       ///< current deformation gradient tensor
                                const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

    /// Update your own auxiliary data, if any, at the end of time step (ex for plasticity).
    /// This is called at the end of every time step (or nl static step)
    virtual void ComputeUpdateEndStep(ChFieldData* data_per_point,          ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                      ChElementData* data_per_element,      ///< pointer to auxiliary data (ex states), if any, per element 
                                      const double time
    ) {
         // default: do nothing. 
    }

    /// Some material need info on the spatial velocity gradient  l=\nabla_x v ,
    /// where the time derivative of the deformation gradient F is  dF/dt = l*F.
    /// Some others, do not need this info. For optimization reason, then, the ChDomainXXYY 
    /// queries this, and knows if the "l" parameter could be left to null when calling ComputeStress(...)
    virtual bool IsSpatialVelocityGradientNeeded() const = 0;

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
