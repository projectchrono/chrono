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

#ifndef CHDOMAINTHERMOELASTIC_H
#define CHDOMAINTHERMOELASTIC_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChMaterial3DThermal.h"
#include "chrono/fea/ChMaterial3DStressStVenant.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Domain for FEA nonlinear finite strain with thermal coupling. It is based on a vector field,
/// (ChFieldDisplacement3D, that is used to store x, the absolute spatial position of nodes, NOT
/// the displacement from the material reference position, d=x-X, as in some software) and
/// a ChFieldTemperature. It solves the transient Poisson thermal equation together with 
/// structural dynamics. 
/// ***TODO***

class ChDomainThermoElastic : public ChDomainImpl<
    std::tuple<ChFieldTemperature, ChFieldDisplacement3D>, 
    ChFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChDomainThermoElastic(std::shared_ptr<ChFieldTemperature> mthermalfield, std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : ChDomainImpl({ mthermalfield, melasticfield })
    {
        // attach  default materials to simplify user side
        material_thermal = chrono_types::make_shared<ChMaterial3DThermal>();
        material_elasticity = chrono_types::make_shared<ChMaterial3DStressStVenant>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermal> material_thermal;
    std::shared_ptr<ChMaterial3DStress>  material_elasticity;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                            DataPerElement& data,
                                            const int i_point,
                                            ChVector3d& eta,
                                            const double s,
                                            ChVectorDynamic<>& Fi
    ) override {
        assert(false); //***TODO*** to be implemented
    }

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
        DataPerElement& data,
        const int i_point,
        ChVector3d& eta,
        ChMatrixRef H,
        double Kpfactor,
        double Rpfactor = 0,
        double Mpfactor = 0
    ) override {
            assert(false); //***TODO*** to be implemented
    }

};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
