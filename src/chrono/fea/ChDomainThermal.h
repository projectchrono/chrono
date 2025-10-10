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

#ifndef CHDOMAINTHERMAL_H
#define CHDOMAINTHERMAL_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChMaterial3DThermal.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Domain for FEA thermal analysis. It is based on a scalar temperature field,
/// that is ChFieldTemperature.
/// Material properties are defined via a ChMaterial3DThermal.
/// In case you need thermoelasticity, use ChDomainThermoelastic.

class ChDomainThermal : public ChDomainImpl<
    std::tuple<ChFieldTemperature>,
    ChFieldDataNONE,
    ChFeaPerElementDataKRM> {
public:
    ChDomainThermal(std::shared_ptr<ChFieldTemperature> mfield) 
        : ChDomainImpl(mfield)
    {
        // attach a default material to simplify user side
        material = chrono_types::make_shared<ChMaterial3DThermal>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermal> material;
    
    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads( std::shared_ptr<ChFieldElement> melement, 
                                            DataPerElement& data, 
                                            const int i_point, 
                                            ChVector3d& eta, 
                                            const double s, 
                                            ChVectorDynamic<>& Fi
    ) override {
        ChVectorDynamic<> T;
        this->GetStateBlock(melement, T);
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);
        // B = dNdX // in the lucky case of thermal problem, no need to build B because B is simply dNdX

        // We have:  Fi = - K * T;
        // where      K = sum (dNdX' * k * dNdX * w * |J|)
        // so we compute  Fi += -(dNdX' * k * dNdX * T) * s
        Fi += -(dNdX.transpose() * this->material->GetConductivityMatrix() * dNdX) * T * s;
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
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);
        // B = dNdX // in the lucky case of thermal problem, no need to build B because B is simply dNdX

        // K  matrix (jacobian d/dT of:    c dT/dt + div [C] grad T = f )  
        // K = sum (dNdX' * k * dNdX * w * |J|)
        H += Kpfactor * (dNdX.transpose() * this->material->GetConductivityMatrix() * dNdX) ;

        // R  matrix : (jacobian d / d\dot(T) of:    c dT / dt + div[C] grad T = f)
        // R = sum (N' * c*rho * N * w * |J|)
        if (Rpfactor && this->material->GetSpecificHeatCapacity()) {
            H += (Rpfactor * this->material->GetSpecificHeatCapacity() * this->material->GetDensity()) * (N.transpose() * N);
        }
    }

    /// Invoked at the end of each time step. If the material has some 
    /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
    virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
        DataPerElement& data,
        const int i_point,
        const double time
    ) override {
        // DO NOTHING because we assume that thermal materials never implement some  material->ComputeUpdateEndStep(...) 
    }


};


/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
