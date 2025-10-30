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

#ifndef CHDOMAINTHERMODEFORMATION_H
#define CHDOMAINTHERMODEFORMATION_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChMaterial3DThermalStress.h"
#include "chrono/fea/ChMaterial3DStressStVenant.h"
#include "chrono/fea/ChVisualDataExtractor.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Auxiliary scratch data stored per each material point during the ChDomainThermoDeformation
/// computation. This can be plotted in postprocessing, etc.

class ChFieldDataAuxiliaryThermoDeformation : public ChFieldDataNONE {
public:
    ChMatrix33d F;      /// deformation gradient tensor - can be used to plot Green-Lagrange etc. 
    ChMatrix33d F_t;    /// thermal deformation gradient tensor
    ChMatrix33d F_e;    /// elastic deformation gradient tensor
    ChVector3d  q_flux; /// heat flux
    
    // by the way this could have been also: 
    // ChFieldDataAuxiliaryThermal aux_thermal;      // with q_flux
    // ChFieldDataAuxiliaryDeformation aux_thermal;  // with F
    // ChMatrix33d F_t; 
    // ChMatrix33d F_e;
};

/// Domain for FEA nonlinear finite strain with thermal coupling. It is based on a vector field,
/// (ChFieldDisplacement3D, that is used to store x, the absolute spatial position of nodes, NOT
/// the displacement from the material reference position, d=x-X, as in some software) and
/// a ChFieldTemperature. It solves the transient Poisson thermal equation together with 
/// structural dynamics. 
/// ***TODO*** implement

class ChDomainThermoDeformation : public ChDomainImpl<
    std::tuple<ChFieldTemperature, ChFieldDisplacement3D>, 
    ChFieldDataAuxiliaryThermoDeformation,
    ChElementDataKRM> {
public:
    ChDomainThermoDeformation(std::shared_ptr<ChFieldTemperature> mthermalfield, std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : ChDomainImpl({ mthermalfield, melasticfield })
    {
        // attach  default materials to simplify user side
        material_thermalstress = chrono_types::make_shared<ChMaterial3DThermalStress>();
        material_thermalstress->material_stress  = chrono_types::make_shared<ChMaterial3DStressStVenant>();
        material_thermalstress->material_thermal = chrono_types::make_shared<ChMaterial3DThermal>();
    }

    /// Thermal properties of this domain (conductivity, 
    /// heat capacity constants etc.) 
    std::shared_ptr<ChMaterial3DThermalStress> material_thermalstress;

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

    /// Invoked at the end of each time step. If the material has some 
    /// custom handling of auxiliary data (states etc.) it will manage this in ComputeUpdateEndStep()
    virtual void PointUpdateEndStep(std::shared_ptr<ChFieldElement> melement,
        DataPerElement& data,
        const int i_point, 
        const double time
    ) override {
            assert(false); //***TODO*** to be implemented
    }



protected:
    /// Get the material of the domain. Called when creating auxiliary data per material point.
    virtual std::shared_ptr<ChMaterial> GetMaterial() override {
        return material_thermalstress;
    };


};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
