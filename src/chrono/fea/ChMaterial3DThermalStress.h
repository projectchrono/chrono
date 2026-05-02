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

#ifndef CHMATERIAL3DTHERMALSTRESS_H
#define CHMATERIAL3DTHERMALSTRESS_H

#include "chrono/fea/ChMaterial3DStress.h"
#include "chrono/fea/ChMaterial3DThermal.h"


namespace chrono {
namespace fea {

// Forward:
class ChFieldData;


/// @addtogroup chrono_fea
/// @{


/// Material for coupled thermal and stress problems, ex thermoelasticity. This is 
/// implemented here as a composition of two distinct materials, one for the 3D heat 
/// problem, which is independent on stress state and which we recycle from the usual
/// ChMaterial3DThermal, and a one for the 3D stress problem, which could be one of the
/// many ChMaterial3DStress classes, ex. StVenant or Mooney-Rivlin or Ogden etc.

class ChMaterial3DThermalStress : public ChMaterial3DDensity {
public:

    ChMaterial3DThermalStress() {}

    virtual ~ChMaterial3DThermalStress() {}

    /// Material for stress field computation
    std::shared_ptr<ChMaterial3DStress>  material_stress;

    /// Material for thermal field computation
    std::shared_ptr<ChMaterial3DThermal> material_thermal;


    /// Set thermal expansion coefficient [1/K].  Ex. aluminium (23*10e-6)/K, steel (11*10e-6)/K
    void SetThermalExpansionCoefficient(double mte) { thermal_expansion_coefficient = mte; }

    /// Get thermal expansion coefficient [1/K]
    double GetThermalExpansionCoefficient() const { return thermal_expansion_coefficient; }
    

    /// Set rest temperature [K]. No thermal expanson at point, if point temperature is this.
    void SetRestTemperature(double mt) { rest_temperature = mt; }

    /// Get rest temperature [K]. No thermal expanson at point, if point temperature is this.
    double GetRestTemperature() const { return rest_temperature; }


    /// Set density of material. Both for stress problem and for thermal problem.
    virtual void SetDensity(double md) override { 
        this->ChMaterial3DDensity::SetDensity(md);
        this->material_stress->SetDensity(md);
        this->material_thermal->SetDensity(md);
    }

    /*
    // NOTE: WE COULD PROVIDE A CONSTITUTIVE LAW  ComputeStressAndFlux  FOR THIS COMPOUND MATERIAL,
    // COMPUTING AT ONCE THE EFFECTS OF THERMAL AND DISPLACEMENT FIELD, BUT FOR SIMPLICITY AND 
    // POSSIBLE SPEED OPTIMIZATION WE ASSUME IT IS UP TO THE ChDomainThermoDeformation THAT USES THIS COMPOUND MATERIAL
    // TO CALL SEPARATELY THE ChMaterial3DStress::ComputeStress() AND ChMaterial3DThermal::ComputeHeatFlux()

    /// Compute {heat flux, stress} from {temperature, deformation}.
    /// Assuming stress is Piola-Kirchhoff tensor S, in Voigt notation. Assuming finite strain passed via 
    /// a deformation gradient tensor F.

    virtual void ComputeStressAndFlux(ChStressTensor<>& S_stress_temp,      ///< output A: the PK stress
        const ChMatrix33d& F_def,           ///< current deformation gradient tensor F
        const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
        ChVector3d& q_flux,                 ///< output B: heat flux
        const ChVector3d T_grad             ///< current temperature gradient
        const double T,                     ///< current temperature (some materials might have properties changing with temperature)
        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;

    /// Computes the tangent modulus for a given deformation F_def and temperature. The lower right 6x6 block is about
    /// the tangent modulus for stress-strain relation (assuming PK2 stress and Green-Lagrange strain), the upper left 3x3
    /// block is about the tangent modulus for heat problem) 

    virtual void ComputeTangentModulus(ChMatrixNM<double, 9, 9>& tangentModulus, ///< output C tangent modulus
        const ChMatrix33d& F_def,       ///< current deformation gradient tensor F 
        const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
        const ChVector3d T_grad         ///< current temperature gradient
        const double T,                 ///< current temperature (some materials might have properties changing with temperature)
        ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) = 0;
    */
    
    /// Update your own auxiliary data, if any, at the end of time step (ex for plasticity).
    /// This is called at the end of every time step (or nl static step)
    virtual void ComputeUpdateEndStep(ChFieldData* data_per_point,          ///< pointer to auxiliary data (ex states), if any, per quadrature point
        ChElementData* data_per_element,      ///< pointer to auxiliary data (ex states), if any, per element 
        const double time
    ) {
        // default: redirect ComputeUpdateEndStep to the stress material
        material_stress->ComputeUpdateEndStep(data_per_point, data_per_element, time);
        // thermal_stress->ComputeUpdateEndStep(data_per_point, data_per_element, time); // never needed
    }
    

    /// Some material need info on the spatial velocity gradient  l=\nabla_x v ,
    /// where the time derivative of the deformation gradient F is  dF/dt = l*F.
    /// Some others, do not need this info. For optimization reason, then, the ChDomainXXYY 
    /// queries this, and knows if the "l" parameter could be left to null when calling ComputeStress(...)
    virtual bool IsSpatialVelocityGradientNeeded() const {
        return material_stress->IsSpatialVelocityGradientNeeded(); // no need to do "or" with needs of the material_thermal, that never needs spatial velocity gradient.
    }


    /// Implement this because the material might need custom data per material point. 
    /// Here the user can have plugged a material_stress of variuous type, so fall back to its custom data creation
    virtual std::unique_ptr<ChFieldData> CreateMaterialPointData() const override {
        return material_stress->CreateMaterialPointData();
        
        // Note that if also the material_thermal provides a custom data per material point (which we
        // know it never happens in our implementation anyway) then one could have returned a  
        //   auto mdata = std::make_unique<ChCompoundData>();
        //   mdata->stress_material_per_point_data  = material_stress->CreateMaterialPointData();
        //   mdata->thermal_material_per_point_data = material_thermal->CreateMaterialPointData();
        //   return mdata;
        // where we previously have declared something like:
        //  class ChCompoundData : public ChFieldData {
        //   public:
        //     std::unique_ptr<ChFieldData> stress_material_per_point_data;
        //     std::unique_ptr<ChFieldData> thermal_material_per_point_data;
        //  };
    }

private:
    double thermal_expansion_coefficient = 0;
    double rest_temperature = 0;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
