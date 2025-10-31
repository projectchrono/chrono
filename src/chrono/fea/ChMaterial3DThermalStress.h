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

    std::shared_ptr<ChMaterial3DStress>  material_stress;
    
    std::shared_ptr<ChMaterial3DThermal> material_thermal;

    double thermal_expansion_coefficient = 0;
    
    /// Set density of material. Both for stress problem and for thermal problem.
    virtual void SetDensity(double md) override { 
        this->ChMaterial3DDensity::SetDensity(md);
        this->material_stress->SetDensity(md);
        this->material_thermal->SetDensity(md);
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

};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
