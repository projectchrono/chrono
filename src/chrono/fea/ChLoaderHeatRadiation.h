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

#ifndef CHLOADERHEATRADIATION_H
#define CHLOADERHEATRADIATION_H


#include "chrono/fea/ChField.h"
#include "chrono/fea/ChFieldElement.h"
#include "chrono/physics/ChLoaderUV.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Thermal load: radiation from surface.
/// Use this for applying a heat flux on the surface of finite elements, 
/// according to the Boltzmann law. If temperature is high enough, this can cause
/// outgoing heat flux as radiating in outer space. Using this, one can simulate
/// the black body radiation, etc. Negligible for low temperatures.
/// Beware of the high nonlinearity because of the 4th power in the temperature.

class ChApi ChLoaderHeatRadiation : public ChLoaderUVdistributed {
public:
    ChLoaderHeatRadiation(std::shared_ptr<ChLoadableUV> mloadable, std::shared_ptr<ChFieldTemperature> temp_field)
        : ChLoaderUVdistributed(mloadable), m_temp(temp_field), m_emissivity(1), m_T_env(0), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
        double V,              ///< parametric coordinate in surface
        ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override;

    /// Set the emissivity e in  W = e * sigma * T^4  Boltzmann law. 
    /// Typically a value in  0...1 range, with 1 as black body
    void SetSurfaceEmissivity(double emissivity) { m_emissivity = emissivity; }

    /// Get the emissivity e in  W = e * sigma * T^4  Boltzmann law 
    double GetSurfaceEmissivity() { return m_emissivity; }

    /// Set the imposed environment temperature at the external of the 
    /// surface (the smaller the delta with the FE temperature, the less radiation). 
    void SetEnvironmentTemperature(double T_env) { m_T_env = T_env; }

    /// Get the imposed environment temperature at the external of the 
    /// surface (the smaller the delta with the FE temperature, the less radiation). 
    double GetEnvironmentTemperature() { return m_T_env; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

private:
    double m_emissivity;
    double m_T_env;
    int num_integration_points;
    std::shared_ptr<ChFieldTemperature> m_temp;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
