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

#ifndef CHLOADERHEATFLUX_H
#define CHLOADERHEATFLUX_H


#include "chrono/fea/ChField.h"
#include "chrono/fea/ChFieldElement.h"
#include "chrono/physics/ChLoaderUV.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Thermal load: forced heat flux through a boundary surface.
/// Use this for applying a heat flux load on the surface of finite elements, as a
/// scalar flux for thermal analysis. Positive is ingoing flux.

class ChApi ChLoaderHeatFlux : public ChLoaderUVdistributed {
public:
    ChLoaderHeatFlux(std::shared_ptr<ChLoadableUV> mloadable)
        : ChLoaderUVdistributed(mloadable), m_heat_flux(0), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
        double V,              ///< parametric coordinate in surface
        ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override;

    /// Set the heat flux applied to UV surface, as [W/m^2]
    void SetSurfaceHeatFlux(double heat_flux) { m_heat_flux = heat_flux; }

    /// Get the heat flux applied to UV surface, as [W/m^2]
    double GetSurfaceHeatFlux() { return m_heat_flux; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

private:
    double m_heat_flux;
    int num_integration_points;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
