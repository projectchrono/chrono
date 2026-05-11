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

#ifndef CHLOADERGRAVITY_H
#define CHLOADERGRAVITY_H


#include "chrono/physics/ChLoaderUVW.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

/// Constant gravitational load, typically used for FEA elements

class ChApi ChLoaderGravity : public ChLoaderUVWdistributed {
public:
    ChLoaderGravity(std::shared_ptr<ChLoadableUVW> mloadable,  ///< the loadable item, ex a ChFeaTetrahedron4
                    double mdensity = -1                       ///< if >0, this density overrides mloadable->GetDensity() (defaults=-1, so uses mloadable->GetDensity() ) 
    )
        : ChLoaderUVWdistributed(mloadable), G_acc(0, -9.8, 0), num_int_points(1), override_loadable_density(mdensity) {};

    virtual void ComputeF(double U,                    ///< parametric coordinate in volume
        double V,                    ///< parametric coordinate in volume
        double W,                    ///< parametric coordinate in volume
        ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override;

    /// Set the number of integration points for gravity (assumed, for now, same number per direction).
    void SetNumIntPoints(int val) { num_int_points = val; }

    /// Get the number of integration points for gravity.
    int GetNumIntPoints() const { return num_int_points; }

    /// Set the G (gravity) acceleration vector affecting the loadable object.
    void SetGravitationalAcceleration(ChVector3d m_acc) { G_acc = m_acc; }

    /// Get the G (gravity) acceleration vector affecting the loadable object.
    ChVector3d GetGravitationalAcceleration() { return G_acc; }

    virtual int GetIntegrationPointsU() override { return num_int_points; }
    virtual int GetIntegrationPointsV() override { return num_int_points; }
    virtual int GetIntegrationPointsW() override { return num_int_points; }

private:
    ChVector3d G_acc;
    int num_int_points;
    double override_loadable_density;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
