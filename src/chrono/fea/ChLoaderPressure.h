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

#ifndef CHLOADERPRESSURE_H
#define CHLOADERPRESSURE_H


#include "chrono/physics/ChLoaderUV.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Constant pressure load, a 3D per-area force 
/// aligned with the surface normal.

class ChApi ChLoaderPressure : public ChLoaderUVdistributed {
public:
    ChLoaderPressure(std::shared_ptr<ChLoadableUV> mloadable)
        : ChLoaderUVdistributed(mloadable), is_stiff(false), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
        double V,              ///< parametric coordinate in surface
        ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
        ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
        ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
    ) override;

    void SetPressure(double mpressure) { pressure = mpressure; }
    double GetPressure() { return pressure; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

    void SetStiff(bool val) { is_stiff = val; }
    virtual bool IsStiff() override { return is_stiff; }

private:
    double pressure;
    bool is_stiff;
    int num_integration_points;
};



/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
