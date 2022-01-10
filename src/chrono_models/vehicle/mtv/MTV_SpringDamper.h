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
// Authors: Radu Serban
// =============================================================================
//
// Various utility classes for vehicle subsystems.
//
// =============================================================================

#ifndef MTV_SPRING_DAMPER_H
#define MTV_SPRING_DAMPER_H

#include "chrono/physics/ChLinkTSDA.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// MTV rear spring functor class - implements a nonlinear spring
class MTV_SpringForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    MTV_SpringForceRear(double spring_constant, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

/// MTV rear shock functor class - implements a nonlinear damper
class MTV_ShockForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    MTV_ShockForceRear(double compression_slope,
                       double compression_degressivity,
                       double expansion_slope,
                       double expansion_degressivity);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
