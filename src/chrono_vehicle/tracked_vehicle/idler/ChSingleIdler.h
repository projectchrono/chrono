// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a single idler (template definition).
// A single idler is of type LATERAL_PIN.
//
// =============================================================================

#ifndef CH_SINGLE_IDLER_H
#define CH_SINGLE_IDLER_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"

namespace chrono {
namespace vehicle {

///
///
///
class CH_VEHICLE_API ChSingleIdler : public ChIdler {
  public:
    ChSingleIdler(const std::string& name  ///< [in] name of the subsystem
                  );

    virtual ~ChSingleIdler() {}

    /// Return the type of track shoe consistent with this idler.
    virtual TrackShoeType GetType() const override { return LATERAL_PIN; }

    /// Initialize this idler subsystem.
    virtual void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location          ///< [in] location relative to the chassis frame
                            ) override;

    /// Add visualization of the idler wheel.
    /// This (optional) function should be called only after a call to Initialize().
    /// It renders the wheel as a textured cylinder.
    void AddWheelVisualization() override;

  protected:
    /// Return the width of the idler wheel.
    virtual double GetWheelWidth() const = 0;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
