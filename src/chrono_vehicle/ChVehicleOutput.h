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
// Base class for a vehicle output database.
//
// =============================================================================

#ifndef CH_VEHICLE_OUTPUT_H
#define CH_VEHICLE_OUTPUT_H

#include <string>
#include <fstream>

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChLinkSpringCB.h"
#include "chrono/physics/ChLinkRotSpringCB.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for a vehicle output database.
class CH_VEHICLE_API ChVehicleOutput {
  public:
    enum Type {
        ASCII,  ///< ASCII text
        JSON,   ///< JSON
        HDF5    ///< HDF-5
    };

    ChVehicleOutput() {}
    virtual ~ChVehicleOutput() {}

    virtual void WriteTime(double time) = 0;
    virtual void WriteSection(const std::string& name) = 0;
    virtual void WriteBody(std::shared_ptr<ChBody> body) = 0;
    virtual void WriteBodyAuxRef(std::shared_ptr<ChBodyAuxRef> body) = 0;
    virtual void WriteMarker(std::shared_ptr<ChMarker> marker) = 0;
    virtual void WriteShaft(std::shared_ptr<ChShaft> shaft) = 0;
    virtual void WriteJoint(std::shared_ptr<ChLink> joint) = 0;
    virtual void WriteLinSpring(std::shared_ptr<ChLinkSpringCB> spring) = 0;
    virtual void WriteRotSpring(std::shared_ptr<ChLinkRotSpringCB> spring) = 0;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
