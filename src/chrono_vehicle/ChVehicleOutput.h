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

#include <vector>
#include <string>

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkSpringCB.h"
#include "chrono/physics/ChLinkRotSpringCB.h"
#include "chrono/physics/ChLoadsBody.h"

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

    virtual void WriteTime(int frame, double time) = 0;

    virtual void WriteSection(const std::string& name) = 0;

    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;
    virtual void WriteAuxRefBodies(const std::vector<std::shared_ptr<ChBodyAuxRef>>& bodies) = 0;
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) = 0;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkSpringCB>>& springs) = 0;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRotSpringCB>>& springs) = 0;
    virtual void WriteBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
