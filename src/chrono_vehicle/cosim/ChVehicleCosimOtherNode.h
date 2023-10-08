// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Definition of an additional MPI node not directly involved in co-simulation.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_OTHER_NODE_H
#define CH_VEHCOSIM_OTHER_NODE_H

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Definition of an additional MPI node not directly involved in co-simulation.
/// Such nodes can be used for distributed terrain simulation.
class ChVehicleCosimOtherNode : public ChVehicleCosimBaseNode {
  public:
    ChVehicleCosimOtherNode() : ChVehicleCosimBaseNode("other") {}
    ~ChVehicleCosimOtherNode() {}

    /// Return the node type as NodeType::TIRE.
    virtual NodeType GetNodeType() const override final { return NodeType::TERRAIN; }

    /// Synchronize this node.
    virtual void Synchronize(int step_number, double time) override final {}

    /// Advance simulation.
    virtual void Advance(double step_size) override final {}

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override final {}

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override final {}

    /// No Chrono system for post-processing export.
    virtual ChSystem* GetSystemPostprocess() const override { return nullptr; }
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
