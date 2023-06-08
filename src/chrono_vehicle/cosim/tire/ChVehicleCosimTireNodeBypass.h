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
// Definition of a vehicle co-simulation TIRE NODE class which only
// intermediates communication between the MBS and Terrain nodes.
// This type of tire node communicates with the terrain node through a BODY
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TIRE_NODE_BYPASS_H
#define CH_VEHCOSIM_TIRE_NODE_BYPASS_H

#include "chrono_vehicle/cosim/ChVehicleCosimTireNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_tire
/// @{

/// Definition of the bypass tire node.
/// A tire node of this type does not provide mesh information nor contact material information.
class CH_VEHICLE_API ChVehicleCosimTireNodeBypass : public ChVehicleCosimTireNode {
  public:
    ChVehicleCosimTireNodeBypass(int index, double mass, double radius, double width);
    ~ChVehicleCosimTireNodeBypass() {}

    /// Return the tire type.
    virtual TireType GetTireType() const override { return TireType::BYPASS; }

    /// Advance simulation.
    /// A bypass tire node has no dynamics.
    virtual void Advance(double step_size) override {}

  private:
    /// A bypass tire implements the BODY communication interface.
    virtual InterfaceType GetInterfaceType() const override { return InterfaceType::BODY; }

    /// Return the tire mass.
    virtual double GetTireMass() const override { return m_mass; }

    /// Return the tire radius.
    virtual double GetTireRadius() const override { return m_radius; }

    /// Return the tire width.
    virtual double GetTireWidth() const override { return m_width; }

    /// Initialize the tire by attaching it to the provided ChWheel.
    /// A bypass tire requires no construction.
    virtual void InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector<>& init_loc) override {}

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override {}

  private:
    double m_mass;
    double m_radius;
    double m_width;
};

/// @} vehicle_cosim_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
