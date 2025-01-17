// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// HMMWV ANCF-4 single-layer tire.
//
// =============================================================================

#ifndef HMMWV_ANCF4_LUMPED_TIRE_H
#define HMMWV_ANCF4_LUMPED_TIRE_H

#include "chrono_models/ChApiModels.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_ANCF4LumpedTire : public ChANCFTire {
  public:
    HMMWV_ANCF4LumpedTire(const std::string& name);
    ~HMMWV_ANCF4LumpedTire() {}

    /// Set FEA mesh resolution.
    /// The number of divisions in the transversal direction is calculated as
    /// <pre>
    /// num_div_transversalnum_div_transversal = 2 * (num_elements_b + num_elements_s + num_elements_t)
    /// </pre>
    /// resulting in a total number of elements:
    /// <pre>
    /// num_elements = num_div_circumference * num_div_transversal
    /// </pre>
    void SetMeshResolution(int num_div_circumference,  ///< number of circumferential divisions (default: 40)
                           int num_elements_b,         ///< number of elements in bead section (default: 1)
                           int num_elements_s,         ///< number of elements in side wall section (default: 2)
                           int num_elements_t          ///< number of elements in tread section (default: 2)
    );

    /// Set the contact material.
    void SetContactMaterial(std::shared_ptr<ChContactMaterialSMC> mat);

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_tire_radius; }

    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const override { return m_rim_radius; }
    
    /// Get the tire width.
    virtual double GetWidth() const override { return m_rim_width; }
    
    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

    /// Return list of nodes connected to the rim.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override { return m_rim_nodes; }

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame, VehicleSide side) override;

  private:
    virtual void CreateContactMaterial() override {}

    int m_num_div_circumference;
    int m_num_div_transversal;
    int m_num_elements_b;
    int m_num_elements_s;
    int m_num_elements_t;

    static const double m_tire_radius;
    static const double m_rim_radius;
    static const double m_rim_width;

    static const double m_alpha;
    static const double m_default_pressure;

    static const unsigned int m_num_points;
    static const double m_profile[71][3];

    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> m_rim_nodes;
};

}  // namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
