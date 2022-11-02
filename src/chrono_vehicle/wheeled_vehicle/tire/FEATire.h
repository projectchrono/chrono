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
// FEA co-rotational tire constructed with data from file (JSON format).
// The mesh data is assumed to be provided through an Abaqus INP file.
//
// =============================================================================

#ifndef FEA_TIRE_H
#define FEA_TIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Co-rotational FEA tire constructed with data from file (JSON format).
class CH_VEHICLE_API FEATire : public ChFEATire {
  public:
    FEATire(const std::string& filename);
    FEATire(const rapidjson::Document& d);
    ~FEATire() {}

    /// Get the tire radius.
    virtual double GetRadius() const override { return m_tire_radius; }
    /// Get the rim radius (inner tire radius).
    virtual double GetRimRadius() const override { return m_rim_radius; }
    /// Get the tire width.
    virtual double GetWidth() const override { return m_rim_width; }
    /// Get the default tire pressure.
    virtual double GetDefaultPressure() const override { return m_default_pressure; }

    /// Return list of internal nodes.
    /// These nodes define the mesh surface over which pressure loads are applied.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetInternalNodes() const override;

    /// Return list of nodes connected to the rim.
    virtual std::vector<std::shared_ptr<fea::ChNodeFEAbase>> GetConnectedNodes() const override;

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(const ChFrameMoving<>& wheel_frame,  ///< frame of associated wheel
                            VehicleSide side                     ///< left/right vehicle side
                            ) override;

  private:
    void ProcessJSON(const rapidjson::Document& d);
    virtual void CreateContactMaterial() override;

    double m_tire_radius;
    double m_rim_radius;
    double m_rim_width;

    double m_default_pressure;

    std::shared_ptr<fea::ChContinuumElastic> m_material;
    std::string m_input_file;

    std::map<std::string, std::vector<std::shared_ptr<fea::ChNodeFEAbase>>> m_node_sets;

    ChContactMaterialData m_mat_info;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
