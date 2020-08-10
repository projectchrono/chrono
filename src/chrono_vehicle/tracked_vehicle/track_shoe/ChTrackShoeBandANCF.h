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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band track shoe using an ANCFshell-based web
// (template definition).
//
// =============================================================================

#ifndef CH_TRACK_SHOE_BAND_ANCF_H
#define CH_TRACK_SHOE_BAND_ANCF_H

#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBand.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a continuous band track shoe using an ANCFshell-based web.
/// (template definition)
class CH_VEHICLE_API ChTrackShoeBandANCF : public ChTrackShoeBand {
  public:
    enum class ElementType {
        ANCF_4,  ///< 4-node ANCF shell element
        ANCF_8   ///< 8-node ANCF shell element
    };

    ChTrackShoeBandANCF(const std::string& name,                        ///< [in] name of the subsystem
                        ElementType element_type = ElementType::ANCF_4  ///< [in] ANCF shell element type
    );

    virtual ~ChTrackShoeBandANCF() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackShoeBandANCF"; }

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// This version initializes the bodies of a CB rigid-link track shoe such that
    /// the center of the track shoe subsystem is at the specified location and all
    /// bodies have the specified orientation.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next,  ///< [in] handle to the neighbor track shoe
                         bool ccw                            ///< [in] track assembled in counter clockwise direction
                         ) override;

    /// Add visualization assets for the track shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track shoe subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Get the number of shell elements across the web length (from tread body to tread body).
    virtual int GetNumElementsLength() const = 0;

    /// Get the number of shell elements across the web width (side to side).
    virtual int GetNumElementsWidth() const = 0;

    /// Get thickness of the inner steel layer.
    /// The rubber layer thicknesses are obtained from the total web thickness.
    virtual double GetSteelLayerThickness() const = 0;

  private:
    /// Set the FEA mesh container to which this track shoe will add its nodes and elements.
    void SetWebMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Set the FEA layer material properties.
    void SetWebMeshProperties(std::shared_ptr<fea::ChMaterialShellANCF> rubber_mat,
                              std::shared_ptr<fea::ChMaterialShellANCF> steel_mat,
                              double angle_1,
                              double angle_2,
                              double angle_3,
                              double alpha);

    /// Initialize this track shoe system.
    /// This version specifies the locations and orientations of the tread body and of
    /// the web link bodies (relative to the chassis frame).
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,          ///< [in] handle to chassis body
                    const std::vector<ChCoordsys<>>& component_pos  ///< [in] location & orientation of the shoe bodies
    );

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    ElementType m_element_type;

    std::shared_ptr<fea::ChMesh> m_web_mesh;
    std::shared_ptr<fea::ChMaterialShellANCF> m_rubber_mat;
    std::shared_ptr<fea::ChMaterialShellANCF> m_steel_mat;
    double m_angle_1;
    double m_angle_2;
    double m_angle_3;
    double m_alpha;

    unsigned int m_starting_node_index;

    friend class ChTrackAssemblyBandANCF;
};

/// Vector of handles to continuous band ANCFshell-based track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeBandANCF>> ChTrackShoeBandANCFList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
