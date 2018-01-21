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

#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChNodeFEAbase.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

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
    virtual void Connect(std::shared_ptr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
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

    /// Set material properties of the rubber layers (isotropic material).
    void SetRubberLayerMaterial(double rho,            ///< density (default: 1100)
                                const ChVector<>& E,   ///< elasticity moduli, E_x, E_y, E_z (default: 1e7)
                                const ChVector<>& nu,  ///< Poisson ratios, nu_x, nu_y, nu_z (default: 0.49)
                                const ChVector<>& G    ///< shear moduli, G_x, Gy, Gz (default: 0.5 * E / (1 + nu))
    );

    /// Set material properties of the steel layer (isotropic material).
    void SetSteelLayerMaterial(double rho,            ///< density (default: 1100)
                               const ChVector<>& E,   ///< elasticity moduli, E_x, E_y, E_z (default: 1e7)
                               const ChVector<>& nu,  ///< Poisson ratios, nu_x, nu_y, nu_z (default: 0.3)
                               const ChVector<>& G    ///< shear moduli, G_x, Gy, Gz (default: 0.5 * E / (1 + nu))
    );

    /// Set element structural damping (default: 0.05).
    void SetElementStructuralDamping(double alpha);

    /// Set fiber angle for the three layers.
    void SetLayerFiberAngles(double angle_1,  ///< fiber angle for bottom (outter) rubber layer (default: 0 rad)
                             double angle_2,  ///< fiber angle for middle steel layer (default: 0 rad)
                             double angle_3   ///< fiber angle for top (inner) rubber layer (default: 0 rad)
    );

  private:
    /// Set the FEA mesh container to which this track shoe will add its nodes and elements.
    void SetWebMesh(std::shared_ptr<fea::ChMesh> mesh) { m_web_mesh = mesh; }

    /// Initialize this track shoe system.
    /// This version specifies the locations and orientations of the tread body and of
    /// the web link bodies (relative to the chassis frame).
    void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,          ///< [in] handle to chassis body
                    const std::vector<ChCoordsys<>>& component_pos  ///< [in] location & orientation of the shoe bodies
    );

    ElementType m_element_type;

    std::shared_ptr<fea::ChMesh> m_web_mesh;

    unsigned int m_starting_node_index;

    double m_rubber_rho;
    ChVector<> m_rubber_E;
    ChVector<> m_rubber_nu;
    ChVector<> m_rubber_G;

    double m_steel_rho;
    ChVector<> m_steel_E;
    ChVector<> m_steel_nu;
    ChVector<> m_steel_G;

    double m_angle_1;
    double m_angle_2;
    double m_angle_3;

    double m_alpha;

    friend class ChTrackAssemblyBandANCF;
};

/// Vector of handles to continuous band ANCFshell-based track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeBandANCF>> ChTrackShoeBandANCFList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
