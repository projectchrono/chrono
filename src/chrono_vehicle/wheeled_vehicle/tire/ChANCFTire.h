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
// Template for a deformable ANCF tire.
//
// =============================================================================

#ifndef CH_ANCFTIRE_H
#define CH_ANCFTIRE_H

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include "chrono/fea/ChMaterialShellANCF.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// ANCF tire template.
/// This tire is modeled as a mesh composed of ANCF shell elements.
class CH_VEHICLE_API ChANCFTire : public ChDeformableTire {
  public:
    ChANCFTire(const std::string& name);

    virtual ~ChANCFTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ANCFTire"; }

  protected:
    /// Tire profile.
    struct Profile {
        std::vector<double> t;  ///< independent parameter
        std::vector<double> x;  ///< x coordinate (radial direction)
        std::vector<double> y;  ///< y coordinate (transversal direction)
    };

    /// Tire section.
    struct Section {
        int num_divs;                                                ///< number of section divisions
        int num_layers;                                              ///< number of layers
        std::vector<double> thickness;                               ///< layer thickness
        std::vector<double> angle;                                   ///< layer ply angle
        std::vector<std::shared_ptr<fea::ChMaterialShellANCF>> mat;  ///< layer material
    };

    /// Create the ChLoad for applying pressure to the tire.
    virtual void CreatePressureLoad() override final;

    /// Create the contact surface for the tire mesh.
    virtual void CreateContactSurface() override final;

    /// Create the tire-rim connections.
    virtual void CreateRimConnections(std::shared_ptr<ChBody> wheel) override final;

    /// Utility class to generate a tire mesh using 4-node ANCF shell elements (ChElementShellANCF_3423).
    /// Returns a vector with the nodes that must be connected to the wheel rim.
    static std::vector<std::shared_ptr<fea::ChNodeFEAbase>> CreateMeshANCF4(
        const Profile& profile,             ///< tire profile
        const Section& bead,                ///< specification of bead section
        const Section& sidewall,            ///< specification of sidewall section
        const Section& tread,               ///< specification of tread section
        int div_circumference,              ///< number of divisions along circumference
        double rim_radius,                  ///< rim radius
        double damping,                     ///< structural damping
        std::shared_ptr<fea::ChMesh> mesh,  ///< containing FEA mesh
        const ChFrameMoving<>& wheel_frame  ///< associated wheel frame
    );

    /// Utility class to generate a tire mesh using 8-node ANCF shell elements (ChElementShellANCF_3833).
    /// Returns a vector with the nodes that must be connected to the wheel rim.
    static std::vector<std::shared_ptr<fea::ChNodeFEAbase>> CreateMeshANCF8(
        const Profile& profile,             ///< tire profile
        const Section& bead,                ///< specification of bead section
        const Section& sidewall,            ///< specification of sidewall section
        const Section& tread,               ///< specification of tread section
        int div_circumference,              ///< number of divisions along circumference
        double rim_radius,                  ///< rim radius
        double damping,                     ///< structural damping
        std::shared_ptr<fea::ChMesh> mesh,  ///< containing FEA mesh
        const ChFrameMoving<>& wheel_frame  ///< associated wheel frame
    );
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
