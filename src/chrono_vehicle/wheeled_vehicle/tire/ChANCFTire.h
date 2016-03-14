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
// Template for a deformable ANCF tire
//
// =============================================================================

#ifndef CH_ANCFTIRE_H
#define CH_ANCFTIRE_H

#include "chrono/physics/ChBody.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// List of nodes
typedef std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> NodeList;

/// ANCF tire model.
/// This tire is modeled as a mesh composed of ANCF shell element.
class CH_VEHICLE_API ChANCFTire : public ChTire {
  public:
    enum ContactSurfaceType { NODE_CLOUD, TRIANGLE_MESH };

    ChANCFTire(const std::string& name  ///< [in] name of this tire system
               );

    virtual ~ChANCFTire() {}

    /// Set the type of contact surface.
    void SetContactSurfaceType(ContactSurfaceType type) { m_contact_type = type; }

    /// Set radius of contact nodes.
    /// This value is relevant only for NODE_CLOUD contact surface type.
    void SetContactNodeRadius(double radius) { m_contact_node_radius = radius; }

    /// Set thickness of contact faces (radius of swept sphere).
    /// This value is relevant only for TRIANGLE_MESH contact surface type.
    void SetContactFaceThickness(double thickness) { m_contact_face_thickness = thickness; }

    /// Set contact material properties
    void SetContactMaterial(float friction_coefficient = 0.6f,    ///< [in] coefficient of friction
                            float restitution_coefficient = 0.1,  ///< [in] coefficient of restitution
                            float young_modulus = 2e5f,           ///< [in] Young's modulus of elasticity
                            float poisson_ratio = 0.3f            ///< [in] Poisson ratio
                            );

    /// Enable/didable tire pressure.
    void EnablePressure(bool val) { m_pressure_enabled = val; }

    /// Enable/disable tire contact.
    void EnableContact(bool val) { m_contact_enabled = val; }

    /// Enable/disable tire-rim connection.
    void EnableRimConnection(bool val) { m_connection_enabled = val; }

    /// Set the tire pressure.
    void SetPressure(double pressure) {
        assert(m_pressure > 0);
        m_pressure = pressure;
    }

    /// Get total tire mass.
    double GetMass() const;

    /// Get the tire force and moment.
    /// For a rigid tire, the tire forces are automatically applied to the
    /// associated wheel (through Chrono's frictional contact system). The values
    /// returned here are never used.
    virtual TireForce GetTireForce() const override;

    /// Initialize this tire system.
    /// This function creates the tire contact shape and attaches it to the
    /// associated wheel body.
    void Initialize(std::shared_ptr<ChBody> wheel,  ///< handle to the associated wheel body
                    VehicleSide side                ///< left/right vehicle side
                    );

  protected:
    /// Return the tire radius.
    virtual double GetTireRadius() const = 0;

    /// Return the rim radius.
    virtual double GetRimRadius() const = 0;

    /// Return the tire width.
    virtual double GetWidth() const = 0;

    /// Return the default tire pressure.
    virtual double GetDefaultPressure() const = 0;

    /// Return list of nodes connected to the rim.
    virtual NodeList GetConnectedNodes(const std::shared_ptr<fea::ChMesh>& mesh) const = 0;

    /// Create the FEA nodes and elements.
    /// The wheel rotational axis is assumed to be the Y axis.
    virtual void CreateMesh(std::shared_ptr<fea::ChMesh> mesh,   ///< containing mesh
                            const ChFrameMoving<>& wheel_frame,  ///< frame of associated wheel
                            VehicleSide side                     ///< left/right vehicle side
                            ) = 0;

  private:
    std::shared_ptr<fea::ChMesh> m_mesh;

    bool m_connection_enabled;
    bool m_pressure_enabled;
    bool m_contact_enabled;

    double m_pressure;

    ContactSurfaceType m_contact_type;
    double m_contact_node_radius;
    double m_contact_face_thickness;
    float m_friction;
    float m_restitution;
    float m_young_modulus;
    float m_poisson_ratio;
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
