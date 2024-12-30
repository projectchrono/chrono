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
// Base class for all vehicle subsystems.
//
// =============================================================================

#ifndef CH_PART_H
#define CH_PART_H

#include <string>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChContactMaterialSMC.h"

#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/utils/ChCompositeInertia.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleJoint.h"
#include "chrono_vehicle/ChVehicleOutput.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for a vehicle subsystem.
/// It manages the part's name, visualization assets, and output.
class CH_VEHICLE_API ChPart {
  public:
    virtual ~ChPart() {}

    /// Get the name identifier for this subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const = 0;

    /// Return flag indicating whether or not the part is fully constructed.
    bool IsInitialized() const { return m_initialized; }

    /// Get the subsystem mass.
    /// Note that the correct value is reported only *after* the subsystem is initialized.
    double GetMass() const { return m_mass; }

    /// Get the current subsystem COM frame (relative to and expressed in the subsystem's reference frame).
    /// Note that the correct value is reported only *after* the subsystem is initialized.
    const ChFrame<>& GetCOMFrame() const { return m_com; }

    /// Get the current subsystem inertia (relative to the subsystem COM frame).
    /// The return 3x3 symmetric matrix
    /// <pre>
    ///  [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [                                int{x^2+y^2}dm ]
    /// </pre>
    /// represents the inertia tensor relative to the subsystem COM frame.
    /// Note that the correct value is reported only *after* the subsystem is initialized.
    const ChMatrix33<>& GetInertia() const { return m_inertia; }

    /// Get the current subsystem position relative to the global frame.
    /// Note that the vehicle frame is defined to be the reference frame of the (main) chassis.
    /// Note that the correct value is reported only *after* the subsystem is initialized.
    const ChFrame<>& GetTransform() const { return m_xform; }

    /// Set the visualization mode for this subsystem.
    void SetVisualizationType(VisualizationType vis);

    /// Add visualization assets to this subsystem, for the specified visualization mode.
    virtual void AddVisualizationAssets(VisualizationType vis) {}

    /// Remove all visualization assets from this subsystem.
    virtual void RemoveVisualizationAssets() {}

    /// Enable/disable output for this subsystem.
    virtual void SetOutput(bool state) { m_output = state; }

    /// Return the output state for this subsystem.
    bool OutputEnabled() const { return m_output; }

    /// Export this subsystem's component list to the specified JSON object.
    /// Derived classes should override this function and first invoke the base class implementation,
    /// followed by calls to the various static Export***List functions, as appropriate.
    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const;

    /// Output data for this subsystem's component list to the specified database.
    virtual void Output(ChVehicleOutput& database) const {}

    /// Utility function for transforming inertia tensors between centroidal frames.
    /// It converts an inertia matrix specified in a centroidal frame aligned with the
    /// vehicle reference frame to an inertia matrix expressed in a centroidal body
    /// reference frame.
    static ChMatrix33<> TransformInertiaMatrix(
        const ChVector3d& moments,        ///< moments of inertia in vehicle-aligned centroidal frame
        const ChVector3d& products,       ///< products of inertia in vehicle-aligned centroidal frame
        const ChMatrix33<>& vehicle_rot,  ///< vehicle absolute orientation matrix
        const ChMatrix33<>& body_rot      ///< body absolute orientation matrix
    );

  protected:
    /// Construct a vehicle subsystem with the specified name.
    ChPart(const std::string& name);

    /// Initialize subsystem inertia properties.
    /// Derived classes must override this function and set the subsystem mass (m_mass) and, if constant, the subsystem
    /// COM frame and its inertia tensor. This function is called during initialization of the vehicle system.
    virtual void InitializeInertiaProperties() = 0;

    /// Update subsystem inertia properties.
    /// Derived classes must override this function and set the global subsystem transform (m_xform) and, unless
    /// constant, the subsystem COM frame (m_com) and its inertia tensor (m_inertia). Calculate the current inertia
    /// properties and global frame of this subsystem. This function is called every time the state of the vehicle
    /// system is advanced in time.
    virtual void UpdateInertiaProperties() = 0;

    /// Add this subsystem's mass.
    /// This utility function first invokes InitializeInertiaProperties and then increments the given total mass.
    void AddMass(double& mass);

    /// Add this subsystem's inertia properties.
    /// This utility function first invokes UpdateInertiaProperties and then incorporates the contribution from this
    /// subsystem to the provided quantities:
    /// - com:  COM expressed in the global frame (scaled by the subsystem mass)
    /// - inertia: inertia tensor relative to the global reference frame
    void AddInertiaProperties(ChVector3d& com, ChMatrix33<>& inertia);

    /// Create a vehicle subsystem from JSON data.
    /// A derived class must override this function and first invoke the base class implementation.
    virtual void Create(const rapidjson::Document& d);

    /// Export the list of bodies to the specified JSON document.
    void ExportBodyList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChBody>> bodies) const;

    /// Export the list of shafts to the specified JSON document.
    void ExportShaftList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChShaft>> shafts) const;

    /// Export the list of joints to the specified JSON document.
    void ExportJointList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLink>> joints) const;

    /// Export the list of shaft couples to the specified JSON document.
    void ExportCouplesList(rapidjson::Document& jsonDocument,
                           std::vector<std::shared_ptr<ChShaftsCouple>> couples) const;

    /// Export the list of markers to the specified JSON document.
    void ExportMarkerList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChMarker>> markers) const;

    /// Export the list of translational springs to the specified JSON document.
    void ExportLinSpringList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLinkTSDA>> springs) const;

    /// Export the list of rotational springs to the specified JSON document.
    void ExportRotSpringList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLinkRSDA>> springs) const;

    /// Export the list of body-body loads to the specified JSON document.
    void ExportBodyLoadList(rapidjson::Document& jsonDocument,
                            std::vector<std::shared_ptr<ChLoadBodyBody>> loads) const;

    /// Erase all visual shapes from the visual model associated with the specified physics item (if any).
    static void RemoveVisualizationAssets(std::shared_ptr<ChPhysicsItem> item);

    /// Erase the given shape from the visual model associated with the specified physics item (if any).
    static void RemoveVisualizationAsset(std::shared_ptr<ChPhysicsItem> item, std::shared_ptr<ChVisualShape> shape);

    std::string m_name;  ///< subsystem name
    bool m_initialized;  ///< specifies whether ot not the part is fully constructed
    bool m_output;       ///< specifies whether or not output is generated for this subsystem

    std::shared_ptr<ChPart> m_parent;  ///< parent subsystem (empty if parent is vehicle)
    double m_mass;                     ///< subsystem mass
    ChMatrix33<> m_inertia;            ///< inertia tensor (relative to subsystem COM)
    ChFrame<> m_com;                   ///< COM frame (relative to subsystem reference frame)
    ChFrame<> m_xform;                 ///< subsystem frame expressed in the global frame

  private:
    friend class ChAxle;
    friend class ChWheeledVehicle;
    friend class ChTrackedVehicle;
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
