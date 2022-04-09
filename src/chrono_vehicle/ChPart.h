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
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

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
    //// RADU TODO - protected constructor

    /// Construct a vehicle subsystem with the specified name.
    ChPart(const std::string& name  ///< [in] name of the subsystem
           );

    virtual ~ChPart() {}

    /// Get the name identifier for this subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const = 0;

    /// Get the subsystem mass.
    double GetMass() const { return m_mass; }

    /// Get the current subsystem COM frame (relative to and expressed in the subsystem's reference frame).
    const ChFrame<>& GetCOMFrame() const { return m_com; }

    /// Get the current subsystem inertia (relative to the subsystem COM frame).
    /// The return 3x3 symmetric matrix
    /// <pre>
    ///  [ int{x^2+z^2}dm    -int{xy}dm    -int{xz}dm    ]
    ///  [                  int{x^2+z^2}   -int{yz}dm    ]
    ///  [                                int{x^2+y^2}dm ]
    /// </pre>
    /// represents the inertia tensor relative to the subsystem COM frame.
    const ChMatrix33<>& GetInertia() const { return m_inertia; }

    /// Get the current subsystem position relative to the global frame.
    /// Note that the vehicle frame is defined to be the reference frame of the (main) chassis.
    const ChFrame<>& GetPosition() const { return m_pos; }

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
        const ChVector<>& moments,        ///< moments of inertia in vehicle-aligned centroidal frame
        const ChVector<>& products,       ///< products of inertia in vehicle-aligned centroidal frame
        const ChMatrix33<>& vehicle_rot,  ///< vehicle absolute orientation matrix
        const ChMatrix33<>& body_rot      ///< body absolute orientation matrix
        );

  protected:
    /// Calculate the subsystem mass.
    /// Derived classes must override this function and set m_mass.
    virtual void CalculateMass() = 0;

    /// Calculate the current inertia properties and global frame of this subsystem.
    /// Derived classes must implement this function and set m_inertia, m_com, and m_pos.
    virtual void CalculateInertia() = 0;

    /// Get the subsystem COM frame in the absolute frame.
    ChFrame<> GetCOMFrame_abs() const;

    /// Create a vehicle subsystem from JSON data.
    /// A derived class must override this function and first invoke the base class implementation.
    virtual void Create(const rapidjson::Document& d);

    /// Export the list of bodies to the specified JSON document.
    static void ExportBodyList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChBody>> bodies);

    /// Export the list of shafts to the specified JSON document.
    static void ExportShaftList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChShaft>> shafts);

    /// Export the list of joints to the specified JSON document.
    static void ExportJointList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChLink>> joints);

    /// Export the list of shaft couples to the specified JSON document.
    static void ExportCouplesList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChShaftsCouple>> couples);

    /// Export the list of markers to the specified JSON document.
    static void ExportMarkerList(rapidjson::Document& jsonDocument, std::vector<std::shared_ptr<ChMarker>> markers);

    /// Export the list of translational springs to the specified JSON document.
    static void ExportLinSpringList(rapidjson::Document& jsonDocument,
                                    std::vector<std::shared_ptr<ChLinkTSDA>> springs);

    /// Export the list of rotational springs to the specified JSON document.
    static void ExportRotSpringList(rapidjson::Document& jsonDocument,
                                    std::vector<std::shared_ptr<ChLinkRSDA>> springs);

    /// Export the list of body-body loads to the specified JSON document.
    static void ExportBodyLoadList(rapidjson::Document& jsonDocument,
                                   std::vector<std::shared_ptr<ChLoadBodyBody>> loads);

    std::string m_name;  ///< subsystem name
    bool m_output;       ///< specifies whether or not output is generated for this subsystem

    double m_mass;           ///< subsystem mass
    ChMatrix33<> m_inertia;  ///< inertia tensor (relative to subsystem COM)
    ChFrame<> m_com;         ///< COM frame (relative to subsystem reference frame)
    ChFrame<> m_pos;         ///< global position of the subsystem frame

  private:
    friend class ChAxle;
    friend class ChWheeledVehicle;
    friend class ChTrackedVehicle;
};

// Utility class with material information for a collision shape.
class CH_VEHICLE_API MaterialInfo {
  public:
    float mu;  // coefficient of friction
    float cr;  // coefficient of restitution
    float Y;   // Young's modulus
    float nu;  // Poisson ratio
    float kn;  // normal stiffness
    float gn;  // normal viscous damping
    float kt;  // tangential stiffness
    float gt;  // tangential viscous damping

    MaterialInfo();
    MaterialInfo(float mu_, float cr_, float Y_, float nu_, float kn_, float gn_, float kt_, float gt_);

    // Construct a contact material, consistent with the specified method, using the data in the provided structure.
    std::shared_ptr<ChMaterialSurface> CreateMaterial(ChContactMethod contact_method);
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
