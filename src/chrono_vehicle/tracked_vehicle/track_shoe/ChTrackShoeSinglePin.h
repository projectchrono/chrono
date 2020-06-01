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
// Base class for a single-pin track shoe (template definition).
// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_SINGLE_PIN_H
#define CH_TRACK_SHOE_SINGLE_PIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a single-pin track shoe (template definition).
/// A single-pin track shoe can be either of CENTRAL_PIN or LATERAL_PIN type.
class CH_VEHICLE_API ChTrackShoeSinglePin : public ChTrackShoe {
  public:
    ChTrackShoeSinglePin(const std::string& name  ///< [in] name of the subsystem
                         );

    virtual ~ChTrackShoeSinglePin() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackShoeSinglePin"; }

    /// Get the mass of the track shoe.
    virtual double GetMass() const override;

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// A derived class must extend this default implementation and specify the contact
    /// geometry for the track shoe body.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) override;

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after both track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
                         ) override;

    /// Add visualization assets for the track-shoe subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the track-shoe subsystem.
    virtual void RemoveVisualizationAssets() override final;

  protected:
    /// Return the mass of the shoe body.
    virtual double GetShoeMass() const = 0;

    /// Return the moments of inertia of the shoe body.
    virtual const ChVector<>& GetShoeInertia() const = 0;

    /// Return the location of the front contact cylinder.
    /// This location is relative to the shoe reference frame (in the positive x direction)
    virtual double GetFrontCylinderLoc() const = 0;

    /// Return the location of the rear contact cylinder.
    /// This location is relative to the shoe reference frame (in the negative x direction)
    virtual double GetRearCylinderLoc() const = 0;

    /// Return the radius of the contact cylinders.
    virtual double GetCylinderRadius() const = 0;

    /// Create the contact materials for the shoe, consistent with the specified contact method. A derived class must
    /// set m_cyl_material (used for contact with the sprocket) and m_shoe_materials which must include one or more
    /// contact materials for the collision shapes of the shoe itself (for contact with the wheels, idler, and ground).
    virtual void CreateContactMaterials(ChContactMethod contact_method) = 0;

    /// Add contact geometry for the track shoe.
    /// Note that this is for contact with wheels, idler, and ground only.
    /// This contact geometry does not affect contact with the sprocket.
    /// The default implementation uses contact boxes for the pad and central guiding pin.
    virtual void AddShoeContact();

    virtual void ExportComponentList(rapidjson::Document& jsonDocument) const override;

    virtual void Output(ChVehicleOutput& database) const override;

    struct BoxShape {
        BoxShape(const ChVector<>& pos, const ChQuaternion<>& rot, const ChVector<>& dims, int matID = -1)
            : m_pos(pos), m_rot(rot), m_dims(dims), m_matID(matID) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        ChVector<> m_dims;
        int m_matID;
    };

    struct CylinderShape {
        CylinderShape(const ChVector<>& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1)
            : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length), m_matID(matID) {}
        ChVector<> m_pos;
        ChQuaternion<> m_rot;
        double m_radius;
        double m_length;
        int m_matID;
    };

    std::vector<BoxShape> m_coll_boxes;                                ///< collision boxes on shoe body
    std::vector<CylinderShape> m_coll_cylinders;                       ///< collision cylinders on shoe body
    std::vector<std::shared_ptr<ChMaterialSurface>> m_shoe_materials;  ///< contact materials for shoe collision shapes
    std::shared_ptr<ChMaterialSurface> m_cyl_material;                 ///< contact material for cylindrical surface

    std::vector<BoxShape> m_vis_boxes;
    std::vector<CylinderShape> m_vis_cylinders;

    friend class ChSprocketSinglePin;
    friend class SprocketSinglePinContactCB;
    friend class ChTrackAssemblySinglePin;
};

/// Vector of handles to single-pin track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoeSinglePin> > ChTrackShoeSinglePinList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
