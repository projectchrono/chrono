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
// Base class for a tire.
// A tire system is a force element. It is passed position and velocity
// information of the wheel body and it produces ground reaction forces and
// moments to be applied to the wheel body.
//
// =============================================================================

#ifndef CH_TIRE_H
#define CH_TIRE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Base class for a tire system.
/// A tire subsystem is a force element. It is passed position and velocity
/// information of the wheel body and it produces ground reaction forces and
/// moments to be applied to the wheel body.
class CH_VEHICLE_API ChTire : public ChPart {
  public:
    enum class CollisionType { SINGLE_POINT, FOUR_POINTS, ENVELOPE };

    virtual ~ChTire() {}

    /// Set the value of the integration step size for the underlying dynamics (if applicable).
    /// Default value: 1ms.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Get the current value of the integration step size.
    double GetStepsize() const { return m_stepsize; }

    /// Set the collision type for tire-terrain interaction.
    /// Default: SINGLE_POINT
    void SetCollisionType(CollisionType collision_type) { m_collision_type = collision_type; }

    /// Set the internal tire pressure [Pa].
    /// Default value: 0.
    /// Derived classes (concrete tire models) may or may not use this setting.
    void SetPressure(double pressure) { m_pressure = pressure; }

    /// Get the internal tire pressure [Pa].
    double GetPressure() const { return m_pressure; }

    /// Get the tire radius.
    virtual double GetRadius() const = 0;

    /// Get the tire width.
    virtual double GetWidth() const = 0;

    /// Return the tire mass.
    virtual double GetTireMass() const = 0;

    /// Return the tire moments of inertia (in the tire centroidal frame).
    virtual ChVector<> GetTireInertia() const = 0;

    /// Report the tire force and moment.
    /// This function can be used for reporting purposes or else to calculate tire forces in a co-simulation framework.
    /// The return application point, force, and moment are assumed to be expressed in the global reference frame.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const = 0;

    /// Get the tire force and moment expressed in the tire frame.
    /// The tire frame has its origin in the contact patch, the X axis in the tire heading direction and the Z axis in
    /// the terrain normal at the contact point.
    /// If the tire is not in contact, the tire frame is not set and the function returns zero force and moment.
    virtual TerrainForce ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const = 0;

    /// Return the tire slip angle calculated based on the current state of the associated wheel body.
    /// The return value is in radians (positive sign = left turn, negative sign = right turn).
    double GetSlipAngle() const { return m_slip_angle; }

    /// Return the tire longitudinal slip calculated based on the current state of the associated wheel body.
    /// (positive sign = driving, negative sign = breaking)
    double GetLongitudinalSlip() const { return m_longitudinal_slip; }

    /// Return the tire camber angle calculated based on the current state of the associated
    /// wheel body. The return value is in radians.
    /// (positive sign = upper side tipping to the left, negative sign = upper side tipping to the right)
    double GetCamberAngle() const { return m_camber_angle; }

    /// Utility function for estimating the tire moments of inertia.
    /// The tire is assumed to be specified with the common scheme (e.g. 215/65R15)
    /// and the mass of the tire (excluding the wheel) provided.
    static ChVector<> EstimateInertia(double tire_width,    ///< tire width [mm]
                                      double aspect_ratio,  ///< aspect ratio: height to width [percentage]
                                      double rim_diameter,  ///< rim diameter [in]
                                      double tire_mass,     ///< mass of the tire [kg]
                                      double t_factor = 2   ///< tread to sidewall thickness factor
    );

    /// Report the tire deflection.
    virtual double GetDeflection() const { return 0; }

    /// Get the name of the Wavefront file with tire visualization mesh.
    /// An empty string is returned if no mesh was specified.
    const std::string& GetMeshFilename() const { return m_vis_mesh_file; }

  public:
    // NOTE: Typically, users should not directly call these functions. They are public for use in special cases and to
    // allow extensions to Chrono::Vehicle in user code.

    /// Initialize this tire subsystem by associating it to an existing wheel subsystem.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel);

    /// Update the state of this tire system at the current time.
    /// A derived class should also set the current slip, longitudinal slip, and camber angle.
    virtual void Synchronize(double time, const ChTerrain& terrain) {}

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) {}

  protected:
    /// Construct a tire subsystem with given name.
    ChTire(const std::string& name);

    /// Calculate kinematics quantities based on the given state of the associated wheel body.
    void CalculateKinematics(const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChCoordsys<>& tire_frame  ///< [in] tire contact frame
    );

    /// Get offset from spindle center.
    /// This queries the associated wheel, so it must be called only after the wheel was initialized.
    double GetOffset() const { return m_wheel->m_offset; }

    /// Get the mass added to the associated spindle body.
    /// Certain tires (e.g., those FEA-based) have their own physical representation and hence do not add mass and
    /// inertia to the spindle body. All others increment the spindle body mass by the amount repoirted by this
    /// function.
    virtual double GetAddedMass() const = 0;

    /// Get the inertia added to the associated spindle body.
    /// Certain tires (e.g., those FEA-based) have their own physical representation and hence do not add mass and
    /// inertia to the spindle body. All others increment the spindle body moments of inertia by the amount reported by
    /// this function.
    virtual ChVector<> GetAddedInertia() const = 0;

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the vehicle system. The return application
    /// point, force, and moment are assumed to be expressed in the global reference frame.  Typically, the vehicle
    /// subsystem will pass the tire force to the appropriate suspension subsystem which applies it as an external force
    /// on the wheel body. NOTE: tire models that rely on underlying Chrono functionality (e.g., the Chrono contact
    /// system or Chrono constraints) must always return zero forces and moments, else tire forces are double counted.
    virtual TerrainForce GetTireForce() const = 0;

    /// Add mesh visualization to the body associated with this tire (a wheel spindle body).
    /// The two meshes are assumed to be specified with respect to a frame with origin at the center of the tire and Y
    /// axis pointing towards the outside. This function uses one of the two provided OBJ files, depending on the side
    /// on which the tire is mounted. The name of the output mesh shape is set to be the stem of the input filename.
    std::shared_ptr<ChVisualShapeTriangleMesh> AddVisualizationMesh(const std::string& mesh_file_left,
                                                              const std::string& mesh_file_right);

    /// Perform disc-terrain collision detection, using the specified method.
    static bool DiscTerrainCollision(
        CollisionType method,                ///< [in] tire-terrain collision detection method
        const ChTerrain& terrain,            ///< [in] reference to terrain system
        const ChVector<>& disc_center,       ///< [in] global location of the disc center
        const ChVector<>& disc_normal,       ///< [in] disc normal, expressed in the global frame
        double disc_radius,                  ///< [in] disc radius
        double width,                        ///< [in] tire width
        const ChFunction_Recorder& areaDep,  ///< [in] lookup table to calculate depth from intersection area
        ChCoordsys<>& contact,               ///< [out] contact coordinate system (relative to the global frame)
        double& depth,                       ///< [out] penetration depth (positive if contact occurred)
        float& mu                            ///< [out] coefficient of friction at contact
    );

    /// Utility function to construct a loopkup table for penetration depth as function of intersection area,
    /// for a given tire radius.  The return map can be used in DiscTerrainCollisionEnvelope.
    static void ConstructAreaDepthTable(double disc_radius, ChFunction_Recorder& areaDep);

    /// Perform disc-terrain collision detection.
    /// This utility function checks for contact between a disc of specified radius with given position and orientation
    /// (specified as the location of its center and a unit vector normal to the disc plane) and the terrain system
    /// associated with this tire. It returns true if the disc contacts the terrain and false otherwise.  If contact
    /// occurs, it returns a coordinate system with the Z axis along the contact normal and the X axis along the
    /// "rolling" direction, as well as a positive penetration depth (i.e. the height below the terrain of the lowest
    /// point on the disc).
    static bool DiscTerrainCollision1pt(
        const ChTerrain& terrain,       ///< [in] reference to terrain system
        const ChVector<>& disc_center,  ///< [in] global location of the disc center
        const ChVector<>& disc_normal,  ///< [in] disc normal, expressed in the global frame
        double disc_radius,             ///< [in] disc radius
        ChCoordsys<>& contact,          ///< [out] contact coordinate system (relative to the global frame)
        double& depth,                  ///< [out] penetration depth (positive if contact occurred)
        float& mu                       ///< [out] coefficient of friction at contact
    );

    /// Perform disc-terrain collision detection considering the curvature of the road surface.
    /// The surface normal is calculated based on 4 different height values below the wheel center. The effective height
    /// is calculated as average value of the four height values. This utility function checks for contact between a
    /// disc of specified radius with given position and orientation (specified as the location of its center and a unit
    /// vector normal to the disc plane) and the terrain system associated with this tire. It returns true if the disc
    /// contacts the terrain and false otherwise.  If contact occurs, it returns a coordinate system with the Z axis
    /// along the contact normal and the X axis along the "rolling" direction, as well as a positive penetration depth
    /// (i.e. the height below the terrain of the lowest point on the disc).
    static bool DiscTerrainCollision4pt(
        const ChTerrain& terrain,       ///< [in] reference to terrain system
        const ChVector<>& disc_center,  ///< [in] global location of the disc center
        const ChVector<>& disc_normal,  ///< [in] disc normal, expressed in the global frame
        double disc_radius,             ///< [in] disc radius
        double width,                   ///< [in] tire width
        ChCoordsys<>& contact,          ///< [out] contact coordinate system (relative to the global frame)
        double& depth,                  ///< [out] penetration depth (positive if contact occurred)
        float& mu                       ///< [out] coefficient of friction at contact
    );

    /// Collsion algorithm based on a paper of J. Shane Sui and John A. Hirshey II:
    /// "A New Analytical Tire Model for Vehicle Dynamic Analysis" presented at 2001 MSC User Meeting
    static bool DiscTerrainCollisionEnvelope(
        const ChTerrain& terrain,            ///< [in] reference to terrain system
        const ChVector<>& disc_center,       ///< [in] global location of the disc center
        const ChVector<>& disc_normal,       ///< [in] disc normal, expressed in the global frame
        double disc_radius,                  ///< [in] disc radius
        double width,                        ///< [in] tire width
        const ChFunction_Recorder& areaDep,  ///< [in] lookup table to calculate depth from intersection area
        ChCoordsys<>& contact,               ///< [out] contact coordinate system (relative to the global frame)
        double& depth,                       ///< [out] penetration depth (positive if contact occurred)
        float& mu                            ///< [out] coefficient of friction at contact
    );

    std::shared_ptr<ChWheel> m_wheel;  ///< associated wheel subsystem
    double m_stepsize;                 ///< tire integration step size (if applicable)
    double m_pressure;                 ///< internal tire pressure
    CollisionType m_collision_type;    ///< method used for tire-terrain collision
    std::string m_vis_mesh_file;  ///< name of OBJ file for visualization of this tire (may be empty)

    double m_slip_angle;
    double m_longitudinal_slip;
    double m_camber_angle;

    friend class ChWheel;
    friend class ChWheeledVehicle;
    friend class ChWheeledTrailer;
};

/// Vector of handles to tire subsystems.
typedef std::vector<std::shared_ptr<ChTire> > ChTireList;

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
