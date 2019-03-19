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
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChTerrain.h"

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

    ChTire(const std::string& name  ///< [in] name of this tire system
           );

    virtual ~ChTire() {}

    /// Initialize this tire subsystem.
    /// Cache the associated wheel body and vehicle side flag, and add tire mass and
    /// inertia to the wheel body. A derived class must first call this base implementation.
    virtual void Initialize(std::shared_ptr<ChBody> wheel,  ///< [in] associated wheel body
                            VehicleSide side                ///< [in] left/right vehicle side
                            );

    /// Update the state of this tire system at the current time.
    /// The tire system is provided the current state of its associated wheel and
    /// a handle to the terrain system.
    virtual void Synchronize(double time,                    ///< [in] current time
                             const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChTerrain& terrain,       ///< [in] reference to the terrain system
                             CollisionType collision_type = CollisionType::SINGLE_POINT  ///< [in] collision method
                             ) {
        CalculateKinematics(time, wheel_state, terrain);
    }

    /// Set the value of the integration step size for the underlying dynamics (if applicable).
    /// Default value: 1ms.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Get the current value of the integration step size.
    double GetStepsize() const { return m_stepsize; }

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) {}

    /// Get the tire radius.
    virtual double GetRadius() const = 0;

    /// Get the tire mass.
    /// Note that this should not include the mass of the wheel (rim).
    virtual double GetMass() const = 0;

    /// Report the tire mass.
    /// Certain tire models (e.g. those based on FEA) must return 0 in GetMass()
    /// so that the tire mass is not double counted in the underlying mechanical system.
    /// For reporting purposes, use this function instead.
    virtual double ReportMass() const;

    /// Get the tire moments of inertia.
    /// Note that these should not include the inertia of the wheel (rim).
    virtual ChVector<> GetInertia() const = 0;

    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system. Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force on the wheel body.
    /// NOTE: tire models that rely on underlying Chrono functionality (e.g., the
    /// Chrono contact system or Chrono constraints) must always return zero forces
    /// and moments, else tire forces are double counted.
    virtual TerrainForce GetTireForce() const = 0;

    /// Report the tire force and moment.
    /// This function can be used for reporting purposes or else to calculate tire
    /// forces in a co-simulation framework.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const = 0;

    /// Return the tire slip angle calculated based on the current state of the associated
    /// wheel body. The return value is in radians.
    /// (positive sign = left turn, negative sign = right turn)
    double GetSlipAngle() const { return m_slip_angle; }

    /// Return the tire longitudinal slip calculated based on the current state of the associated
    /// wheel body. (positive sign = driving, negative sign = breaking)
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

  protected:
    /// Perform disc-terrain collision detection.
    /// This utility function checks for contact between a disc of specified
    /// radius with given position and orientation (specified as the location of
    /// its center and a unit vector normal to the disc plane) and the terrain
    /// system associated with this tire. It returns true if the disc contacts the
    /// terrain and false otherwise.  If contact occurs, it returns a coordinate
    /// system with the Z axis along the contact normal and the X axis along the
    /// "rolling" direction, as well as a positive penetration depth (i.e. the
    /// height below the terrain of the lowest point on the disc).
    static bool DiscTerrainCollision(
        const ChTerrain& terrain,       ///< [in] reference to terrain system
        const ChVector<>& disc_center,  ///< [in] global location of the disc center
        const ChVector<>& disc_normal,  ///< [in] disc normal, expressed in the global frame
        double disc_radius,             ///< [in] disc radius
        ChCoordsys<>& contact,          ///< [out] contact coordinate system (relative to the global frame)
        double& depth                   ///< [out] penetration depth (positive if contact occurred)
        );

    /// Perform disc-terrain collision detection considering the curvature of the road
    /// surface. The surface normal is calculated based on 4 different height values below
    /// the wheel center. The effective height is calculated as average value of the four
    /// height values.
    /// This utility function checks for contact between a disc of specified
    /// radius with given position and orientation (specified as the location of
    /// its center and a unit vector normal to the disc plane) and the terrain
    /// system associated with this tire. It returns true if the disc contacts the
    /// terrain and false otherwise.  If contact occurs, it returns a coordinate
    /// system with the Z axis along the contact normal and the X axis along the
    /// "rolling" direction, as well as a positive penetration depth (i.e. the
    /// height below the terrain of the lowest point on the disc).
    static bool DiscTerrainCollision4pt(
        const ChTerrain& terrain,       ///< [in] reference to terrain system
        const ChVector<>& disc_center,  ///< [in] global location of the disc center
        const ChVector<>& disc_normal,  ///< [in] disc normal, expressed in the global frame
        double disc_radius,             ///< [in] disc radius
        double width,                   ///< [in] tire width
        ChCoordsys<>& contact,          ///< [out] contact coordinate system (relative to the global frame)
        double& depth,                  ///< [out] penetration depth (positive if contact occurred)
        double& camber_angle            ///< [out] tire camber angle
        );

    /// Collsion algorithm based on a paper of J. Shane Sui and John A. Hirshey II:
    /// "A New Analytical Tire Model for Vehicle Dynamic Analysis" presented at 2001 MSC User Meeting
    static bool DiscTerrainCollisionEnvelope(
        const ChTerrain& terrain,            ///< [in] reference to terrain system
        const ChVector<>& disc_center,       ///< [in] global location of the disc center
        const ChVector<>& disc_normal,       ///< [in] disc normal, expressed in the global frame
        double disc_radius,                  ///< [in] disc radius
        const ChFunction_Recorder& areaDep,  ///< [in] lookup table to calculate depth from intersection area
        ChCoordsys<>& contact,               ///< [out] contact coordinate system (relative to the global frame)
        double& depth                        ///< [out] penetration depth (positive if contact occurred)
    );

    /// Utility function to construct a loopkup table for penetration depth as function of intersection area,
    /// for a given tire radius.  The return map can be used in DiscTerrainCollisionEnvelope.
    static void ConstructAreaDepthTable(double disc_radius, ChFunction_Recorder& areaDep);

    VehicleSide m_side;               ///< tire mounted on left/right side
    std::shared_ptr<ChBody> m_wheel;  ///< associated wheel body
    double m_stepsize;                ///< tire integration step size (if applicable)

  private:
    /// Calculate kinematics quantities based on the current state of the associated wheel body.
    void CalculateKinematics(double time,                    ///< [in] current time
                             const WheelState& wheel_state,  ///< [in] current state of associated wheel body
                             const ChTerrain& terrain        ///< [in] reference to the terrain system
                             );

    double m_slip_angle;
    double m_longitudinal_slip;
    double m_camber_angle;
};

/// Vector of handles to tire subsystems.
typedef std::vector<std::shared_ptr<ChTire> > ChTireList;

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
