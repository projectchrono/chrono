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
// Base class for a track shoe.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_SHOE_H
#define CH_TRACK_SHOE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

/**
    @addtogroup vehicle_tracked
    @{
        @defgroup vehicle_tracked_shoe Track-shoe subsystem
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_shoe
/// @{

/// Base class for a track shoe.
class CH_VEHICLE_API ChTrackShoe {
  public:
    ChTrackShoe(const std::string& name  ///< [in] name of the subsystem
                );

    virtual ~ChTrackShoe() {}

    /// Return the type of track shoe (guiding pin).
    /// A derived class must specify the type of track shoe (which must be
    /// consistent with the idler and road wheels in the containing track assembly.
    virtual GuidePinType GetType() const = 0;

    /// Get the name identifier for this track shoe subsystem.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this track shoe subsystem.
    void SetName(const std::string& name) { m_name = name; }

    /// Get a handle to the shoe body.
    std::shared_ptr<ChBody> GetShoeBody() const { return m_shoe; }

    /// Return the height of the track shoe.
    virtual double GetHeight() const = 0;

    /// Return the pitch length of the track shoe.
    /// This quantity must agree with the pitch of the sprocket gear.
    virtual double GetPitch() const = 0;

    /// Get the index of this track shoe within its containing track assembly.
    size_t GetIndex() const { return m_index; }

    /// Get the mass of the track shoe assembly.
    virtual double GetMass() const = 0;

    /// Set coefficient of friction.
    /// The default value is 0.8
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restiturion.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the DEM-P contact method is being used).
    /// The default values are: Y = 1e7 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the DEM-P contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get Young's modulus of elasticity for contact material.
    float GetYoungModulus() const { return m_young_modulus; }
    /// Get Poisson ratio for contact material.
    float GetPoissonRatio() const { return m_poisson_ratio; }
    /// Get normal stiffness coefficient for contact material.
    float GetKn() const { return m_kn; }
    /// Get tangential stiffness coefficient for contact material.
    float GetKt() const { return m_kt; }
    /// Get normal viscous damping coefficient for contact material.
    float GetGn() const { return m_gn; }
    /// Get tangential viscous damping coefficient for contact material.
    float GetGt() const { return m_gt; }

    /// Turn on/off collision flag for the shoe body.
    void SetCollide(bool val) { m_shoe->SetCollide(val); }

    /// Initialize this track shoe subsystem.
    /// The track shoe is created within the specified system and initialized
    /// at the specified location and orientation (expressed in the global frame).
    /// All actual work is deferred to derived classes (subsystem templates) which
    /// must create the bodies, joints, etc.
    virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,  ///< [in] handle to the chassis body
                            const ChVector<>& location,             ///< [in] location relative to the chassis frame
                            const ChQuaternion<>& rotation          ///< [in] orientation relative to the chassis frame
                            ) = 0;

    /// Connect this track shoe to the specified neighbor.
    /// This function must be called only after all track shoes have been initialized.
    virtual void Connect(std::shared_ptr<ChTrackShoe> next  ///< [in] handle to the neighbor track shoe
                         ) = 0;

  protected:
    /// Set the index of this track shoe within its containing track assembly.
    void SetIndex(size_t index) { m_index = index; }

    std::string m_name;              ///< name of the subsystem
    size_t m_index;                  ///< index of this track shoe within its containing track assembly
    std::shared_ptr<ChBody> m_shoe;  ///< handle to the shoe body

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping

    friend class ChTrackAssembly;
};

/// Vector of handles to track shoe subsystems.
typedef std::vector<std::shared_ptr<ChTrackShoe> > ChTrackShoeList;

/// @} vehicle_tracked_shoe

}  // end namespace vehicle
}  // end namespace chrono

#endif
