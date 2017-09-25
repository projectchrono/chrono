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
// Granular terrain model.
// This class implements a rectangular patch of granular terrain.
// Optionally, a moving patch feature can be enable so that the patch is
// relocated (currently only in the positive X direction) based on the position
// of a user-specified body.
// Boundary conditions (model of a container bin) are imposed through a custom
// collision detection object.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#ifndef GRANULAR_TERRAIN_H
#define GRANULAR_TERRAIN_H

#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Granular terrain model.
/// This class implements a rectangular patch of granular terrain with spherical particles.
/// Boundary conditions (model of a container bin) are imposed through a custom collision
/// detection object.
class CH_VEHICLE_API GranularTerrain : public ChTerrain {
  public:
    /// Construct a default GranularTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    GranularTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                    );

    ~GranularTerrain() {}

    /// Set coefficient of friction.
    /// The default value is 0.9
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restitution.
    /// The default value is 0.
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

    /// Set the cohesion constant.
    /// The default value is 0.
    void SetContactCohesion(float cohesion) { m_cohesion = cohesion; }

    /// Set contact material properties.
    /// These values are used to calculate contact material coefficients (if the containing
    /// system is so configured and if the SMC contact method is being used).
    /// The default values are: Y = 2e5 and nu = 0.3
    void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                      float poisson_ratio   ///< [in] Poisson ratio
                                      );

    /// Set contact material coefficients.
    /// These values are used directly to compute contact forces (if the containing system
    /// is so configured and if the SMC contact method is being used).
    /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
    void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                        float gn,  ///< [in] normal contact damping
                                        float kt,  ///< [in] tangential contact stiffness
                                        float gt   ///< [in] tangential contact damping
                                        );

    /// Set outward collision envelope (default: 0).
    /// This value is used for the internal custom collision detection for imposing
    /// boundary conditions.  Note that if the underlying system is of SMC type (i.e.,
    /// using a penalty-based contact method), the envelope is automatically set to 0.
    void SetCollisionEnvelope(double envelope) { m_envelope = envelope; }

    /// Enable/disable verbose output (default: false).
    void EnableVerbose(bool val) { m_verbose = val; }

    /// Enable creation of particles fixed to bottom container.
    void EnableRoughSurface(int num_spheres_x,  ///< number of fixed spheres in X direction
                            int num_spheres_y   ///< number of fixed spheres in Y direction
                            );

    /// Enable moving patch and set parameters.
    void EnableMovingPatch(std::shared_ptr<ChBody> body,              ///< monitored body
                           double buffer_distance,                    ///< look-ahead distance
                           double shift_distance,                     ///< chunk size of relocated particles
                           const ChVector<>& init_vel = ChVector<>()  ///< initial particle velocity
                           );

    /// Set start value for body identifiers of generated particles (default: 1000000).
    /// It is assumed that all bodies with a larger identifier are granular material particles.
    void SetStartIdentifier(int id) { m_start_id = id; }

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get cohesion constant.
    float GetCohesion() const { return m_cohesion; }
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

    /// Enable/disable visualization of boundaries (default: false).
    void EnableVisualization(bool val) { m_vis_enabled = val; }
    bool IsVisualizationEnabled() const { return m_vis_enabled; }

    /// Set boundary visualization color.
    void SetColor(ChColor color) { m_color->SetColor(color); }

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the granular terrain system.
    /// The granular material is created in successive layers within the specified volume,
    /// using the specified generator, until the number of particles exceeds the specified value.
    /// The initial particle locations are obtained with Poisson Disk sampling, using the
    /// given minimum separation distance.
    void Initialize(const ChVector<>& center,                  ///< [in] center of bottom
                    double length,                             ///< [in] patch dimension in X direction
                    double width,                              ///< [in] patch dimension in Y direction
                    unsigned int num_particles,                ///< [in] minimum number of particles
                    double radius,                             ///< [in] particle radius
                    double density,                            ///< [in] particle density
                    const ChVector<>& init_vel = ChVector<>()  ///< [in] particle initial velocity
                    );

    /// Update the state of the terrain system at the specified time.
    virtual void Synchronize(double time) override;

    /// Get current front boundary location (in positive X direction).
    double GetPatchFront() const { return m_front; }
    /// Get current rear boundary location (in negative X direction).
    double GetPatchRear() const { return m_rear; }
    /// Get left boundary location (in positive Y direction).
    double GetPatchLeft() const { return m_left; }
    /// Get right boundary location (in negative Y direction).
    double GetPatchRight() const { return m_right; }
    /// Get bottom boundary location.
    double GetPatchBottom() const { return m_bottom; }

    /// Report if the patch was moved during the last call to Synchronize().
    bool PatchMoved() const { return m_moved; }

    /// Get the number of particles.
    unsigned int GetNumParticles() const { return m_num_particles; }

    /// Get the terrain height at the specified (x,y) location.
    /// This function returns the heighest point over all granular particles.
    virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    virtual chrono::ChVector<> GetNormal(double x, double y) const override { return ChVector<>(1, 0, 0); }

    /// Get the terrain coefficient of friction at the specified (x,y) location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For GranularTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified through SetContactFrictionCoefficient.
    virtual float GetCoefficientFriction(double x, double y) const override;

  private:
    unsigned int m_num_particles;  ///< requested minimum number of particles
    int m_start_id;                ///< start body identifier for particles
    double m_radius;               ///< particle radius

    // Patch dimensions
    double m_length;  ///< length (X direction) of granular patch
    double m_width;   ///< width (Y direction) of granular patch

    // Boundary locations
    double m_front;   ///< front (positive X) boundary location
    double m_rear;    ///< rear (negative X) boundary location
    double m_left;    ///< left (positive Y) boundary location
    double m_right;   ///< right (negative Y) boundary location
    double m_bottom;  ///< bottom boundary location

    // Moving patch parameters
    bool m_moving_patch;             ///< moving patch feature enabled?
    bool m_moved;                    ///< was the patch moved?
    std::shared_ptr<ChBody> m_body;  ///< tracked body
    double m_buffer_distance;        ///< minimum distance to front boundary
    double m_shift_distance;         ///< size (X direction) of relocated volume
    ChVector<> m_init_part_vel;      ///< initial particle velocity

    // Rough surface (ground-fixed spheres)
    bool m_rough_surface;  ///< rough surface feature enabled?
    int m_nx;              ///< number of fixed spheres in X direction
    int m_ny;              ///< number of fixed spheres in Y direction
    double m_sep_x;        ///< separation distance in X direction
    double m_sep_y;        ///< separation distance in Y direction

    // Collision envelope used in custom collision detection
    double m_envelope;  ///< collision outward envelope

    bool m_vis_enabled;                     ///< boundary visualization enabled?
    std::shared_ptr<ChBody> m_ground;       ///< ground body
    std::shared_ptr<ChColorAsset> m_color;  ///< color of boundary visualization asset

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_cohesion;       ///< contact cohesion constant
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping

    bool m_verbose;  ///< verbose output

    friend class BoundaryContact;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
