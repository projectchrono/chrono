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

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

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

    ~GranularTerrain();

    /// Set contact material (must be consistent with the containing system).
    void SetContactMaterial(std::shared_ptr<ChMaterialSurface> material) { m_material = material; }

    /// Get the current contact material.
    std::shared_ptr<ChMaterialSurface> GetContactMaterial() const { return m_material; }

    /// Set outward collision envelope.
    /// This value is used for the internal custom collision detection for imposing
    /// boundary conditions.  Note that if the underlying system is of SMC type (i.e.,
    /// using a penalty-based contact method), the envelope is automatically set to 0.
    /// For NSC systems (i.e., when using a complementarity-based contact method), if
    /// the envelope is not specified, the default value is 5% of the particle radius.
    void SetCollisionEnvelope(double envelope) { m_envelope = envelope; }

    /// Set the minimum number of particles to be generated (default: 0).
    void SetMinNumParticles(unsigned int min_num_particles) { m_min_num_particles = min_num_particles; }

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

    /// Enable/disable visualization of boundaries (default: false).
    void EnableVisualization(bool val) { m_vis_enabled = val; }
    bool IsVisualizationEnabled() const { return m_vis_enabled; }

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the granular terrain system.
    /// The granular material is created in successive layers within the specified volume,
    /// using the specified generator, until the number of particles exceeds the specified
    /// minimum value (see SetMinNumParticles).
    /// The initial particle locations are obtained with Poisson Disk sampling, using the
    /// given minimum separation distance.
    void Initialize(const ChVector<>& center,                  ///< [in] center of bottom
                    double length,                             ///< [in] patch dimension in X direction
                    double width,                              ///< [in] patch dimension in Y direction
                    unsigned int num_layers,                   ///< [in] number of layers
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

    /// Get the terrain height below the specified location.
    /// This function returns the highest point over all granular particles.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual chrono::ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For GranularTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified through SetContactFrictionCoefficient.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

  private:
    unsigned int m_min_num_particles;  ///< requested minimum number of particles
    unsigned int m_num_particles;      ///< actual number of particles
    int m_start_id;                    ///< start body identifier for particles
    double m_radius;                   ///< particle radius

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

    bool m_vis_enabled;                ///< boundary visualization enabled?
    std::shared_ptr<ChBody> m_ground;  ///< ground body

    std::shared_ptr<ChMaterialSurface> m_material;  ///< contact material properties

    std::shared_ptr<ChSystem::CustomCollisionCallback> m_collision_callback;  ///< custom collision callback

    bool m_verbose;  ///< verbose output

    friend class BoundaryContact;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
