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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band track assembly using an ANCFshell-based web
// (template definition).
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_TRACK_ASSEMBLY_BAND_ANCF_H
#define CH_TRACK_ASSEMBLY_BAND_ANCF_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblyBand.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked
/// @{

/// Definition of a continuous band track assembly using an ANCFshell-based web
/// A track assembly consists of a sprocket, an idler (with tensioner mechanism),
/// a set of suspensions (road-wheel assemblies), and a collection of track shoes.
/// This class defines the template for a track assembly using a web modeled as
/// an FEA mesh with ANCF shell elements.
class CH_VEHICLE_API ChTrackAssemblyBandANCF : public ChTrackAssemblyBand {
  public:
    /// Type of the mesh contact surface.
    enum ContactSurfaceType { NONE, NODE_CLOUD, TRIANGLE_MESH };

    /// Construct an ANCFshell-based track assembly on the specified vehicle side.
    ChTrackAssemblyBandANCF(const std::string& name,  ///< [in] name of the subsystem
                            VehicleSide side          ///< [in] assembly on left/right vehicle side
    );

    virtual ~ChTrackAssemblyBandANCF();

    /// Set the type of contact surface (default: TRIANGLE_MESH).
    void SetContactSurfaceType(ContactSurfaceType type) { m_contact_type = type; }

    /// Get the number of track shoes.
    virtual size_t GetNumTrackShoes() const override { return m_shoes.size(); }

    /// Get a handle to the specified track shoe subsystem.
    virtual std::shared_ptr<ChTrackShoe> GetTrackShoe(size_t id) const override { return m_shoes[id]; }

  protected:
    ChTrackShoeBandANCFList m_shoes;            ///< track shoes
    std::shared_ptr<fea::ChMesh> m_track_mesh;  ///< web mesh
    ContactSurfaceType m_contact_type;          ///< type of contact surface model (node cloud or mesh)

  private:
    /// Custom callback class for culling broadphase collisions.
    class BroadphaseCulling : public collision::ChCollisionSystem::BroadphaseCallback {
      public:
        BroadphaseCulling(ChTrackAssemblyBandANCF* assembly);

      private:
        virtual bool OnBroadphase(collision::ChCollisionModel* modelA, collision::ChCollisionModel* modelB) override;

        ChTrackAssemblyBandANCF* m_assembly;
    };

    /// Assemble track shoes over wheels.
    /// Return true if the track shoes were initialized in a counter clockwise
    /// direction and false otherwise.
    virtual bool Assemble(std::shared_ptr<ChBodyAuxRef> chassis) override final;

    /// Add visualization assets for the track assembly subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override final;

    /// Remove visualization assets for the track shoe subsystem.
    virtual void RemoveVisualizationAssets() override final;

    BroadphaseCulling* m_callback;  ///< custom broadphase callback object

    friend class BroadphaseCulling;
};

/// @} vehicle_tracked

}  // end namespace vehicle
}  // end namespace chrono

#endif
