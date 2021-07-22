// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Definition of the base class TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM__TERRAINNODE_H
#define CH_VEHCOSIM__TERRAINNODE_H

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Base class for all terrain nodes.
class CH_VEHICLE_API ChVehicleCosimTerrainNode : public ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimTerrainNode() {}

    /// Set terrain patch dimensions (length and width).
    void SetPatchDimensions(double length,  ///< length in direction X (default: 2)
                            double width    ///< width in Y direction (default: 0.5)
    );

    /// Enable/disable run-time visualization (default: false).
    /// If enabled, rendering is done with the specified frequency.
    /// Note that a particular concrete terrain node may not support run-time visualization or may not render all physics elements.
    void EnableRuntimeVisualization(bool render, double render_fps = 100);

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override final;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override final;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override final;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override final;

    /// Return current number of contacts.
    /// (concrete terrain specific)
    virtual int GetNumContacts() const { return 0; }

  protected:
    /// Construct a base class terrain node.
    ChVehicleCosimTerrainNode();

    /// Specify whether or not flexible tire is supported.
    virtual bool SupportsFlexibleTire() const = 0;

    // ------------------------- Virtual methods for a flexible tire
    //     A derived class must implement these methods if SupportsFlexibleTire returns true.

    /// Create proxy bodies for a flexible tire mesh.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateMeshProxies() {
        if (SupportsFlexibleTire()) {
            throw ChException("Current terrain type does not support flexible tires!");
        }
    }
    /// Update the state of all proxy bodies for a flexible tire.
    /// Use information in the m_mesh_state struct (vertex positions and velocities expressed in absolute frame).
    virtual void UpdateMeshProxies() {
        if (SupportsFlexibleTire()) {
            throw ChException("Current terrain type does not support flexible tires!");
        }
    }
    /// Collect cumulative contact forces on all proxy bodies for a flexible tire.
    /// Load indices of vertices in contact and the corresponding vertex forces (expressed in absolute frame)
    /// into the m_mesh_contact struct.
    virtual void GetForcesMeshProxies() {
        if (SupportsFlexibleTire()) {
            throw ChException("Current terrain type does not support flexible tires!");
        }
    }

    // ------------------------- Virtual methods for a rigid tire.

    /// Create proxy body for a rigid tire mesh.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateWheelProxy() = 0;

    /// Update the state of the wheel proxy body for a rigid tire.
    /// Use information in the m_wheel_state struct (popse and velocities expressed in absolute frame).
    virtual void UpdateWheelProxy() = 0;
    
    /// Collect cumulative contact force and torque on the wheel proxy body.
    /// Load contact forces (expressed in absolute frame) into the m_wheel_contact struct.
    virtual void GetForceWheelProxy() = 0;

    // ------------------------- Other virtual methods

    /// Perform any additional operations after the initial data exchange with the rig node.
    virtual void OnInitialize() = 0;

    /// Perform any additional operations after the data exchange and synchronization with the rig node.
    virtual void OnSynchronize(int step_number, double time) {}

    /// Advance the state of the terrain system by the specified step.
    virtual void OnAdvance(double step_size) = 0;

    /// Perform additional output at the specified frame (called once per integration step).
    /// For example, output terrain-specific data for post-procesing.
    virtual void OnOutputData(int frame) {}

    /// Render simulation.
    virtual void OnRender(double time) {}

  protected:
    bool m_render;         ///< if true, perform run-time rendering
    double m_render_step;  ///< time step between rendered frames

    double m_hdimX;  ///< patch half-length (X direction)
    double m_hdimY;  ///< patch half-width (Y direction)

    // Communication data
    MeshData m_mesh_data;          ///< tire mesh data
    MeshState m_mesh_state;        ///< tire mesh state (used for flexible tire)
    MeshContact m_mesh_contact;    ///< tire mesh contact forces (used for flexible tire)
    WheelState m_wheel_state;      ///< wheel state (used for rigid tire)
    TerrainForce m_wheel_contact;  ///< wheel contact force (used for rigid tire)

    bool m_flexible_tire;      ///< flag indicating whether the tire is flexible or rigid
    double m_rig_mass;         ///< mass of the rig assembly
    MaterialInfo m_mat_props;  ///< tire contact material properties

    double m_init_height;  ///< initial terrain height (after optional settling)

  private:
    void SynchronizeRigidTire(int step_number, double time);
    void SynchronizeFlexibleTire(int step_number, double time);

    /// Print vertex and face connectivity data, as received from the rig node at synchronization.
    /// Invoked only for a flexible tire.
    void PrintMeshUpdateData();
};

}  // end namespace vehicle
}  // end namespace chrono

/// @} vehicle_cosim

#endif
