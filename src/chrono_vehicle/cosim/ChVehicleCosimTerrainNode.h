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
// Definition of the base vehicle co-simulation TERRAIN NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TERRAIN_NODE_H
#define CH_VEHCOSIM_TERRAIN_NODE_H

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim
/// @{

/// Base class for a terrain node.
/// Implements required functionality for a co-simulation node (Initialize(), Synchronize(), Advance()) and all MPI
/// communication.
///
/// A derived class must implement functions to:
/// - specify the communication interface type (SupportsMeshInterface())
/// - construct and initialize the concrete terrain object (OnInitialize())
/// - accept new body states (UpdateRigidProxy() and/or UpdateMeshProxy())
/// - provide terrain forces acting on bodies (GetForceRigidProxy() and/or GetForceMeshProxy())
/// - advance the dynamic state of the terrain (OnAdvance())
///
/// Optionally, a derived class may implement functions to:
/// - perform additional operations after a synchronization data exchange (OnSynchronize())
/// - perform additional data output (OnOutputData())
/// - provide run-time visualization (Render())
class CH_VEHICLE_API ChVehicleCosimTerrainNode : public ChVehicleCosimBaseNode {
  public:
    virtual ~ChVehicleCosimTerrainNode() {}

    /// Return the node type as NodeType::TERRAIN.
    virtual NodeType GetNodeType() const override { return NodeType::TERRAIN; }

    /// Enable/disable run-time visualization (default: false).
    /// If enabled, rendering is done with the specified frequency.
    /// Note that a particular concrete terrain node may not support run-time visualization or may not render all
    /// physics elements.
    void EnableRuntimeVisualization(bool render, double render_fps = 100);

    /// Set the terrain patch dimensions.
    /// If invoked, this function must be called before Initialize.
    void SetDimensions(double length, double width);

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

    /// Output post-processing visualization data.
    /// If implemented, this function should write a file in the "visualization" subdirectory of m_node_out_dir.
    virtual void OutputVisualizationData(int frame) override {}

    /// Return current number of contacts.
    /// (concrete terrain specific)
    virtual int GetNumContacts() const { return 0; }

  protected:
    /// Construct a terrain node to wrap a terrain patch of given length and width.
    ChVehicleCosimTerrainNode(double length, double width);

    // ------------------------- Virtual methods

    /// Specify whether or not the terrain node supports the MESH communication interface.
    /// See ChVehicleCosimBaseNode::InterfaceType.
    /// A terrain that also supports the MESH communication interface must override the functions UpdateMeshProxy()
    /// and GetForceMeshProxy().
    virtual bool SupportsMeshInterface() const = 0;

    /// Return the terrain initial height.
    /// This value must be available before the call to Initialize() (and therefore, before the derived class's
    /// OnInitialize() is called).
    virtual double GetInitHeight() const = 0;

    /// Perform any additional operations after the initial data exchange with the MBS node, including creating any
    /// required proxies for the specified number of objects. A derived class has access to the following vectors,
    /// each of size m_num_shapes:
    /// - m_aabb: collision model bounding box
    /// - m_load_mass: vertical load on each object
    /// - m_geometry: collision geometry and contact material for each object
    virtual void OnInitialize(unsigned int num_objects) = 0;

    /// Perform any additional operations after the data exchange and synchronization with the MBS node. A derived class
    /// has access to the following vectors (of size equal to the number of interacting objects):
    /// - full dynamic state of the rigid bodies (through m_rigid_state) when using the BODY communication interface
    /// - state of the mesh vertices (through m_mesh_state) when using the MESH communication interface
    virtual void OnSynchronize(int step_number, double time) {}

    /// Advance the state of the terrain system by the specified step.
    virtual void OnAdvance(double step_size) = 0;

    /// Perform additional output at the specified frame (called from within OutputData).
    virtual void OnOutputData(int frame) {}

    /// Render simulation.
    /// This function is called from Advance() at the frequency spoecified in the call to EnableRuntimeVisualization().
    /// Any call to Render occurs after a call to OnAdvance().
    virtual void Render(double time) {}

    // ------------------------- Virtual methods for the MESH communication interface
    // A derived class must implement these methods if SupportsMeshInterface returns true.

    /// Update the state of the i-th proxy mesh.
    /// Use information in the provided MeshState struct (vertex positions and velocities expressed in absolute frame).
    virtual void UpdateMeshProxy(unsigned int i, MeshState& mesh_state) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not support the MESH communication interface!");
        }
    }

    /// Collect cumulative contact forces on the i-th proxy mesh.
    /// Load indices of vertices in contact and the corresponding vertex forces (expressed in absolute frame)
    /// into the provided MeshContact struct.
    virtual void GetForceMeshProxy(unsigned int i, MeshContact& mesh_contact) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not the MESH communication interface!");
        }
    }

    // ------------------------- Virtual methods for the BODY communication interface

    /// Update the state of the i-th proxy rigid.
    /// Use information in the provided BodyState struct (pose and velocities expressed in absolute frame).
    virtual void UpdateRigidProxy(unsigned int i, BodyState& rigid_state) = 0;

    /// Collect cumulative contact force and torque on the i-th proxy rigid.
    /// Load contact forces (expressed in absolute frame) into the provided TerrainForce struct.
    virtual void GetForceRigidProxy(unsigned int i, TerrainForce& rigid_contact) = 0;

  protected:
    bool m_render;         ///< if true, perform run-time rendering
    double m_render_step;  ///< time step between rendered frames

    double m_hdimX;  ///< patch half-length (X direction)
    double m_hdimY;  ///< patch half-width (Y direction)

    // Communication data

    bool m_wheeled;                  ///< comm node (true: TIRE nodes, false: tracked MBS node)
    InterfaceType m_interface_type;  ///< communication interface (body or mesh)
    int m_num_objects;               ///< number of interacting objects

    std::vector<ChVehicleGeometry::AABB> m_aabb;  ///< AABB of collision models for interacting objects
    std::vector<ChVehicleGeometry> m_geometry;    ///< contact geometry and materials for interacting objects
    std::vector<double> m_load_mass;              ///< vertical load on interacting objects
    std::vector<int> m_obj_map;                   ///< mapping from interacting object to shape 

    std::vector<MeshState> m_mesh_state;        ///< mesh state (used for MESH communication)
    std::vector<BodyState> m_rigid_state;       ///< rigid state (used for BODY communication interface)
    std::vector<MeshContact> m_mesh_contact;    ///< mesh contact forces (used for MESH communication interface)
    std::vector<TerrainForce> m_rigid_contact;  ///< rigid contact force (used for BODY communication interface)

  private:
    void InitializeTireData();
    void InitializeTrackData();

    void SynchronizeWheeledBody(int step_number, double time);
    void SynchronizeTrackedBody(int step_number, double time);
    void SynchronizeWheeledMesh(int step_number, double time);
    void SynchronizeTrackedMesh(int step_number, double time);

    /// Print vertex and face connectivity data for the i-th object, as received at synchronization.
    /// Invoked only when using the MESH communication interface.
    void PrintMeshUpdateData(int i);
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
