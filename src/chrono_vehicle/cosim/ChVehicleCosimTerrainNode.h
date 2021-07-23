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
    /// Type of the communication interface for the terrain node.
    /// - A BODY interface assumes communication is done at the wheel spindle level.  At a synchronization time, the
    /// terrain node receives the full state of the spindle body and must send forces acting on the spindle, for each
    /// tire present in the simulation.  This type of interface should be used for a rigid tire or when the terrain node
    /// also performs the dynamics of a flexible tire.
    /// - A MESH interface assumes communication is done at the tire mesh level. At a synchronization time, the terrain
    /// node receives the tire mesh vertex states (positions and velocities) are must send forces acting on vertices of
    /// the mesh, for each tire. This tire interface is typically used when flexible tires are simulated outside the
    /// terrain node (either on the multibody node or else on separate tire nodes).
    enum class InterfaceType {
        BODY,  ///< exchange state and force for a single body (wheel spindle)
        MESH   ///< exchange state and force for a mesh (flexible tire mesh)
    };

    virtual ~ChVehicleCosimTerrainNode() {}

    /// Return the node type as NodeType::TERRAIN.
    virtual NodeType GetNodeType() const override { return NodeType::TERRAIN; }

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
    ChVehicleCosimTerrainNode(unsigned int num_tires = 1);

    /// Specify whether or not the terrain node supports the MESH communication interface.
    /// See ChVehicleCosimBaseNode::InterfaceType.
    /// A terrain that also supports the MESH communication interface must override the functions UpdateMeshProxies and
    /// GetForcesMeshProxies.
    virtual bool SupportsMeshInterface() const = 0;

    // ------------------------- Virtual methods for the MESH communication interface
    //     A derived class must implement these methods if SupportsMeshInterface returns true.

    /// Create proxy bodies for the i-th tire mesh.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateMeshProxies(unsigned int i) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not support the MESH communication interface!");
        }
    }
    /// Update the state of all proxy bodies for the i-th tire mesh.
    /// Use information in the provided MeshState struct (vertex positions and velocities expressed in absolute frame).
    virtual void UpdateMeshProxies(unsigned int i, const MeshState& mesh_state) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not support the MESH communication interface!");
        }
    }
    /// Collect cumulative contact forces on all proxy bodies for the i-th tire mesh.
    /// Load indices of vertices in contact and the corresponding vertex forces (expressed in absolute frame)
    /// into the provided MeshContact struct.
    virtual void GetForcesMeshProxies(unsigned int i, MeshContact& mesh_contact) {
        if (SupportsMeshInterface()) {
            throw ChException("Current terrain type does not the MESH communication interface!");
        }
    }

    // ------------------------- Virtual methods for the BODY communication interface

    /// Create proxy body for the i-th tire.
    /// Use information in the m_mesh_data struct (vertex positions expressed in local frame).
    virtual void CreateWheelProxy(unsigned int i) = 0;

    /// Update the state of the wheel proxy body for the i-th tire.
    /// Use information in the provided WheelState struct (pose and velocities expressed in absolute frame).
    virtual void UpdateWheelProxy(unsigned int i, const WheelState& wheel_state) = 0;
    
    /// Collect cumulative contact force and torque on the wheel proxy body for the i-th tire.
    /// Load contact forces (expressed in absolute frame) into the provided TerrainForce struct.
    virtual void GetForceWheelProxy(unsigned int i, TerrainForce& wheel_contact) = 0;

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
    unsigned int m_num_tires;  ///< number of tires

    bool m_render;         ///< if true, perform run-time rendering
    double m_render_step;  ///< time step between rendered frames

    double m_hdimX;        ///< patch half-length (X direction)
    double m_hdimY;        ///< patch half-width (Y direction)
    double m_init_height;  ///< initial terrain height (after optional settling)

    // Communication data

    InterfaceType m_interface_type; ///< type of communication interface

    std::vector<double> m_load_mass;        ///< vertical load on tire
    std::vector<MaterialInfo> m_mat_props;  ///< tire contact material properties

    std::vector<MeshData> m_mesh_data;          ///< tire mesh data
    std::vector<MeshState> m_mesh_state;        ///< tire mesh state (used for MESH communication)
    std::vector<MeshContact> m_mesh_contact;    ///< tire mesh contact forces (used for MESH communication interface)
    std::vector<WheelState> m_wheel_state;      ///< wheel state (used for BODY communication interface)
    std::vector<TerrainForce> m_wheel_contact;  ///< wheel contact force (used for BODY communication interface)

  private:
    int PartnerRank(unsigned int i);
    void SynchronizeBody(int step_number, double time);
    void SynchronizeMesh(int step_number, double time);

    /// Print vertex and face connectivity data for the i-th tire, as received at synchronization.
    /// Invoked only when using the MESH communicatin interface.
    void PrintMeshUpdateData(unsigned int i);
};

}  // end namespace vehicle
}  // end namespace chrono

/// @} vehicle_cosim

#endif
