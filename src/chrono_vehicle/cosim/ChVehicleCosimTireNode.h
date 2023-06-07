// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Definition of the base vehicle co-simulation TIRE NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TIRE_NODE_H
#define CH_VEHCOSIM_TIRE_NODE_H

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

namespace chrono {
namespace vehicle {

/** @addtogroup vehicle_cosim_tire
 *
 * This module defines concrete tire nodes using Chrono physics:
 * - ChVehicleCosimTireNodeRigid wraps a rigid tire model and implements the ChVehicleCosimBaseNode::InterfaceType::BODY
 * communication interface.
 * - ChVehicleCosimTireNodeFlexible wraps a flexible FEA-based tire model and implements the
 * ChVehicleCosimBaseNode::InterfaceType::MESH communication interface.
 * - ChVehicleCosimTireNodeBypass is a simple counduit between the MBS and Terrain node and implements the
 * ChVehicleCosimBaseNode::InterfaceType::BODY communication interface.
 */

/// @addtogroup vehicle_cosim
/// @{

/// Base class for all tire nodes.
class CH_VEHICLE_API ChVehicleCosimTireNode : public ChVehicleCosimBaseNode {
  public:
    /// Tire type.
    enum class TireType {
        RIGID,     ///< rigid tire
        FLEXIBLE,  ///< flexible tire
        BYPASS,    ///< bypass (dummy) tire
        UNKNOWN    ///< unknown tire type
    };

    virtual ~ChVehicleCosimTireNode() {}

    /// Return the node type as NodeType::TIRE.
    virtual NodeType GetNodeType() const override final { return NodeType::TIRE; }

    /// Return the tire type.
    virtual TireType GetTireType() const = 0;

    /// Return a string describing the tire type.
    static std::string GetTireTypeAsString(TireType type);

    /// Infer the tire type from the given string.
    static TireType GetTireTypeFromString(const std::string& type);

    /// Read a JSON specification file for a tire.
    static bool ReadSpecfile(const std::string& specfile, rapidjson::Document& d);

    /// Get the tire type from the given JSON specification file.
    static TireType GetTireTypeFromSpecfile(const std::string& specfile);

    /// Enable/disable tire pressure (default: true).
    void EnableTirePressure(bool val);

    /// Set the number of OpenMP threads used in Chrono simulation (default: 1).
    void SetNumThreads(int num_threads);

    /// Set integrator and solver types.
    /// For the MKL solver, use slv_type = ChSolver::Type::CUSTOM.
    void SetIntegratorType(ChTimestepper::Type int_type,  ///< integrator type (default: HHT)
                           ChSolver::Type slv_type        ///< solver type (default:: MKL)
    );

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override final;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override final;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override final;

  protected:
    ChVehicleCosimTireNode(int index, const std::string& tire_json = "");

    /// Specify the type of communication interface (BODY or MESH) required by this the tire node.
    /// See ChVehicleCosimBaseNode::InterfaceType.
    virtual InterfaceType GetInterfaceType() const = 0;

    /// Return the tire mass.
    virtual double GetTireMass() const { return m_tire->GetMass(); }

    /// Return the tire radius.
    virtual double GetTireRadius() const { return m_tire->GetRadius(); }

    /// Return the tire width.
    virtual double GetTireWidth() const { return m_tire->GetWidth(); }

    /// Initialize the tire by attaching it to the provided ChWheel.
    /// A derived class must load m_geometry (collision shape and contact material).
    virtual void InitializeTire(std::shared_ptr<ChWheel>, const ChVector<>& init_loc) = 0;

    /// Apply the spindle state.
    /// The BodyState struct contains the spindle body state as received from the MBS node.
    virtual void ApplySpindleState(const BodyState& spindle_state) {}

    /// Load current spindle force.
    /// A derived class which implements the MESH communication interface must override this function and must load the
    /// provided TerrainForce struct to be sent to the MBS node.
    virtual void LoadSpindleForce(TerrainForce& spindle_force) {
        if (GetInterfaceType() == InterfaceType::MESH) {
            throw ChException("Current tire does not properly implement the MESH communication interface!");
        }
    }
 
    /// Apply the spindle force (BODY communication interface).
    /// The TerrainForce struct contains the terrain forces applied to the spindle as received from the TERRAIN node.
    virtual void ApplySpindleForce(const TerrainForce& spindle_force) {}

    /// Load current tire mesh state.
    /// A derived class which implements the MESH communication interface must override this function and must load the
    /// provided MeshState struct to be sent to the TERRAIN node.
    virtual void LoadMeshState(MeshState& mesh_state) {
        if (GetInterfaceType() == InterfaceType::MESH) {
            throw ChException("Current tire does not properly implement the MESH communication interface!");
        }
    }

    /// Apply the mesh contact forces.
    /// A derived class which implements the MESH communication interface must override this function and must use the
    /// MeshContact struct received from the TERRAIN node.
    virtual void ApplyMeshForces(const MeshContact& mesh_contact) {
        if (GetInterfaceType() == InterfaceType::MESH) {
            throw ChException("Current tire does not properly implement the MESH communication interface!");
        }
    }

    /// Perform additional output at the specified frame (called once per integration step).
    virtual void OnOutputData(int frame) {}

  protected:
    ChSystemSMC* m_system;                           ///< containing system
    ChTimestepper::Type m_int_type;                  ///< integrator type
    ChSolver::Type m_slv_type;                       ///< solver type
    std::shared_ptr<ChTimestepperHHT> m_integrator;  ///< HHT integrator object

    bool m_tire_pressure;     ///< tire pressure enabled?
    int m_index;              ///< index of the tire

    std::shared_ptr<ChBody> m_spindle;  ///< spindle body
    std::shared_ptr<ChWheel> m_wheel;   ///< wheel subsystem (to which a tire is attached)
    std::shared_ptr<ChTire> m_tire;     ///< tire subsystem

    // Communication data (loaded by derived classes)
    ChVehicleGeometry m_geometry;  ///< tire geometry and contact material

  private:
    virtual ChSystem* GetSystemPostprocess() const override { return m_system; }
    void InitializeSystem();
    void SynchronizeBody(int step_number, double time);
    void SynchronizeMesh(int step_number, double time);
};

/// @} vehicle_cosim

}  // end namespace vehicle
}  // end namespace chrono

#endif
