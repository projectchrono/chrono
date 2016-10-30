// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Definition of a TIRE NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef HMMWV_COSIM_TIRENODE_H
#define HMMWV_COSIM_TIRENODE_H

#include <vector>
#include <array>

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FEATire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

#include "BaseNode.h"

// Forward declaration
class TireBase;

// =============================================================================

/// Definition of an MPI node responsible for tire co-simulation.
/// A tire node communicates with the vehicle node (RECV rim body states and SEND
/// tire forces on the rim body) and with the terrain node (SEND tire mesh state
/// and RECV contact forces on mesh vertices).
class TireNode : public BaseNode {
  public:
    TireNode(const std::string& json_filename,   ///< JSON tire specification file
             chrono::vehicle::WheelID wheel_id,  ///< id of associated wheel
             int num_threads                     ///< number of OpenMP threads
             );
    ~TireNode();

    /// Set properties of the rim proxy body.
    void SetProxyProperties(double mass,                        ///< mass of the rim proxy body
                            const chrono::ChVector<>& inertia,  ///< moments of inertia of the rim proxy body
                            bool fixed                          ///< proxy body fixed to ground? (default: false)
                            );

    /// Enable/disable tire pressure (default: true).
    /// Ignored for RIGID tire.
    void EnableTirePressure(bool val) { m_tire_pressure = val; }

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override;

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override;

    /// Control log detail
    void SetVerboseStates(bool val) { m_verbose_states = val; }
    void SetVerboseForces(bool val) { m_verbose_forces = val; }
    void SetVerboseSolver(bool val) { m_verbose_solver = val; }

  private:
    enum Type { ANCF, FEA, RIGID };

    Type m_type;  ///< tire type (FEA, i.e. co-rotational not yet supported)

    chrono::ChSystemDEM* m_system;  ///< containing system

    std::shared_ptr<chrono::ChTimestepperHHT> m_integrator;  ///< HHT integrator object

    chrono::vehicle::WheelID m_wheel_id;    ///< id of associated vehicle wheel
    std::shared_ptr<chrono::ChBody> m_rim;  ///< wheel rim body
    double m_rim_mass;                      ///< mass of wheel body
    chrono::ChVector<> m_rim_inertia;       ///< inertia of wheel body
    bool m_rim_fixed;                       ///< flag indicating whether or not the rim proxy is fixed to ground

    std::string m_tire_json;  ///< name of tire JSON specification file
    bool m_tire_pressure;     ///< tire pressure enabled?

    TireBase* m_tire_wrapper;  ///< wrapper object for a Chrono::Vehicle tire

    // Current contact forces on tire mesh vertices
    std::vector<int> m_vert_indices;                ///< indices of vertices experiencing contact forces
    std::vector<chrono::ChVector<>> m_vert_pos;     ///< position of vertices experiencing contact forces
    std::vector<chrono::ChVector<>> m_vert_forces;  ///< contact forces on mesh vertices

    // Verbose level
    bool m_verbose_states;
    bool m_verbose_forces;
    bool m_verbose_solver;

    // Private methods
    void PrintLowestVertex(const std::vector<chrono::ChVector<>>& vert_pos,
                           const std::vector<chrono::ChVector<>>& vert_vel);
    void PrintContactData(const std::vector<chrono::ChVector<>>& forces, const std::vector<int>& indices);
};

// =============================================================================

/// Interface class for a tire wrapper.
class TireBase {
  public:
    virtual ~TireBase() {}

    /// Get handle to underlying ChTire.
    virtual std::shared_ptr<chrono::vehicle::ChTire> GetTire() const = 0;

    /// Initialize underlying tire and return surface contact properties.
    virtual void Initialize(std::shared_ptr<chrono::ChBody> rim,
                            chrono::vehicle::VehicleSide side,
                            std::array<int, 2>& surf_props,
                            std::array<float, 8>& mat_props) = 0;

    /// Extract and return current tire mesh state (SEND to terrain node).
    virtual void GetMeshState(std::vector<chrono::ChVector<>>& vert_pos,
                              std::vector<chrono::ChVector<>>& vert_vel,
                              std::vector<chrono::ChVector<int>>& triangles) = 0;

    /// Apply contact forces on tire mesh (RECV from terrain node).
    virtual void SetContactForces(std::shared_ptr<chrono::ChBody> rim,
                                  const std::vector<int>& vert_indices,
                                  const std::vector<chrono::ChVector<>>& vert_pos,
                                  const std::vector<chrono::ChVector<>>& vert_forces) = 0;

    /// Extract and return current tire forces on rim body (SEND to vehicle node).
    virtual void GetTireForce(chrono::vehicle::TireForce& tire_force) = 0;

    /// Callback invoked before taking a new step.
    virtual void OnAdvance() = 0;

    /// Append tire-specific solution stats in cumulative output stream.
    virtual void OutputData(std::ofstream& outf, const std::string& del) = 0;

    /// Write mesh node state information.
    virtual void WriteStateInformation(chrono::utils::CSV_writer& csv) = 0;
    /// Write mesh connectivity and strain information.
    virtual void WriteMeshInformation(chrono::utils::CSV_writer& csv) = 0;
    /// Write contact forces on tire mesh vertices.
    virtual void WriteContactInformation(chrono::utils::CSV_writer& csv,
                                         std::shared_ptr<chrono::ChBody> rim,
                                         const std::vector<int>& vert_indices,
                                         const std::vector<chrono::ChVector<>>& vert_pos,
                                         const std::vector<chrono::ChVector<>>& vert_forces) = 0;
};

/// Deformable (ANCF) tire wrapper.
class TireANCF : public TireBase {
  public:
    TireANCF(const std::string& json, bool enable_pressure);

    virtual std::shared_ptr<chrono::vehicle::ChTire> GetTire() const override { return m_tire; }

    virtual void Initialize(std::shared_ptr<chrono::ChBody> rim,
                            chrono::vehicle::VehicleSide side,
                            std::array<int, 2>& surf_props,
                            std::array<float, 8>& mat_props) override;

    virtual void GetMeshState(std::vector<chrono::ChVector<>>& vert_pos,
                              std::vector<chrono::ChVector<>>& vert_vel,
                              std::vector<chrono::ChVector<int>>& triangles) override;

    virtual void SetContactForces(std::shared_ptr<chrono::ChBody> rim,
                                  const std::vector<int>& vert_indices,
                                  const std::vector<chrono::ChVector<>>& vert_pos,
                                  const std::vector<chrono::ChVector<>>& vert_forces) override;

    virtual void GetTireForce(chrono::vehicle::TireForce& tire_force) override;

    virtual void OnAdvance() override;

    virtual void OutputData(std::ofstream& outf, const std::string& del) override;
    virtual void WriteStateInformation(chrono::utils::CSV_writer& csv) override;
    virtual void WriteMeshInformation(chrono::utils::CSV_writer& csv) override;
    virtual void WriteContactInformation(chrono::utils::CSV_writer& csv,
                                         std::shared_ptr<chrono::ChBody> rim,
                                         const std::vector<int>& vert_indices,
                                         const std::vector<chrono::ChVector<>>& vert_pos,
                                         const std::vector<chrono::ChVector<>>& vert_forces) override;

  private:
    std::shared_ptr<chrono::vehicle::ChANCFTire> m_tire;                    ///< deformable ANCF tire
    std::shared_ptr<chrono::fea::ChLoadContactSurfaceMesh> m_contact_load;  ///< tire contact surface

    std::vector<std::vector<int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<std::vector<int>> m_adjVertices;  ///< list of vertex indices for each mesh element
};

/// Rigid (mesh) tire wrapper.
class TireRigid : public TireBase {
  public:
    TireRigid(const std::string& json);

    virtual std::shared_ptr<chrono::vehicle::ChTire> GetTire() const override { return m_tire; }

    virtual void Initialize(std::shared_ptr<chrono::ChBody> rim,
                            chrono::vehicle::VehicleSide side,
                            std::array<int, 2>& surf_props,
                            std::array<float, 8>& mat_props) override;

    virtual void GetMeshState(std::vector<chrono::ChVector<>>& vert_pos,
                              std::vector<chrono::ChVector<>>& vert_vel,
                              std::vector<chrono::ChVector<int>>& triangles) override;

    virtual void SetContactForces(std::shared_ptr<chrono::ChBody> rim,
                                  const std::vector<int>& vert_indices,
                                  const std::vector<chrono::ChVector<>>& vert_pos,
                                  const std::vector<chrono::ChVector<>>& vert_forces) override;

    virtual void GetTireForce(chrono::vehicle::TireForce& tire_force) override;

    virtual void OnAdvance() override {}

    virtual void OutputData(std::ofstream& outf, const std::string& del) override {}
    virtual void WriteStateInformation(chrono::utils::CSV_writer& csv) override;
    virtual void WriteMeshInformation(chrono::utils::CSV_writer& csv) override;
    virtual void WriteContactInformation(chrono::utils::CSV_writer& csv,
                                         std::shared_ptr<chrono::ChBody> rim,
                                         const std::vector<int>& vert_indices,
                                         const std::vector<chrono::ChVector<>>& vert_pos,
                                         const std::vector<chrono::ChVector<>>& vert_forces) override;

  private:
    std::shared_ptr<chrono::vehicle::ChRigidTire> m_tire;  ///< rigid tire
    chrono::vehicle::TireForce m_tire_force;               ///< accumulated tire force

    std::vector<std::vector<int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<double> m_vertexArea;             ///< representative areas for each mesh vertex
};

#endif
