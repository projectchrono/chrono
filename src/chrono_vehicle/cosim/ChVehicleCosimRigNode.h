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
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a terrain system.
//
// Definition of the RIG NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM__RIGNODE_H
#define CH_VEHCOSIM__RIGNODE_H

#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/fea/ChLoadContactSurfaceMesh.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// Mechanism for testing tires over granular terrain.
/// The mechanism + tire system is co-simulated with a terrain system.
class CH_VEHICLE_API ChVehicleCosimRigNode : public ChVehicleCosimBaseNode {
  public:
    enum class TireType { RIGID, FLEXIBLE, UNKNOWN };
    enum class ActuationType { SET_LIN_VEL, SET_ANG_VEL, UNKNOWN };

    virtual ~ChVehicleCosimRigNode();

    TireType GetTireType() const { return m_tire_type; }
    static std::string GetTireTypeAsString(TireType type);
    static TireType GetTireTypeFromString(const std::string& type);

    ActuationType GetActuationType() const { return m_act_type; }
    static std::string GetActuationTypeAsString(ActuationType type);
    static ActuationType GetActuationTypeFromString(const std::string& type);

    static bool ReadSpecfile(const std::string& specfile, rapidjson::Document& d);
    static TireType GetTireTypeFromSpecfile(const std::string& specfile);

    /// Set the number of OpenMP threads used in Chrono simulation (default: 1).
    void SetNumThreads(int num_threads);

    /// Set integrator and solver types.
    /// For the MKL solver, use slv_type = ChSolver::Type::CUSTOM.
    void SetIntegratorType(ChTimestepper::Type int_type,  ///< integrator type (default: HHT)
                           ChSolver::Type slv_type        ///< solver type (default:: MKL)
    );

    /// Set total rig system mass (default: 100).
    /// This represents the equivalent load on the soil from all rig bodies and the tire itself. Note that the total
    /// mass must be at least 2 kg more than the tire mass; otherwise, it will be overwritten.
    void SetTotalMass(double mass) { m_total_mass = mass; }

    /// Set (constant) toe angle in radians (default: 0).
    void SetToeAngle(double angle) { m_toe_angle = angle; }

    /// Specify the tire JSON specification file name.
    void SetTireFromSpecfile(const std::string& filename);

    /// Enable/disable tire pressure (default: true).
    void EnableTirePressure(bool val);

    /// Set window (in seconds) for the running average filter for drawbar pull reporting (default: 0.1 s).
    void SetDBPfilterWindow(double window) { m_dbp_filter_window = window; }

    /// Initialize this node.
    /// This function allows the node to initialize itself and, optionally, perform an
    /// initial data exchange with any other node.
    virtual void Initialize() override;

    /// Output logging and debugging data.
    virtual void OutputData(int frame) override;

  protected:
    /// Protected constructor. A RigNode cannot be directly created.
    ChVehicleCosimRigNode(TireType tire_type,      ///< tire type (RIGID or FLEXIBLE)
                          ActuationType act_type,  ///< actuation type (SET_LIN_VEL or SET_ANG_VEL)
                          double base_vel,         ///< constant linear or angular velocity
                          double slip              ///< desired longitudinal slip
    );

    TireType m_tire_type;      ///< tire type
    ActuationType m_act_type;  ///< actuation type

    ChSystemSMC* m_system;  ///< containing system
    bool m_constructed;     ///< system construction completed?

    ChTimestepper::Type m_int_type;                  ///< integrator type
    ChSolver::Type m_slv_type;                       ///< solver type
    std::shared_ptr<ChTimestepperHHT> m_integrator;  ///< HHT integrator object

    std::shared_ptr<ChBody> m_ground;   ///< ground body
    std::shared_ptr<ChBody> m_chassis;  ///< chassis body
    std::shared_ptr<ChBody> m_set_toe;  ///< set toe body
    std::shared_ptr<ChBody> m_upright;  ///< upright body
    std::shared_ptr<ChBody> m_spindle;  ///< spindle body

    double m_total_mass;  ///< total equivalent wheel mass

    double m_toe_angle;  ///< toe angle (controls tire slip angle)

    std::shared_ptr<ChWheel> m_wheel;  ///< wheel subsystem (to which a tire is attached)

    std::string m_tire_json;  ///< name of tire JSON specification file
    bool m_tire_pressure;     ///< tire pressure enabled?

    std::shared_ptr<ChLinkMotorRotationAngle> m_slip_motor;  ///< angular motor constraint
    std::shared_ptr<ChLinkLockPrismatic> m_prism_vel;        ///< prismatic joint for chassis linear velocity
    std::shared_ptr<ChLinkLinActuator> m_lin_actuator;       ///< actuator imposing linear velocity to system
    std::shared_ptr<ChLinkLockPrismatic> m_prism_axl;        ///< prismatic joint for chassis-axle joint
    std::shared_ptr<ChLinkMotorRotationAngle> m_rev_motor;   ///< motor enforcing prescribed rim angular vel

    double m_base_vel;  ///< base velocity (linear or angular, depending on actuation type)
    double m_slip;      ///< prescribed longitudinal slip for wheel
    double m_lin_vel;   ///< rig linear velocity (based on actuation type and slip value)
    double m_ang_vel;   ///< wheel angular velocity (based on actuation type and slip value)

    utils::ChRunningAverage* m_dbp_filter;  ///< running average filter for DBP
    double m_dbp_filter_window;             ///< window (span) for the DBP filter
    double m_dbp;                           ///< current value of filtered DBP

    // Communication data
    std::shared_ptr<ChMaterialSurfaceSMC> m_contact_mat;  ///< tire contact material
    MeshData m_mesh_data;                                 ///< tire mesh data
    MeshState m_mesh_state;                               ///< tire mesh state (used for flexible tire)
    MeshContact m_mesh_contact;                           ///< tire mesh contact forces (used for flexible tire)
    WheelState m_wheel_state;                             ///< wheel state (used for rigid tire)
    TerrainForce m_wheel_contact;                         ///< wheel contact force (used for rigid tire)

  private:
    void Construct();
    virtual void ConstructTire() = 0;
    virtual bool IsTireFlexible() const = 0;
    virtual double GetTireRadius() const = 0;
    virtual double GetTireWidth() const = 0;
    virtual double GetTireMass() const = 0;

    virtual void InitializeTire() = 0;

    /// Output tire-related statistics.
    virtual void OutputTireData(const std::string& del) = 0;

    /// Write state information for rig bodies and for the tire.
    void WriteBodyInformation(utils::CSV_writer& csv);
    virtual void WriteTireInformation(utils::CSV_writer& csv) = 0;
};

// =============================================================================

/// Rig node with a flexible tire.
class CH_VEHICLE_API ChVehicleCosimRigNodeFlexibleTire : public ChVehicleCosimRigNode {
  public:
    ChVehicleCosimRigNodeFlexibleTire(ActuationType act_type,  ///< actuation type (SET_LIN_VEL or SET_ANG_VEL)
                                      double base_vel,         ///< constant linear or angular velocity
                                      double slip              ///< desired longitudinal slip
                                      )
        : ChVehicleCosimRigNode(TireType::FLEXIBLE, act_type, base_vel, slip) {}

    ~ChVehicleCosimRigNodeFlexibleTire() {}

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

  private:
    /// Initialize the flexible tire (set tire mesh information and tire mass).
    virtual void InitializeTire() override;

    /// Construct the flexible tire.
    virtual void ConstructTire() override;
    virtual bool IsTireFlexible() const override { return true; }
    virtual double GetTireRadius() const { return m_tire->GetRadius(); }
    virtual double GetTireWidth() const { return m_tire->GetWidth(); }
    virtual double GetTireMass() const { return m_tire->ReportMass(); }

    /// Output tire-related statistics.
    virtual void OutputTireData(const std::string& del) override;

    /// Write tire-related information at current synchronization frame.
    virtual void WriteTireInformation(utils::CSV_writer& csv) override;

    /// Write mesh vertex positions and velocities.
    void WriteTireStateInformation(utils::CSV_writer& csv);
    /// Write mesh connectivity and strain information.
    void WriteTireMeshInformation(utils::CSV_writer& csv);
    /// Write contact forces on tire mesh vertices.
    void WriteTireContactInformation(utils::CSV_writer& csv);

    /// Print the current lowest node in the tire mesh.
    void PrintLowestNode();
    void PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel);
    void PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indices);

    std::shared_ptr<ChDeformableTire> m_tire;                       ///< deformable tire
    std::shared_ptr<fea::ChLoadContactSurfaceMesh> m_contact_load;  ///< tire contact surface
    std::vector<std::vector<unsigned int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<std::vector<unsigned int>> m_adjVertices;  ///< list of vertex indices for each mesh element
};

// =============================================================================

/// Rig node with a rigid tire.
class CH_VEHICLE_API ChVehicleCosimRigNodeRigidTire : public ChVehicleCosimRigNode {
  public:
    ChVehicleCosimRigNodeRigidTire(ActuationType act_type,  ///< actuation type (SET_LIN_VEL or SET_ANG_VEL)
                                   double base_vel,         ///< constant linear or angular velocity
                                   double slip              ///< desired longitudinal slip
                                   )
        : ChVehicleCosimRigNode(TireType::RIGID, act_type, base_vel, slip) {}

    ~ChVehicleCosimRigNodeRigidTire() {}

    /// Synchronize this node.
    /// This function is called at every co-simulation synchronization time to
    /// allow the node to exchange information with any other node.
    virtual void Synchronize(int step_number, double time) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;

  private:
    /// Construct the rigid tire.
    virtual void ConstructTire() override;
    virtual bool IsTireFlexible() const override { return false; }
    virtual double GetTireRadius() const { return m_tire->GetRadius(); }
    virtual double GetTireWidth() const { return m_tire->GetWidth(); }
    virtual double GetTireMass() const { return m_tire->ReportMass(); }

    /// Initialize the rigid tire (set tire mesh information and tire mass).
    virtual void InitializeTire() override;

    /// Output tire-related statistics.
    virtual void OutputTireData(const std::string& del) override;

    /// Write tire-related information at current synchronization frame.
    virtual void WriteTireInformation(utils::CSV_writer& csv) override;

    /// Write mesh vertex positions and velocities.
    void WriteTireStateInformation(utils::CSV_writer& csv);
    /// Write mesh connectivity.
    void WriteTireMeshInformation(utils::CSV_writer& csv);
    /// Write contact forces on tire mesh vertices.
    void WriteTireContactInformation(utils::CSV_writer& csv);

    std::shared_ptr<ChRigidTire> m_tire;                   ///< rigid tire
    std::vector<std::vector<unsigned int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<double> m_vertexArea;                      ///< representative areas for each mesh vertex
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
