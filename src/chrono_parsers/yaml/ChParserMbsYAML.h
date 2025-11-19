// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_PARSER_MBS_YAML_H
#define CH_PARSER_MBS_YAML_H

#include <vector>
#include <unordered_map>

#include "chrono_parsers/yaml/ChParserYAML.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/utils/ChBodyGeometry.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

// -----------------------------------------------------------------------------

class ChParserMbsYAML;

/// Base class for an external controller to apply loads on a body.
/// A load controller must have a corresponding specification in the model YAML file.
/// A derived controller object can be attached for any given instance of a model created from the model YAML file.
/// Attached controllers are processed and loads applied to the associated bodies in ChParserMbsYAML::DoStepDynamics.
class ChLoadController {
  public:
    virtual ~ChLoadController() {}

    virtual void Initialize(const ChParserMbsYAML& parser, int model_instance) {}
    virtual void Synchronize(double time) {}
    virtual void Advance(double time_step) {}

    virtual ChVector3d GetLoad() const = 0;
};

/// List of load controller names and their associated loads.
typedef std::unordered_map<std::string, ChVector3d> LoadControllerLoads;

/// Base class for an external controller to actuate a ChLinkMotor.
/// A motor controller must have a corresponding specification in the model YAML file.
/// A derived controller object can be attached for any given instance of a model created from the model YAML file.
/// Attached controllers are processed and actuations applied to the associated motors in
/// ChParserMbsYAML::DoStepDynamics.
class ChMotorController {
  public:
    virtual ~ChMotorController() {}

    virtual void Initialize(const ChParserMbsYAML& parser, int model_instance) {}
    virtual void Synchronize(double time) {}
    virtual void Advance(double time_step) {}

    virtual double GetActuation() const = 0;
};

/// List of motor controller names and their associated loads.
typedef std::unordered_map<std::string, double> MotorControllerActuations;

// -----------------------------------------------------------------------------

/// Parser for YAML specification files for Chrono models and simulations.
/// The parser caches model information and simulation settings from the corresponding YAML input files and then allows
/// populating a Chrono system and setting solver and simulation parameters.
class ChApiParsers ChParserMbsYAML : public ChParserYAML {
  public:
    ChParserMbsYAML(bool verbose = false);

    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserMbsYAML(const std::string& yaml_model_filename, const std::string& yaml_sim_filename, bool verbose = false);
    ~ChParserMbsYAML();

    /// Return true if a YAML simulation file has been loaded.
    bool HasSimulationData() const { return m_sim_loaded; }

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // --------------

    /// Load the simulation parameters from the specified input YAML simulation file.
    void LoadSimulationFile(const std::string& yaml_filename);

    double GetTimestep() const { return m_sim.time_step; }
    double GetEndtime() const { return m_sim.end_time; }
    bool EnforceRealtime() const { return m_sim.enforce_realtime; }

    bool Render() const { return m_sim.visualization.render; }
    double GetRenderFPS() const { return m_sim.visualization.render_fps; }
    CameraVerticalDir GetCameraVerticalDir() const { return m_sim.visualization.camera_vertical; }
    const ChVector3d& GetCameraLocation() const { return m_sim.visualization.camera_location; }
    const ChVector3d& GetCameraTarget() const { return m_sim.visualization.camera_target; }
    bool EnableShadows() const { return m_sim.visualization.enable_shadows; }

    /// Create and return a Chrono system configured from cached simulation parameters.
    /// If no YAML simulation file was loaded, this function returns a ChSystemNSC with default settings.
    std::shared_ptr<ChSystem> CreateSystem();

    /// Set solver and integrator settings from cached values.
    /// If no YAML simulation file was loaded, this function is a no-op.
    void SetSimulationParameters(ChSystem& sys);

    // --------------

    /// Load the model from the specified input YAML model file.
    void LoadModelFile(const std::string& yaml_filename);

    /// Populate the given system with the cached Chrono components.
    /// An instance of the underlying Chrono model can be created at the specified frame (relative to the global frame),
    /// with all Chrono object names using the specified prefix. Throws an error if no YAML model file was loaded.
    int Populate(ChSystem& sys, const ChFramed& model_frame = ChFramed(), const std::string& model_prefix = "");

    /// Remove from the specified system the Chrono objects from the specified instance.
    void Depopulate(ChSystem& sys, int instance_index);

    /// Return the number of instances created from the YAML model file.
    int GetNumInstances() const { return m_crt_instance + 1; }

    /// Find and return the body with specified name in the current model instance.
    std::shared_ptr<ChBodyAuxRef> FindBodyByName(const std::string& name) const;

    /// Find and return the body with specified name in the given model instance.
    std::shared_ptr<ChBodyAuxRef> FindBodyByName(const std::string& name, int model_instance) const;

    /// Find and return bodies with given base name from all model instances.
    std::vector<std::shared_ptr<ChBodyAuxRef>> FindBodiesByName(const std::string& name) const;

    /// Find and return the motor with specified name in the current model instance.
    std::shared_ptr<ChLinkMotor> FindMotorByName(const std::string& name) const;

    /// Find and return the motor with specified name in the given model instance.
    std::shared_ptr<ChLinkMotor> FindMotorByName(const std::string& name, int model_instance) const;

    /// Find and return motors with given base name from all model instances.
    std::vector<std::shared_ptr<ChLinkMotor>> FindMotorsByName(const std::string& name) const;

    // --------------

    /// Attach the external controller for the load controller with given name.
    /// This function can be called only after the MBS model was loaded and the model specification must include
    /// parameters for a load controller with specified name.
    void AttachLoadController(std::shared_ptr<ChLoadController> controller,
                              const std::string& name,
                              int model_instance);

    /// Attach the external controller for the motor controller with given name.
    /// This function can be called only after the MBS model was loaded, the model specification must include
    /// parameters for a motor with specified name, and that motor was set as externally actuated.
    void AttachMotorController(std::shared_ptr<ChMotorController> controller,
                               const std::string& name,
                               int model_instance);

    /// Advance dynamics of the multibody system.
    /// - load controllers (if any are attached) are synchronized and their dynamics advanced in time;
    /// - output is generated if requested in the simulation YAML file;
    /// - the dynamics of the underlying Chrono system is advanced in time;
    /// - soft real-time is enforced if requested in the simulation YAML file.
    void DoStepDynamics();

    /// Apply loads generated by the load controllers in the given list.
    /// Notes:
    /// - controller loads are automatically set in ChParserMbsYAML::DoStepDynamics if controllers have been added with
    ///   ChParserMbsYAML::AttachLoadController.
    /// - this function provides an alternative mechanism for applying controller loads (for arbitrary controllers, not
    ///   necessarily derived from ChLoadController).
    void ApplyLoadControllerLoads(const LoadControllerLoads& controller_loads);

    /// Apply actuations generated by the motor controllers in the given list.
    /// Notes:
    /// - controller actuations are automatically set in ChParserMbsYAML::DoStepDynamics if controllers have been added
    ///   with ChParserMbsYAML::AttachMotorController.
    /// - this function provides an alternative mechanism for applying controller actuations (for arbitrary controllers,
    ///   not necessarily derived from ChMotorController).
    void ApplyMotorControllerActuations(const MotorControllerActuations& controller_loads);

    /// Save simulation output results at the current time.
    /// Note: this function is automatically called in ChParserMbsYAML::DoStepDynamics.
    void SaveOutput(ChSystem& sys, int frame);

  private:
    /// Solver parameters.
    struct SolverParams {
        SolverParams();
        void PrintInfo();

        ChSolver::Type type;
        bool lock_sparsity_pattern;
        bool use_sparsity_pattern_learner;
        double tolerance;
        bool enable_diagonal_preconditioner;
        int max_iterations;
        double overrelaxation_factor;
        double sharpness_factor;
    };

    /// Integrator parameters.
    struct IntegratorParams {
        IntegratorParams();
        void PrintInfo();

        ChTimestepper::Type type;
        double rtol;
        double atol_states;
        double atol_multipliers;
        int max_iterations;
        bool use_stepsize_control;
        bool use_modified_newton;
    };

    /// Run-time visualization parameters.
    struct VisParams {
        VisParams();
        void PrintInfo();

        VisualizationType type;
        bool render;
        double render_fps;
        CameraVerticalDir camera_vertical;
        ChVector3d camera_location;
        ChVector3d camera_target;
        bool enable_shadows;
    };

    /// Simulation and run-time visualization parameters.
    struct SimParams {
        SimParams();
        void PrintInfo();

        ChVector3d gravity;

        ChContactMethod contact_method;

        int num_threads_chrono;
        int num_threads_collision;
        int num_threads_eigen;
        int num_threads_pardiso;

        double time_step;
        double end_time;
        bool enforce_realtime;

        SolverParams solver;
        IntegratorParams integrator;
        VisParams visualization;
    };

    /// Internal specification of a body.
    struct BodyParams {
        BodyParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChBodyAuxRef>> body;  ///< underlying Chrono bodies (one per instance)
        ChVector3d pos;                                   ///< body position (relative to instance frame)
        ChQuaterniond rot;                                ///< body orientation (relative to instance frame)
        ChVector3d lin_vel;                               ///< initial linear velocity
        ChVector3d ang_vel;                               ///< initial angular velocity (in body frame)
        bool is_fixed;                                    ///< indicate if body fixed relative to global frame
        double mass;                                      ///< body mass
        ChFramed com;                                     ///< centroidal frame (relative to body frame)
        ChVector3d inertia_moments;                       ///< moments of inertia (relative to centroidal frame)
        ChVector3d inertia_products;                      ///< products of inertia (relative to centroidal frame)
        std::shared_ptr<utils::ChBodyGeometry> geometry;  ///< visualization and collision geometry
    };

    /// Internal specification of a joint.
    struct JointParams {
        JointParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChJoint>> joint;  ///< underlying Chrono joints (one per instance)
        ChJoint::Type type;                           ///< joint type
        std::string body1;                            ///< identifier of 1st body
        std::string body2;                            ///< identifier of 2nd body
        ChFramed frame;                               ///< joint frame (relative to instance frame)
        std::shared_ptr<ChJoint::BushingData> bdata;  ///< bushing data
        bool is_kinematic;                            ///< indicate if kinematic joint or bushing
    };

    /// Internal specification of a distance constraint.
    struct DistanceConstraintParams {
        DistanceConstraintParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkDistance>> dist;  ///< underlying Chrono constraints (one per instance)
        std::string body1;                                  ///< identifier of 1st body
        std::string body2;                                  ///< identifier of 2nd body
        ChVector3d point1;                                  ///< point on body1 (relative to instance frame)
        ChVector3d point2;                                  ///< point on body2 (relative to instance frame)
    };

    /// Internal specification of a TsdaParams.
    struct TsdaParams {
        TsdaParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkTSDA>> tsda;    ///< underlying Chrono TSDAs (one per instance)
        std::string body1;                                ///< identifier of 1st body
        std::string body2;                                ///< identifier of 2nd body
        ChVector3d point1;                                ///< point on body1 (relative to instance frame)
        ChVector3d point2;                                ///< point on body2 (relative to instance frame)
        double free_length;                               ///< TSDA free (rest) length
        std::shared_ptr<ChLinkTSDA::ForceFunctor> force;  ///< force functor
        std::shared_ptr<utils::ChTSDAGeometry> geometry;  ///< (optional) visualization geometry
    };

    /// Internal specification of an RSDA.
    struct RsdaParams {
        RsdaParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkRSDA>> rsda;      ///< underlying Chrono RSDAs (one per instance)
        std::string body1;                                  ///< identifier of 1st body
        std::string body2;                                  ///< identifier of 2nd body
        ChVector3d pos;                                     ///< RSDA position (relative to instance frame)
        ChVector3d axis;                                    ///< RSDA action axis (relative to instance frame)
        double free_angle;                                  ///< RSDA free (rest) angle
        std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque;  ///< torque functor
    };

    /// Body load type.
    enum class BodyLoadType { FORCE, TORQUE };

    /// Internal specification of a body load (applied force or torque).
    struct BodyLoadParams {
        BodyLoadParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLoadCustom>> load;  ///< underlying Chrono body load (one per instance)
        BodyLoadType type;                                ///< load type: FORCE or TORQUE
        std::string body;                                 ///< name of body to which the load is applied
        bool local_load;                                  ///< is load provided in local frame?
        bool local_point;                                 ///< is point provided in local frame?
        ChVector3d value;                                 ///< load value (force or torque)
        ChVector3d point;                                 ///< force application point
        std::shared_ptr<ChFunction> modulation;           ///< load modulation as function of time
    };

    /// Motor type.
    enum class MotorType { LINEAR, ROTATION };

    /// Motor actuation type.
    enum class MotorActuation {
        POSITION,  ///< position-level (displacement or angle)
        SPEED,     ///< velocity-level (linear or angular speed)
        FORCE,     ///< force-level (force or torque)
        NONE
    };

    /// Internal specification of a motor (linear or rotational).
    struct MotorParams {
        MotorParams();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkMotor>> motor;  ///< underlying Chrono linear motors (one per instance)
        MotorType type;                                   ///< motor type: LINEAR or ROTATION
        ChLinkMotorLinear::GuideConstraint guide;         ///< motor guide type (for a LINEAR motor)
        ChLinkMotorRotation::SpindleConstraint spindle;   ///< motor spindle type (for a ROTATION motor)
        std::string body1;                                ///< identifier of 1st body
        std::string body2;                                ///< identifier of 2nd body
        ChVector3d pos;                                   ///< motor position (relative to instance frame)
        ChVector3d axis;                                  ///< motor action axis (relative to instance frame)
        MotorActuation actuation_type;                    ///< actuation type (motor type)
        std::shared_ptr<ChFunction> actuation_function;   ///< actuation function
        bool has_controller;                              ///< true if using a controller
    };

    /// Wrapper for a load controllers.
    struct LoadController {
        int model_instance;                            ///< model instance containing the loaded body
        std::shared_ptr<ChLoadController> controller;  ///< externally-provided controller
    };

    /// Wrapper for a motor controllers.
    struct MotorController {
        int model_instance;                             ///< model instance containing the actuated motor
        std::shared_ptr<ChLinkMotor> motor;             ///< actuated motor
        std::shared_ptr<ChMotorController> controller;  ///< externally-provided controller
    };

    /// Output database.
    struct OutputData {
        std::vector<std::shared_ptr<ChBody>> bodies;
        std::vector<std::shared_ptr<ChShaft>> shafts;
        std::vector<std::shared_ptr<ChLink>> joints;
        std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
        std::vector<std::shared_ptr<ChShaftsCouple>> couples;
        std::vector<std::shared_ptr<ChLink>> constraints;
        std::vector<std::shared_ptr<ChLinkTSDA>> tsdas;
        std::vector<std::shared_ptr<ChLinkRSDA>> rsdas;
        std::vector<std::shared_ptr<ChLoadCustom>> loads;
        std::vector<std::shared_ptr<ChLinkMotorLinear>> lin_motors;
        std::vector<std::shared_ptr<ChLinkMotorRotation>> rot_motors;
    };

  private:
    static ChSolver::Type ReadSolverType(const YAML::Node& a);
    static ChTimestepper::Type ReadIntegratorType(const YAML::Node& a);
    static VisualizationType ReadVisualizationType(const YAML::Node& a);

    /// Load and return a contact material specification from the specified node.
    ChContactMaterialData ReadMaterialData(const YAML::Node& mat);

    /// Load and return bushing data from the specified node.
    std::shared_ptr<ChJoint::BushingData> ReadBushingData(const YAML::Node& bd);

    /// Load and return a joint type from the specified node.
    ChJoint::Type ReadJointType(const YAML::Node& a);

    /// Load and return joint coordinate system from the specified node.
    ChFramed ReadJointFrame(const YAML::Node& a);

    /// Load and return a geometry structure from the specified node.
    /// Collision geometry and contact material information is set in the return ChBodyGeometry object if the given
    /// object has a member "Contact". Visualization geometry is loaded if the object has a member "Visualization".
    std::shared_ptr<utils::ChBodyGeometry> ReadGeometry(const YAML::Node& d);

    /// Load and return a TSDA geometry structure from the specified node.
    std::shared_ptr<utils::ChTSDAGeometry> ReadTSDAGeometry(const YAML::Node& d);

    /// Load and return a TSDA functor object from the specified node.
    /// The TSDA free length is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctor(const YAML::Node& td, double& free_length);

    /// Load and return an RSDA functor object.
    /// The TSDA free angle is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctor(const YAML::Node& td, double& free_angle);

    /// Load and return the body load type from the specified node.
    BodyLoadType ReadBodyLoadType(const YAML::Node& a);

    /// Load and return the motor type from the specified node.
    MotorType ReadMotorType(const YAML::Node& a);

    /// Load and return a motor actuation type from the specified node.
    MotorActuation ReadMotorActuationType(const YAML::Node& a);

    /// Load and return a linear motor guide constraint type from the specified node.
    ChLinkMotorLinear::GuideConstraint ReadMotorGuideType(const YAML::Node& a);

    /// Load and return a rotation motor spindle constraint type from the specified node.
    ChLinkMotorRotation::SpindleConstraint ReadMotorSpindleType(const YAML::Node& a);

    /// Set Chrono solver parameters.
    void SetSolver(ChSystem& sys, const SolverParams& params, int num_threads_pardiso);

    /// Set Chrono integrator parameters.
    void SetIntegrator(ChSystem& sys, const IntegratorParams& params);

    /// Return motor actuation type as a string.
    static std::string GetMotorActuationTypeString(MotorActuation type);

  private:
    SimParams m_sim;  ///< simulation parameters

    std::unordered_map<std::string, BodyParams> m_body_params;                     ///< bodies
    std::unordered_map<std::string, JointParams> m_joint_params;                   ///< joints
    std::unordered_map<std::string, DistanceConstraintParams> m_distcnstr_params;  ///< distance constraints
    std::unordered_map<std::string, TsdaParams> m_tsda_params;                     ///< TSDA force elements
    std::unordered_map<std::string, RsdaParams> m_rsda_params;                     ///< RSDA force elements
    std::unordered_map<std::string, BodyLoadParams> m_bodyload_params;             ///< body loads
    std::unordered_map<std::string, BodyLoadParams> m_load_controller_params;      ///< external body load controllers
    std::unordered_map<std::string, MotorParams> m_motor_params;                   ///< motors

    std::unordered_map<std::string, LoadController> m_load_controllers;
    std::unordered_map<std::string, MotorController> m_motor_controllers;

    std::shared_ptr<ChSystem> m_sys;
    ChRealtimeStepTimer m_rt_timer;

    OutputData m_output_data;  ///< output data

    bool m_sim_loaded;    ///< YAML simulation file loaded
    bool m_model_loaded;  ///< YAML model file loaded
    int m_crt_instance;   ///< index of last instance created

    friend class ChParserFsiYAML;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
