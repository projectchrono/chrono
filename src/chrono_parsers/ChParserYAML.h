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

#ifndef CH_PARSER_YAML_H
#define CH_PARSER_YAML_H

#include <string>
#include <vector>
#include <unordered_map>

#include "chrono_parsers/ChApiParsers.h"

#include "chrono/assets/ChVisualSystem.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"

#include "chrono/functions/ChFunction.h"

#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Utility class to parse YAML specification files for Chrono models and simulations.
/// The parser caches model information and simulation settings from the corresponding YAML input files and then allows
/// populating a Chrono system and setting solver and simulation parameters.
class ChApiParsers ChParserYAML {
  public:
    ChParserYAML();

    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserYAML(const std::string& yaml_model_filename, const std::string& yaml_sim_filename, bool verbose = false);
    ~ChParserYAML();

    /// Set verbose temrinal output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

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

    /// Return the name of the YAML model.
    const std::string& GetName() const { return m_name; }

    /// Populate the given system with the cached Chrono components.
    /// An instance of the underlying Chrono model can be created at the specified frame (relative to the global frame),
    /// with all Chrono object names using the specified prefix. Throws an error if no YAML model file was loaded.
    int Populate(ChSystem& sys, const ChFramed& model_frame = ChFramed(), const std::string& model_prefix = "");

    /// Remove from the specified system the Chrono objects from the specified instance.
    void Depopulate(ChSystem& sys, int instance_index);

  private:
    /// Simulation and run-time visualization parameters.
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

    struct VisParams {
        VisParams();
        void PrintInfo();

        bool render;
        double render_fps;
        CameraVerticalDir camera_vertical;
        ChVector3d camera_location;
        ChVector3d camera_target;
        bool enable_shadows;
    };

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

  private:
    /// Internal specification of a body.
    struct Body {
        Body();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChBodyAuxRef>> body;  ///< underlying Chrono bodies (one per instance)
        ChVector3d pos;                                   ///< body position (relative to instance frame)
        ChQuaterniond rot;                                ///< body orientation (relative to instance frame)
        bool is_fixed;                                    ///< indicate if body fixed relative to global frame
        double mass;                                      ///< body mass
        ChFramed com;                                     ///< centroidal frame (relative to body frame)
        ChVector3d inertia_moments;                       ///< moments of inertia (relative to centroidal frame)
        ChVector3d inertia_products;                      ///< products of inertia (relative to centroidal frame)
        utils::ChBodyGeometry geometry;                   ///< visualization and collision geometry
    };

    /// Internal specification of a joint.
    struct Joint {
        Joint();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChJoint>> joint;  ///< underlying Chrono joints (one per instance)
        ChJoint::Type type;                           ///< joint type
        std::string body1;                            ///< identifier of 1st body
        std::string body2;                            ///< identifier of 2nd body
        ChFramed frame;                               ///< joint frame (relative to instance frame)
        std::shared_ptr<ChJoint::BushingData> bdata;  ///< bushing data
    };

    /// Internal specification of a distance constraint.
    struct DistanceConstraint {
        DistanceConstraint();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkDistance>> dist;  ///< underlying Chrono constraints (one per instance)
        std::string body1;                                  ///< identifier of 1st body
        std::string body2;                                  ///< identifier of 2nd body
        ChVector3d point1;                                  ///< point on body1 (relative to instance frame)
        ChVector3d point2;                                  ///< point on body2 (relative to instance frame)
    };

    /// Internal specification of a TSDA.
    struct TSDA {
        TSDA();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkTSDA>> tsda;    ///< underlying Chrono TSDAs (one per instance)
        std::string body1;                                ///< identifier of 1st body
        std::string body2;                                ///< identifier of 2nd body
        ChVector3d point1;                                ///< point on body1 (relative to instance frame)
        ChVector3d point2;                                ///< point on body2 (relative to instance frame)
        double free_length;                               ///< TSDA free (rest) length
        std::shared_ptr<ChLinkTSDA::ForceFunctor> force;  ///< force functor
        utils::ChTSDAGeometry geometry;                   ///< (optional) visualization geometry
    };

    /// Internal specification of an RSDA.
    struct RSDA {
        RSDA();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkRSDA>> rsda;      ///< underlying Chrono RSDAs (one per instance)
        std::string body1;                                  ///< identifier of 1st body
        std::string body2;                                  ///< identifier of 2nd body
        ChVector3d pos;                                     ///< RSDA position (relative to instance frame)
        ChVector3d axis;                                    ///< RSDA action axis (relative to instance frame)
        double free_angle;                                  ///< RSDA free (rest) angle
        std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque;  ///< torque functor
    };

    /// Motor actuation type.
    enum class MotorActuation {
        POSITION,  ///< position-level (displacement or angle)
        SPEED,     ///< velocity-level (linear or angular speed)
        FORCE,     ///< force-level (force or torque)
        NONE
    };

    /// Internal specification of a linear motor.
    struct MotorLinear {
        MotorLinear();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkMotorLinear>> motor;  ///< underlying Chrono motors (one per instance)
        ChLinkMotorLinear::GuideConstraint guide;               ///< motor guide type
        std::string body1;                                      ///< identifier of 1st body
        std::string body2;                                      ///< identifier of 2nd body
        ChVector3d pos;                                         ///< motor position (relative to instance frame)
        ChVector3d axis;                                        ///< motor action axis (relative to instance frame)
        MotorActuation actuation_type;                          ///< actuation type (motor type)
        std::shared_ptr<ChFunction> actuation_function;         ///< actuation function
    };

    /// Internal specification of a rotational motor.
    struct MotorRotation {
        MotorRotation();
        void PrintInfo(const std::string& name);

        std::vector<std::shared_ptr<ChLinkMotorRotation>> motor;  ///< underlying Chrono motors (one per instance)
        ChLinkMotorRotation::SpindleConstraint spindle;           ///< motor spindle type
        std::string body1;                                        ///< identifier of 1st body
        std::string body2;                                        ///< identifier of 2nd body
        ChVector3d pos;                                           ///< motor position (relative to instance frame)
        ChVector3d axis;                                          ///< motor action axis (relative to instance frame)
        MotorActuation actuation_type;                            ///< actuation type (motor type)
        std::shared_ptr<ChFunction> actuation_function;           ///< actuation function
    };

  private:
    /// Load and return a ChVector3d from the specified node.
    ChVector3d ReadVector(const YAML::Node& a);

    ///  Load and return a ChQuaternion from the specified node.
    ChQuaterniond ReadQuaternion(const YAML::Node& a);

    /// Load a Cardan angle sequence from the specified node and return as a quaternion.
    /// The sequence is assumed to be extrinsic rotations X-Y-Z.
    ChQuaterniond ReadCardanAngles(const YAML::Node& a);

    /// Return a quaternion loaded from the specified node.
    /// Data is assumed to provide a quaternion or a Cardan extrinsic X-Y-Z angle set.
    ChQuaterniond ReadRotation(const YAML::Node& a);

    /// Load and return a coordinate system from the specified node.
    ChCoordsysd ReadCoordinateSystem(const YAML::Node& a);

    ///  Load and return a ChColor from the specified node.
    ChColor ReadColor(const YAML::Node& a);

    ChSolver::Type ReadSolverType(const YAML::Node& a);
    ChTimestepper::Type ReadIntegratorType(const YAML::Node& a);

    /// Load and return a contact material specification from the specified node.
    ChContactMaterialData ReadMaterialData(const YAML::Node& mat);

    /// Load and return bushing data from the specified node.
    std::shared_ptr<ChJoint::BushingData> ReadBushingData(const YAML::Node& bd);

    /// Load and return a joint type from the specified node.
    ChJoint::Type ReadJointType(const YAML::Node& a);

    /// Load and return joint coordinate syustem from the specified node.
    ChFramed ReadJointFrame(const YAML::Node& a);

    /// Load and return a geometry structure from the specified node.
    /// Collision geometry and contact material information is set in the return ChBodyGeometry object if the given
    /// object has a member "Contact". Visualization geometry is loaded if the object has a member "Visualization".
    utils::ChBodyGeometry ReadGeometry(const YAML::Node& d);

    /// Load and return a TSDA geometry structure from the specified node.
    utils::ChTSDAGeometry ReadTSDAGeometry(const YAML::Node& d);

    /// Load and return a TSDA functor object from the specified node.
    /// The TSDA free length is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkTSDA::ForceFunctor> ReadTSDAFunctor(const YAML::Node& td, double& free_length);

    /// Load and return an RSDA functor object.
    /// The TSDA free angle is also set if the particular functor type defines it.
    std::shared_ptr<ChLinkRSDA::TorqueFunctor> ReadRSDAFunctor(const YAML::Node& td, double& free_angle);

    /// Load and return a motor actuation type from the specified node.
    MotorActuation ReadMotorActuationType(const YAML::Node& a);

    /// Load and return a linear motor guide constraint type from the specified node.
    ChLinkMotorLinear::GuideConstraint ReadMotorGuideType(const YAML::Node& a);

    /// Load and return a rotation motor spindle constraint type from the specified node.
    ChLinkMotorRotation::SpindleConstraint ReadMotorSpindleType(const YAML::Node& a);

    /// Load and return a motor actuation ChFunction object from the specified node.
    std::shared_ptr<ChFunction> ReadFunction(const YAML::Node& a);

    /// Utility function to find the ChBodyAuxRef with specified name.
    std::shared_ptr<ChBodyAuxRef> FindBody(const std::string& name) const;

    /// Set Chrono solver parameters.
    void SetSolver(ChSystem& sys, const SolverParams& params, int num_threads_pardiso);

    /// Set Chrono integrator parameters.
    void SetIntegrator(ChSystem& sys, const IntegratorParams& params);

    /// Return motor actuation type as a string.
    static std::string GetMotorActuationTypeString(MotorActuation type);

  private:
    SimParams m_sim;  ///< simulation parameters

    std::unordered_map<std::string, Body> m_bodies;               ///< bodies
    std::unordered_map<std::string, Joint> m_joints;              ///< joints
    std::unordered_map<std::string, TSDA> m_tsdas;                ///< TSDA force elements
    std::unordered_map<std::string, RSDA> m_rsdas;                ///< RSDA force elements
    std::unordered_map<std::string, DistanceConstraint> m_dists;  ///< distance constraints
    std::unordered_map<std::string, MotorLinear> m_linmotors;     ///< linear motors
    std::unordered_map<std::string, MotorRotation> m_rotmotors;   ///< rotational motors

    bool m_verbose;  ///< verbose terminal output (default: false)

    bool m_sim_loaded;     ///< YAML simulation file loaded
    bool m_model_loaded;   ///< YAML model file loaded
    std::string m_name;    ///< name of the YAML model
    bool m_use_degrees;    ///< all angles given in degrees (default: true)
    int m_instance_index;  ///< index of the last model instance created
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
