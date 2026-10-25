// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_PARSER_TIRE_STATIC_RIG_YAML_H
#define CH_PARSER_TIRE_STATIC_RIG_YAML_H

#include <string>

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono_parsers/yaml/ChChannelPlotsYAML.h"

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireStaticTestRig.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification files for Chrono::Vehicle static tire test rig simulations (ChTireStaticTestRig).
/// A TIRE_STATIC_RIG simulation specification file references a rig model file (wheel and tire, specified through
/// Chrono::Vehicle JSON files, and the MBS solver), which follows the same schema as the rig model of a single-wheel
/// test rig simulation (WHEEL_RIG), and specifies the rig plate properties and the test scenario.
class ChApiParsers ChParserTireStaticRigYAML : public ChParserMbsYAML {
  public:
    ChParserTireStaticRigYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserTireStaticRigYAML();

    /// Load the specified TIRE_STATIC_RIG simulation specification file (and the rig model file it references).
    virtual void LoadFile(const std::string& yaml_filename) override;

    /// Create the static tire test rig in the given system from the cached specification, then initialize it.
    /// The system should be created with CreateSystem().
    std::shared_ptr<vehicle::ChTireStaticTestRig> CreateRig(ChSystem& sys);

    /// Return the test rig (null if not yet created).
    std::shared_ptr<vehicle::ChTireStaticTestRig> GetRig() const { return m_rig; }

    /// Return the test mode.
    vehicle::ChTireStaticTestRig::Mode GetMode() const { return m_mode; }

    /// Return true if generating output, i.e., writing an output database or recording data for plots.
    virtual bool OutputEnabled() const override { return ChParserYAML::OutputEnabled() || PlotsEnabled(); }

    /// Return true if plots were requested (in the `plots` section of the simulation specification).
    bool PlotsEnabled() const { return m_plots.Enabled(); }

    /// Write the rig output channels (see ChTireStaticTestRig::GetOutputChannels) at the current time and record them
    /// for plotting (if plots were requested).
    /// The output directory must be set (SetOutputDir) and the rig created (CreateRig) before generating output.
    virtual void WriteOutput(int frame, double time) override;

    /// Generate the requested plots from the data recorded during the simulation.
    /// This function must be called once, at the end of the simulation; it does nothing if no plots were requested.
    /// Plotting requires the Chrono::Postprocess module and Gnuplot.
    void GeneratePlots();

    /// Return the camera location relative to the wheel position.
    const ChVector3d& GetCameraOffset() const { return m_camera_offset; }

    /// Return the camera look-at point relative to the wheel position.
    const ChVector3d& GetCameraTargetOffset() const { return m_camera_target_offset; }

  private:
    void LoadRigFile(const std::string& filename);
    void LoadTestData(const YAML::Node& yaml);

    static vehicle::ChTireStaticTestRig::Mode ReadMode(const YAML::Node& a);

    // Rig model
    std::string m_wheel_json;
    std::string m_tire_json;
    bool m_set_collision_type;
    vehicle::ChTire::CollisionType m_collision_type;
    bool m_set_contact_surface;
    vehicle::ChTire::ContactSurfaceType m_contact_surface_type;
    double m_contact_surface_dim;
    int m_collision_family;

    // Rig plate
    double m_plate_friction;
    double m_plate_restitution;
    double m_plate_Young_modulus;

    // Test scenario
    vehicle::ChTireStaticTestRig::Mode m_mode;
    double m_radial_load;
    double m_drop_speed;
    double m_radial_speed;
    double m_longitudinal_speed;
    double m_lateral_speed;
    double m_torsional_speed;
    double m_transition_delay;

    // Visualization
    ChVector3d m_camera_offset;
    ChVector3d m_camera_target_offset;

    // Plots
    ChChannelPlotsYAML m_plots;

    std::shared_ptr<vehicle::ChTireStaticTestRig> m_rig;
};

/// @} parsers_module

}  // namespace parsers
}  // namespace chrono

#endif
