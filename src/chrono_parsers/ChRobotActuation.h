// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility class which provides actuations for a set of motors, based on data
// files. 
//
// =============================================================================

#ifndef CH_ROBOT_ACTUATION_H
#define CH_ROBOT_ACTUATION_H

#include <string>
#include <vector>
#include <fstream>

#include "chrono_parsers/ChApiParsers.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Generic robot actuation driver using interpolated data from files.
/// ChRobotActuation takes 3 input files, the lines of which contain a time stamp and values for the motor actuations
/// (equal in number to the number of specified motors). The "cycle" input data file is required, while the "start"
/// and "stop" input files are optional. Within each input data file, time stamps are assumed to start at 0.
/// Actuations at the current time are calculated (through linear interpolation within each data set) each time the
/// Update() function is called.
///
/// In the most general case, the robot is assumed to go through the following phases:
/// <pre>
/// 1. POSE:
///    During this phase (of user-specified duration), the actuations are evolved to the initial pose, defined by the
///    first line of the "start" dataset (if it exists, otherwise, the "cycle" dataset).
///    This is done by interpolation using a logistic function.
/// 2. HOLD:
///    During this phase (of user-specified duration), the robot holds the initial pose.
/// 3. START:
///    During this phase, the robot assumes the pose necessary for a particular operation mode.
/// 4. CYCLE:
///    This is the main operating phase. This phase can be repeated if so indicated.
/// 5. STOP:
///    During this phase, the robot assumes a rest end position.
/// </pre>
///
/// The different input data files are assumed to be consistent with each other (for example, the end of the "start"
/// dataset must match the beginning of a cycle.
class ChApiParsers ChRobotActuation {
  public:
    /// Actuation phases.
    enum Phase {
        POSE,   ///< from design configuration to an initial pose
        HOLD,   ///< hold the last configuration
        START,  ///< from initial pose to operating pose
        CYCLE,  ///< operation (periodic)
        STOP    ///< from operating pose to final pose
    };

    typedef std::vector<double> Actuation;

    /// Constrauct a robot actuator using the specified input data files.
    /// The data files for the "start" and "stop" phases are not required (pass an ampty string).
    ChRobotActuation(int num_motors,                     ///< number of actuated motors
                     const std::string& filename_start,  ///< name of file with joint actuations for start phase
                     const std::string& filename_cycle,  ///< name of file with joint actuations for cycle phase
                     const std::string& filename_stop,   ///< name of file with joint actuations for stop phase
                     bool repeat = false                 ///< true if cycle phase is looped
    );

    ~ChRobotActuation();

    /// Enable/disable verbose output (default: false).
    void SetVerbose(bool verbose) { m_verbose = verbose; }

    /// Specify time intervals to assume and then hold the initial pose (default: 1s, 1s).
    void SetTimeOffsets(double time_pose,  ///< time to transition to initial pose (duration of POSE phase)
                        double time_hold   ///< time to hold initial pose (duration of HOLD phase)
    );

    /// Return the current motor actuations.
    const Actuation& GetActuation() const { return m_actuations; }

    /// Return the current phase.
    const std::string& GetCurrentPhase() const { return m_phase_names[m_phase]; }

    /// Class to be used as callback interface for user-defined actions at phase changes.
    class ChApiParsers PhaseChangeCallback {
      public:
        virtual ~PhaseChangeCallback() {}
        virtual void OnPhaseChange(ChRobotActuation::Phase old_phase, ChRobotActuation::Phase new_phase) = 0;
    };

    /// Register a phase-change callback object.
    void RegisterPhaseChangeCallback(PhaseChangeCallback* callback) { m_callback = callback; }

    /// Update internal state of the robot driver.
    void Update(double time);

  private:
    void LoadDataLine(double& time, Actuation& buffer);

    int m_num_motors;  ///< number of actuated motors
    bool m_verbose;    ///< verbose output

    std::ifstream m_ifs_start;  ///< input file stream for start phase
    std::ifstream m_ifs_cycle;  ///< input file stream for cycle phase
    std::ifstream m_ifs_stop;   ///< input file stream for stop phase
    std::ifstream* m_ifs;       ///< active input file stream

    double m_time_pose;  ///< time interval to assume initial pose
    double m_time_hold;  ///< time interval to hold initial pose
    double m_offset;     ///< current time offset in input files
    bool m_repeat;       ///< repeat cycle
    Phase m_phase;       ///< current phase

    double m_time_1;           ///< time for cached actuations
    double m_time_2;           ///< time for cached actuations
    Actuation m_actuations_1;  ///< cached actuations (before)
    Actuation m_actuations_2;  ///< cached actuations (after)
    Actuation m_actuations;    ///< current actuations

    PhaseChangeCallback* m_callback;  ///< user callback for phase change

    static const std::string m_phase_names[5];  ///< names of various driver phases

    friend class RoboSimian;
};

/// @} parsers_module


}  // namespace parsers
}  // namespace chrono

#endif