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
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
//
// Definition of the RIG NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO:
////    mesh connectivity doesn't need to be communicated every time (modify Chrono?)  

#ifndef TESTRIG_RIGNODE_H
#define TESTRIG_RIGNODE_H

#include <string>
#include <fstream>
#include <iostream>
#include <vector>

#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

// =============================================================================

class ChFunction_SlipAngle : public chrono::ChFunction {
  public:
    ChFunction_SlipAngle(double max_angle) : m_max_angle(max_angle) {}

    virtual ChFunction_SlipAngle* Clone() const override { return new ChFunction_SlipAngle(m_max_angle); }

    virtual double Get_y(double t) const override;

  private:
    double m_max_angle;
};

// =============================================================================

class RigNode {
  public:
    RigNode(int num_threads);
    ~RigNode();

    void SetOutputFile(const std::string& name);

    void Initialize();
    void Synchronize(int step_number, double time);
    void Advance(double step_size);

    double GetSimTime() { return m_timer.GetTimeSeconds(); }
    double GetTotalSimTime() { return m_cumm_sim_time; }
    void OutputData(int frame);
    void WriteCheckpoint();

  private:
    chrono::ChSystemDEM* m_system;  ///< containing system
    double m_step_size;     ///< integration step size

    std::shared_ptr<chrono::ChBody> m_ground;  ///< ground body
    std::shared_ptr<chrono::ChBody> m_rim;     ///< wheel rim body
    std::shared_ptr<chrono::ChBody> m_set_toe;     ///< set toe body
    std::shared_ptr<chrono::ChBody> m_chassis;     ///< chassis body
    std::shared_ptr<chrono::ChLinkLockPlanePlane> m_plane_plane;  ///< ground-rim joint
    std::shared_ptr<chrono::vehicle::ChDeformableTire> m_tire;                       ///< deformable tire
    std::shared_ptr<chrono::fea::ChLoadContactSurfaceMesh> m_contact_load;  ///< tire contact surface
    std::shared_ptr<chrono::ChLinkLockRevolute> m_revolute; ///< set_toe-rim revolute joint
    std::shared_ptr<ChFunction_SlipAngle> f_slip; ///< function to set toe angle
    std::shared_ptr<chrono::ChLinkEngine> m_slip_motor;   ///< angular motor constraint

    double m_init_vel;  ///< initial wheel forward linear velocity

    std::ofstream m_outf;  ///< output file stream
    chrono::ChTimer<double> m_timer;
    double m_cumm_sim_time;

    static const std::string m_checkpoint_filename;  ///< name of checkpointing file

    // Initialize body and tire state at initial configuration
    void InitBodies(double init_height);
    // Initialize body and tire state from checkpointing file
    void InitBodies(const std::string& filename);

    void WriteStateInformation(chrono::utils::CSV_writer& csv);

    void PrintLowestNode();
    void PrintLowestVertex(const std::vector<chrono::ChVector<>>& vert_pos, const std::vector<chrono::ChVector<>>& vert_vel);
    void PrintContactData(const std::vector<chrono::ChVector<>>& forces, const std::vector<int>& indices);
};

#endif