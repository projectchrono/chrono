// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This file defines a fluid node, it is based on a 3DOF NodeXYZ
// element. This class is similar to ChMatterSPH but is meant to be a bit more
// general.
// =============================================================================

#pragma once

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/real.h"

namespace chrono {

// Forward references (for parent hierarchy pointer)
class ChSystemParallelDVI;
class ChSystemParallelMPM;
class ChParallelDataManager;

class CH_PARALLEL_API Ch3DOFContainer : public ChPhysicsItem {
  public:
    Ch3DOFContainer();
    ~Ch3DOFContainer();

    Ch3DOFContainer(const Ch3DOFContainer& other);             // Copy constructor
    Ch3DOFContainer& operator=(const Ch3DOFContainer& other);  // Assignment operator

    // void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    // Update occurs before he solve
    virtual void Update(double ChTime) = 0;
    virtual void UpdatePosition(double ChTime) = 0;
    virtual int GetNumConstraints() = 0;
    // Integrate happens after the solve
    // void Integrate(double ChTime);
    // Position of the node - in absolute csys.
    real3 GetPos(int i);
    // Position of the node - in absolute csys.
    void SetPos(const int& i, const real3& mpos);

    // Velocity of the node - in absolute csys.
    real3 GetPos_dt(int i);
    // Velocity of the node - in absolute csys.
    void SetPos_dt(const int& i, const real3& mposdt);

  protected:
    ChParallelDataManager* data_manager;
};

class CH_PARALLEL_API ChFluidContainer : public Ch3DOFContainer {
  public:
    ChFluidContainer(ChSystemParallelDVI* system);
    ~ChFluidContainer();
    void AddFluid(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints();
};
class CH_PARALLEL_API ChMPMContainer : public Ch3DOFContainer {
  public:
    ChMPMContainer(ChSystemParallelMPM* system);
    ~ChMPMContainer();
    void AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities);
    void Update(double ChTime);
    void UpdatePosition(double ChTime);
    int GetNumConstraints() { return 0; }
};
// class CH_PARALLEL_API ChFEMContainer : public Ch3DOFContainer {
//  public:
//    ChFEMContainer(ChSystemParallelDVI* system);
//    ~ChFEMContainer();
//};
}
