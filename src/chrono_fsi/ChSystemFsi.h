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
// Author: Arman Pazouki
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_SYSTEMFSI_H_
#define CH_SYSTEMFSI_H_

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChBce.cuh"
#include "chrono_fsi/ChFluidDynamics.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiInterface.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Physical system for fluid-solid interaction problem.
///
/// This class is used to represent a fluid-solid interaction problem consist of
/// fluid dynamics and multibody system. Each of the two underlying physics are
/// independent objects owned and instantiated by this class. Additionally,
/// the fsi system owns other objects to handle the interface between the two
/// systems, boundary condition enforcing markers, and data.
class CH_FSI_API ChSystemFsi : public ChFsiGeneral {
  public:
    /// Constructor for FSI system.
    /// This class constructor instantiates all the member objects. Wherever relevant, the
    /// instantiation is handled by sending a pointer to other objects or data.
    /// Therefore, the sub-classes have pointers to the same data.
    ChSystemFsi(ChSystem* other_physicalSystem, bool other_haveFluid);

    /// Destructor for the FSI system.
    ~ChSystemFsi();

    /// Function to integrate the fsi system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody
    /// system dynamics. The midpoint data of MBS is needed for fluid dynamics update.
    virtual void DoStepDynamics_FSI();

    /// Function to integrate the multibody system dynamics based on Runge-Kutta
    /// 2nd-order integration scheme.
    virtual void DoStepDynamics_ChronoRK2();

    /// Function to initialize the midpoint device data of the fluid system by
    /// copying from the full step
    virtual void CopyDeviceDataToHalfStep();

    /// Function to fill out the dependent data based on the independent one. For instance,
    /// it copies the position of the rigid bodies from the multibody dynamics system
    /// to arrays in fsi system since they are needed for internal use.
    virtual void FinalizeData();

    /// Function to return the pointer to the data manager.
    ChFsiDataManager* GetDataManager() { return fsiData; }

    /// Function to return the pointer to the data.
    SimParams* GetSimParams() { return paramsH; }

    /// Function to return the pointer to the fsi bodies. The pointer to all fsi bodies; i.e.,
    /// the bodies with two representation in fluid and multibody systems, are stored in a vector.
    std::vector<std::shared_ptr<ChBody>>* GetFsiBodiesPtr() { return &fsiBodeisPtr; }

    /// Initialize the graphics interface of the chrono system.
    void InitializeChronoGraphics(chrono::ChVector<> CameraLocation = chrono::ChVector<>(1, 0, 0),
                                  chrono::ChVector<> CameraLookAt = chrono::ChVector<>(0, 0, 0));

    /// Finzalize data by calling FinalizeData function and finalize fluid and bce
    /// objects if the system have fluid.
    virtual void Finalize();

  private:
    /// Integrate the chrono system based on an explicit Euler scheme.
    int DoStepChronoSystem(Real dT, double mTime);

    ChFsiDataManager* fsiData;                          ///< pointer to data manager which holds all the data
    std::vector<std::shared_ptr<ChBody>> fsiBodeisPtr;  ///< vector of a pointers to FSI bodies
    ChFluidDynamics* fluidDynamics;                     ///< pointer to the fluid system
    ChFsiInterface* fsiInterface;                       ///< pointer to the fsi interface system
    ChBce* bceWorker;                                   ///< pointer to the bce workers

    chrono::ChSystem* mphysicalSystem;  ///< pointer to the multibody system

    SimParams* paramsH;            ///< pointer to the simulation parameters
    NumberOfObjects* numObjectsH;  ///< pointer to the number of objects, fluid, bce, and boundary markers

    double mTime;    ///< current real time of the simulation
    bool haveFluid;  ///< boolean to check if the system have fluid or not

#ifdef CHRONO_OPENGL
    chrono::opengl::ChOpenGLWindow* gl_window;  ///< opengl graphics window if opengl exist
#endif
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
