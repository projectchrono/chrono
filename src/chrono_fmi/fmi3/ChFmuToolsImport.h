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
// Chrono wrappers to fmu_tools FMU import classes for FMI standard 3.0.
//
// =============================================================================

#ifndef CH_FMU3_TOOLS_IMPORT_H
#define CH_FMU3_TOOLS_IMPORT_H

#include <string>

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChFrameMoving.h"

// fmu_tools
#include "fmi3/FmuToolsImport.h"

namespace chrono {
namespace fmi3 {

/// @addtogroup chrono_fmi3
/// @{

using FmuVariable = fmu_tools::fmi3::FmuVariable;

/// Extension of FmuUnit class for Chrono FMUs.
class FmuChronoUnit : public fmu_tools::fmi3::FmuUnit {
  public:
    FmuChronoUnit() : FmuUnit() {}

    /// Load the given ChVector3d from the FMU variable with the specified name.
    fmi3Status GetVecVariable(const std::string& name, ChVector3d& v) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            auto status = GetVariable(name + "." + comp[i], v.data()[i]);
            if (status != fmi3Status::fmi3OK)
                return status;
        }
        return fmi3Status::fmi3OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChVector3d.
    fmi3Status SetVecVariable(const std::string& name, const ChVector3d& v) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            auto status = SetVariable(name + "." + comp[i], v.data()[i]);
            if (status != fmi3Status::fmi3OK)
                return status;
        }
        return fmi3Status::fmi3OK;
    }

    /// Load the given ChQuaternion from the FMU variable with the specified name.
    fmi3Status GetQuatVariable(const std::string& name, ChQuaternion<>& q) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            auto status = GetVariable(name + "." + comp[i], q.data()[i]);
            if (status != fmi3Status::fmi3OK)
                return status;
        }
        return fmi3Status::fmi3OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChQuaternion.
    fmi3Status SetQuatVariable(const std::string& name, const ChQuaternion<>& q) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            auto status = SetVariable(name + "." + comp[i], q.data()[i]);
            if (status != fmi3Status::fmi3OK)
                return status;
        }
        return fmi3Status::fmi3OK;
    }

    /// Load the given ChCoordsys from the FMU variable with the specified name.
    fmi3Status GetCoordsysVariable(const std::string& name, ChCoordsysd& csys) {
        auto status_pos = GetVecVariable(name + ".pos", csys.pos);
        if (status_pos != fmi3Status::fmi3OK)
            return status_pos;
        auto status_rot = GetQuatVariable(name + ".rot", csys.rot);
        if (status_rot != fmi3Status::fmi3OK)
            return status_rot;
        return fmi3Status::fmi3OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChQuaternion.
    fmi3Status SetCoordsysVariable(const std::string& name, const ChCoordsysd& csys) {
        auto status_pos = SetVecVariable(name + ".pos", csys.pos);
        if (status_pos != fmi3Status::fmi3OK)
            return status_pos;
        auto status_rot = SetQuatVariable(name + ".rot", csys.rot);
        if (status_rot != fmi3Status::fmi3OK)
            return status_rot;
        return fmi3Status::fmi3OK;
    }

    /// Load the given ChFrame from the FMU variable with the specified name.
    fmi3Status GetFrameVariable(const std::string& name, ChFrame<>& frame) {
        ChCoordsysd csys;
        auto status = GetCoordsysVariable(name, csys);
        frame = ChFrame<>(csys);
        return status;
    }

    /// Set the FMU variable with specified name to the values of the given ChFrame.
    /// Note that only the base data members of the FMU's ChFrame object are set (namely, the position vector and
    /// orientation quaternion). If the rotation matrix of the ChFrame object is needed, it must be explicitly
    /// calculated before use through a call to ChFrame::Set_A_quaternion (e.g., in a pre-step callback).
    fmi3Status SetFrameVariable(const std::string& name, const ChFrame<>& frame) {
        auto status = SetCoordsysVariable(name, frame.GetCoordsys());
        return status;
    }

    /// Load the given ChFrameMoving from FMU variables with the specified name.
    fmi3Status GetFrameMovingVariable(const std::string& name, ChFrameMoving<>& frame) {
        ChCoordsysd csys;
        auto status_csys = GetCoordsysVariable(name, csys);
        if (status_csys != fmi3Status::fmi3OK)
            return status_csys;
        ChVector3d pos_dt;
        auto status_pos_dt = GetVecVariable(name + ".pos_dt", pos_dt);
        if (status_pos_dt != fmi3Status::fmi3OK)
            return status_pos_dt;
        ChQuaternion<> rot_dt;
        auto status_rot_dt = GetQuatVariable(name + ".rot_dt", rot_dt);
        if (status_rot_dt != fmi3Status::fmi3OK)
            return status_rot_dt;

        frame = ChFrameMoving<>(csys);
        frame.SetPosDt(pos_dt);
        frame.SetRotDt(rot_dt);

        return fmi3Status::fmi3OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChgFrameMoving.
    /// Note that only the base data members of the FMU's ChFrameMoving object are set (namely, the position vector and
    /// orientation quaternion and their time derivatives). If the rotation matrix of the ChFrameMoving object is
    /// needed, it must be explicitly calculated before use through a call to ChFrame::Set_A_quaternion (e.g., in a
    /// pre-step callback).
    fmi3Status SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& frame) {
        auto status_csys = SetCoordsysVariable(name, frame.GetCoordsys());
        if (status_csys != fmi3Status::fmi3OK)
            return status_csys;
        auto status_pos_dt = SetVecVariable(name + ".pos_dt", frame.GetPosDt());
        if (status_pos_dt != fmi3Status::fmi3OK)
            return status_pos_dt;
        auto status_rot_dt = SetQuatVariable(name + ".rot_dt", frame.GetRotDt());
        if (status_rot_dt != fmi3Status::fmi3OK)
            return status_rot_dt;

        return fmi3Status::fmi3OK;
    }
};

/// @} chrono_fmi3

}  // end namespace fmi3
}  // end namespace chrono

#endif
