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

#ifndef CH_FMU_TOOLS_IMPORT_H
#define CH_FMU_TOOLS_IMPORT_H

#include <string>

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChFrame.h"

// fmu_tools
#include "FmuToolsImport.hpp"

namespace chrono {

/// Extension of FmuUnit class for Chrono FMUs.
class FmuChronoUnit : public FmuUnit {
  public:
    FmuChronoUnit() : FmuUnit() {}

    /// Load the given ChVector from the FMU variable with the specified name.
    fmi2Status GetVecVariable(const std::string& name, ChVector<>& v) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            fmi2ValueReference vr = scalarVariables.at(name + "." + comp[i]).GetValueReference();
            auto status = GetVariable(vr, v.data()[i], FmuVariable::Type::Real);
            if (status != fmi2OK)
                return status;
        }
        return fmi2OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChVector.
    fmi2Status SetVecVariable(const std::string& name, const ChVector<>& v) {
        std::string comp[3] = {"x", "y", "z"};
        for (int i = 0; i < 3; i++) {
            fmi2ValueReference vr = scalarVariables.at(name + "." + comp[i]).GetValueReference();
            auto status = SetVariable(vr, v.data()[i], FmuVariable::Type::Real);
            if (status != fmi2OK)
                return status;
        }
        return fmi2OK;
    }

    /// Load the given ChQuaternion from the FMU variable with the specified name.
    fmi2Status GetQuatVariable(const std::string& name, ChQuaternion<>& q) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            fmi2ValueReference vr = scalarVariables.at(name + "." + comp[i]).GetValueReference();
            auto status = GetVariable(vr, q.data()[i], FmuVariable::Type::Real);
            if (status != fmi2OK)
                return status;
        }
        return fmi2OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChQuaternion.
    fmi2Status SetQuatVariable(const std::string& name, const ChQuaternion<>& q) {
        std::string comp[4] = {"e0", "e1", "e2", "e3"};
        for (int i = 0; i < 4; i++) {
            fmi2ValueReference vr = scalarVariables.at(name + "." + comp[i]).GetValueReference();
            auto status = SetVariable(vr, q.data()[i], FmuVariable::Type::Real);
            if (status != fmi2OK)
                return status;
        }
        return fmi2OK;
    }

    /// Load the given ChCoordsys from the FMU variable with the specified name.
    fmi2Status GetCsysVariable(const std::string& name, ChCoordsys<>& csys) {
        auto status_pos = GetVecVariable(name + ".pos", csys.pos);
        if (status_pos != fmi2OK)
            return status_pos;
        auto status_rot = GetQuatVariable(name + ".rot", csys.rot);
        if (status_rot != fmi2OK)
            return status_rot;
        return fmi2OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChQuaternion.
    fmi2Status SetCsysVariable(const std::string& name, const ChCoordsys<>& csys) {
        auto status_pos = SetVecVariable(name + ".pos", csys.pos);
        if (status_pos != fmi2OK)
            return status_pos;
        auto status_rot = SetQuatVariable(name + ".rot", csys.rot);
        if (status_rot != fmi2OK)
            return status_rot;
        return fmi2OK;
    }

    /// Load the given ChFrame from the FMU variable with the specified name.
    fmi2Status GetFrameVariable(const std::string& name, ChFrame<>& frame) {
        ChCoordsys<> csys;
        auto status = GetCsysVariable(name, csys);
        frame = ChFrame<>(csys);
        return status;
    }

    /// Set the FMU variable with specified name to the values of the given ChFrame.
    /// Note that only the base data members of the FMU's ChFrame object are set (namely, the position vector and
    /// orientation quaternion). If the rotation matrix of the ChFrame object is needed, it must be explicitly
    /// calculated before use through a call to ChFrame::Set_A_quaternion (e.g., in a pre-step callback).
    fmi2Status SetFrameVariable(const std::string& name, const ChFrame<>& frame) {
        auto status = SetCsysVariable(name, frame.GetCoord());
        return status;
    }

    /// Load the given ChFrameMoving from FMU variables with the specified name.
    fmi2Status GetFrameMovingVariable(const std::string& name, ChFrameMoving<>& frame) {
        ChCoordsys<> csys;
        auto status_csys = GetCsysVariable(name, csys);
        if (status_csys != fmi2OK)
            return status_csys;
        ChVector<> pos_dt;
        auto status_pos_dt = GetVecVariable(name + ".pos_dt", pos_dt);
        if (status_pos_dt != fmi2OK)
            return status_pos_dt;
        ChQuaternion<> rot_dt;
        auto status_rot_dt = GetQuatVariable(name + ".rot_dt", rot_dt);
        if (status_rot_dt != fmi2OK)
            return status_rot_dt;

        frame = ChFrameMoving<>(csys);
        frame.SetPos_dt(pos_dt);
        frame.SetRot_dt(rot_dt);

        return fmi2OK;
    }

    /// Set the FMU variable with specified name to the values of the given ChgFrameMoving.
    /// Note that only the base data members of the FMU's ChFrameMoving object are set (namely, the position vector and
    /// orientation quaternion and their time derivatives). If the rotation matrix of the ChFrameMoving object is
    /// needed, it must be explicitly calculated before use through a call to ChFrame::Set_A_quaternion (e.g., in a
    /// pre-step callback).
    fmi2Status SetFrameMovingVariable(const std::string& name, const ChFrameMoving<>& frame) {
        auto status_csys = SetCsysVariable(name, frame.GetCoord());
        if (status_csys != fmi2OK)
            return status_csys;
        auto status_pos_dt = SetVecVariable(name + ".pos_dt", frame.GetPos_dt());
        if (status_pos_dt != fmi2OK)
            return status_pos_dt;
        auto status_rot_dt = SetQuatVariable(name + ".rot_dt", frame.GetRot_dt());
        if (status_rot_dt != fmi2OK)
            return status_rot_dt;

        return fmi2OK;
    }
};

}  // end namespace chrono

#endif
