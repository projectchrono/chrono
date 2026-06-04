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
//
// Implementation of the base class for a Chrono output database.
//
// =============================================================================

#include "chrono/input_output/ChOutput.h"

namespace chrono {

void ChOutput::Write(double time, int frame, const ChAssembly::Components& components) {
    WriteTime(frame, time);
    Write(components);
}

void ChOutput::Write(const ChAssembly::Components& components) {
    switch (m_mode) {
        case Mode::FRAMES:
            WriteBodies(components.bodies);
            WriteShafts(components.shafts);
            WriteJoints(components.joints);
            WriteCouples(components.couples);
            WriteBodyBodyLoads(components.bushings);
            ////WriteConstraints(components.constraints);
            WriteLinSprings(components.tsdas);
            WriteRotSprings(components.rsdas);
            WriteLinMotors(components.lin_motors);
            WriteRotMotors(components.rot_motors);

            break;
        case Mode::SERIES:

            break;
    }
}

std::string ChOutput::GetFormatAsString(Format type) {
    switch (type) {
        case Format::NONE:
            return "NONE";
        case Format::ASCII:
            return "ASCII";
        case Format::HDF5:
            return "HDF5";
    }
    return "";
}

std::string ChOutput::GetModeAsString(Mode mode) {
    switch (mode) {
        case Mode::FRAMES:
            return "FRAMES";
        case Mode::SERIES:
            return "SERIES";
    }
    return "";
}

}  // end namespace chrono
