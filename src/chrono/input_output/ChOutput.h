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
// Base class for a Chrono output database.
//
// =============================================================================

#ifndef CH_OUTPUT_H
#define CH_OUTPUT_H

#include "chrono/physics/ChAssembly.h"

namespace chrono {

/// @addtogroup chrono_io
/// @{

/// Base class for a Chrono output database.
class ChApi ChOutput {
  public:
    /// Output database type. 
    /// Currently supported options are ASCII and HDF5.
    enum class Type {
        ASCII,  ///< ASCII text
        HDF5,   ///< HDF-5
        NONE    ///< no output
    };

    /// Output mode options.
    /// - FRAMES: output is organized in groups for each separate frame;
    ///           suitable for postprocessing (e.g., rendering).
    /// - SERIES: output is organized by model components, each of them containing time-series for their various output quantities;
    ///           suitable for plotting results.
    enum class Mode {
      FRAMES,  ///< organize output on a frame-by-frame basis
      SERIES   ///< organize output on component-by-component basis
    };

    ChOutput() {}
    virtual ~ChOutput() {}

    virtual void Initialize(Mode mode) {
        m_mode = mode;
    }

    void Save(double time, int frame, ChAssembly::Components& components) {
        switch (m_mode) {
            case Mode::FRAMES:
                WriteTime(frame, time);
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

    static std::string GetOutputTypeAsString(Type type) {
        switch (type) {
            case Type::NONE:
                return "NONE";
            case Type::ASCII:
                return "ASCII";
            case Type::HDF5:
                return "HDF5";
        }
        return "NONE";
    }

    static std::string GetOutputModeAsString(Mode mode) {
        switch (mode) {
            case Mode::FRAMES:
                return "FRAMES";
            case Mode::SERIES:
                return "SERIES";
        }
        return "FRAMES";
    }

  protected:
    // Functions for Mode::FRAMES

    virtual void WriteTime(int frame, double time) = 0;
    virtual void WriteSection(const std::string& name) = 0;
    virtual void WriteBodies(const std::vector<std::shared_ptr<ChBody>>& bodies) = 0;
    virtual void WriteMarkers(const std::vector<std::shared_ptr<ChMarker>>& markers) = 0;
    virtual void WriteShafts(const std::vector<std::shared_ptr<ChShaft>>& shafts) = 0;
    virtual void WriteJoints(const std::vector<std::shared_ptr<ChLink>>& joints) = 0;
    virtual void WriteCouples(const std::vector<std::shared_ptr<ChShaftsCouple>>& couples) = 0;
    virtual void WriteLinSprings(const std::vector<std::shared_ptr<ChLinkTSDA>>& springs) = 0;
    virtual void WriteRotSprings(const std::vector<std::shared_ptr<ChLinkRSDA>>& springs) = 0;
    virtual void WriteBodyBodyLoads(const std::vector<std::shared_ptr<ChLoadBodyBody>>& loads) = 0;
    virtual void WriteLinMotors(const std::vector<std::shared_ptr<ChLinkMotorLinear>>& motors) = 0;
    virtual void WriteRotMotors(const std::vector<std::shared_ptr<ChLinkMotorRotation>>& motors) = 0;

    Mode m_mode; ///< output mode
};

/// @} chrono_io

}  // end namespace chrono

#endif
