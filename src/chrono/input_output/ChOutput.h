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
    enum class Format {
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

    virtual ~ChOutput() {}

    void Write(int frame, double time, const ChAssembly::Components& components);
    void Write(int frame, double time, const std::vector<const ChAssembly::Components*>& components);

    static std::string GetFormatAsString(Format type);

    static std::string GetModeAsString(Mode mode);

  protected:
    ChOutput(Mode mode);

    Mode m_mode;  ///< output mode

  protected:
    // Functions for Mode::FRAMES

    virtual void WriteTimeStamp(int frame, double time) = 0;
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

  protected:
    // Data and functions for Mode::SERIES

    struct BodyBuffers {
        std::vector<ChVector3d> pos;      ///< REF position
        std::vector<ChVector3d> rot;      ///< REF orientation (Tait-Bryan extrinsic X-Y-Z angles)
        std::vector<ChVector3d> lin_vel;  ///< REF linear velocity
        std::vector<ChVector3d> ang_vel;  ///< REF angular velocity
    };

    struct ShaftBuffers {
        std::vector<double> pos;  ///< position or angle
        std::vector<double> vel;  ///< linear or angular velocity
    };

    struct JointBuffers {
        std::vector<ChWrenchd> react1;  ///< reaction wrench on body 1
        std::vector<ChWrenchd> react2;  ///< reaction wrench on body 2
    };

    struct TSDABuffers {
        std::vector<ChVector3d> point1;  ///< position of point 1
        std::vector<ChVector3d> point2;  ///< position of point 2
        std::vector<double> len;         ///< length
        std::vector<double> vel;         ///< linear velocity
        std::vector<double> force;       ///< force
    };

    struct RSDABuffers {
        std::vector<double> ang;     ///< angle
        std::vector<double> vel;     ///< angular velocity
        std::vector<double> torque;  ///< torque
    };

    bool m_buf_allocated;                   ///< buffers allocated?
    std::vector<double> m_time;             ///< time series
    std::vector<BodyBuffers> m_body_buf;    ///< body buffers
    std::vector<ShaftBuffers> m_shaft_buf;  ///< shaft buffers
    std::vector<JointBuffers> m_joint_buf;  ///< joint buffers
    std::vector<TSDABuffers> m_tsda_buf;    ///< TSDA buffers
    std::vector<RSDABuffers> m_rsda_buf;    ///< RSDA buffers
};

/// @} chrono_io

}  // end namespace chrono

#endif
