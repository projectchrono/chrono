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
// Various pre-defined TSDA and RSDA force/torque functors.
//
// =============================================================================

#ifndef CH_FORCE_FUNCTORS_H
#define CH_FORCE_FUNCTORS_H

#include <cstdint>
#include <string>
#include <vector>

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkTSDA.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

// -----------------------------------------------------------------------------
// Utility functor classes for TSDA force elements
// -----------------------------------------------------------------------------

/// Base class for linear and nonlinear translational spring forces.
class ChApi SpringForce : public ChLinkTSDA::ForceFunctor {
  public:
    SpringForce(double preload);
    void enable_stops(double min_length, double max_length);
    void set_stops(const std::vector<std::pair<double, double>>& data_bump,
                   const std::vector<std::pair<double, double>>& data_rebound);
    void set_stops(double bump_coefficient, double rebound_coefficient);
    double evaluate_stops(double length);
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  protected:
    double m_P;  ///< pre-tension

    bool m_stops;
    double m_min_length;
    double m_max_length;
    ChFunctionInterp m_bump;
    ChFunctionInterp m_rebound;
};

/// Utility class for specifying a linear translational spring force with pre-tension.
/// F = P - K * (length - rest_length)
class ChApi LinearSpringForce : public SpringForce {
  public:
    LinearSpringForce(double k, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
};

/// Utility class for specifying a nonlinear translational spring force with pre-tension.
/// F = P - mapK(length - rest_length)
class ChApi NonlinearSpringForce : public SpringForce {
  public:
    NonlinearSpringForce(double preload = 0);
    NonlinearSpringForce(const std::vector<std::pair<double, double>>& dataK, double preload = 0);
    void add_pointK(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapK;
};

/// Utility class for specifying a linear translational damper force.
/// F = -C * vel
class ChApi LinearDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    LinearDamperForce(double c, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c;
};

/// Utility class for specifying a nonlinear translational damper force.
/// F = -mapC(vel)
class ChApi NonlinearDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    NonlinearDamperForce();
    NonlinearDamperForce(const std::vector<std::pair<double, double>>& dataC);
    void add_pointC(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapC;
};

/// Utility class for specifying a degressive translational damper force.
class ChApi DegressiveDamperForce : public ChLinkTSDA::ForceFunctor {
  public:
    /// Fallback to LinearDamperForce
    DegressiveDamperForce(double c_compression);

    /// Fallback to LinearDamperForce with different compression and expansion bins
    DegressiveDamperForce(double c_compression, double c_expansion);

    /// Different compression and expansion degressivity, same damper coefficient at origin
    DegressiveDamperForce(double c_compression, double degr_compression, double degr_expansion);

    /// Full parametrization
    DegressiveDamperForce(double c_compression, double degr_compression, double c_expansion, double degr_expansion);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c_compression;
    double m_c_expansion;
    double m_degr_compression;
    double m_degr_expansion;
};

/// Utility class for specifying a linear translational spring-damper force with pre-tension.
/// F = P - K * (length - rest_length) - C * vel
class ChApi LinearSpringDamperForce : public SpringForce {
  public:
    LinearSpringDamperForce(double k, double c, double preload = 0);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_c;
};

/// Utility class for specifying a nonlinear translational spring-damper force with pre-tension.
/// F = P - mapK(length - rest_length) - mapC(vel)
class ChApi NonlinearSpringDamperForce : public SpringForce {
  public:
    NonlinearSpringDamperForce(double preload = 0);
    NonlinearSpringDamperForce(const std::vector<std::pair<double, double>>& dataK,
                               const std::vector<std::pair<double, double>>& dataC,
                               double preload = 0);
    void add_pointK(double x, double y);
    void add_pointC(double x, double y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapK;
    ChFunctionInterp m_mapC;
};

/// Utility class for specifying a general nonlinear translational spring-damper force with pre-tension.
/// F = P - map(length - rest_length, vel)
class ChApi MapSpringDamperForce : public SpringForce {
  public:
    MapSpringDamperForce(double preload = 0);
    MapSpringDamperForce(const std::vector<double>& defs,
                         const std::vector<double>& vels,
                         ChMatrixConstRef data,
                         double preload = 0);
    void set_deformations(const std::vector<double> defs);
    void add_pointC(double x, const std::vector<double>& y);
    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

    void print_data();

  private:
    std::vector<double> m_defs;
    std::vector<double> m_vels;
    ChMatrixDynamic<double> m_data;

    std::pair<int, int> m_last;
};

// -----------------------------------------------------------------------------
// Utility functor classes for RSDA torque elements
// -----------------------------------------------------------------------------

/// Utility class for specifying a linear rotational spring torque.
class ChApi LinearSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringTorque(double k, double preload = 0);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_P;
};

/// Utility class for specifying a nonlinear rotational spring torque.
class ChApi NonlinearSpringTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearSpringTorque(double preload = 0);
    NonlinearSpringTorque(const std::vector<std::pair<double, double>>& dataK, double preload = 0);
    void add_pointK(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapK;
    double m_P;
};

/// Utility class for specifying a linear rotational damper torque.
class ChApi LinearDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearDamperTorque(double c);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_c;
};

/// Utility class for specifying a nonlinear rotational damper torque.
class ChApi NonlinearDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearDamperTorque();
    NonlinearDamperTorque(const std::vector<std::pair<double, double>>& dataC);
    void add_pointC(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapC;
};

/// Utility class for specifying a linear rotational spring-damper torque.
class ChApi LinearSpringDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    LinearSpringDamperTorque(double k, double c, double preload = 0);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    double m_k;
    double m_c;
    double m_P;
};

/// Utility class for specifying a nonlinear rotational spring-damper torque.
class ChApi NonlinearSpringDamperTorque : public ChLinkRSDA::TorqueFunctor {
  public:
    NonlinearSpringDamperTorque(double preload = 0);
    NonlinearSpringDamperTorque(const std::vector<std::pair<double, double>>& dataK,
                                const std::vector<std::pair<double, double>>& dataC,
                                double preload = 0);
    void add_pointK(double x, double y);
    void add_pointC(double x, double y);
    virtual double evaluate(double time, double rest_angle, double angle, double vel, const ChLinkRSDA& link) override;
#ifndef SWIG
    virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) override;
#endif

  private:
    ChFunctionInterp m_mapK;
    ChFunctionInterp m_mapC;
    double m_P;
};

/// @} utils

}  // end namespace utils
}  // end namespace chrono

#endif
