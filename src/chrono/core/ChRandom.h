// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_RANDOM_H
#define CH_RANDOM_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

#include <random>

namespace chrono {

class ChApi ChRandom {
  public:
    /// Get a random number in the interval [0, 1)
    static double Get();

    /// Seeds the random number generator to allow a repeatable generation.
    static void SetSeed(double seed);

    ChRandom(const ChRandom&) = delete;
    ChRandom& operator=(const ChRandom&) = delete;

  protected:
    static ChRandom& GetInstance();

    ChRandom();

    virtual ~ChRandom() {}

    std::random_device m_rand_device;                       ///< random number generator
    std::mt19937 m_generator;                               ///< Mersenne-Twister pseudo-random engine
    std::uniform_real_distribution<double> m_distribution;  ///< uniform distribution
};

//-------------------------------------------------

/// Base class for all random distributions.
class ChApi ChDistribution {
  public:
    ChDistribution() {}

    virtual ~ChDistribution() {}

    /// Compute a random value whose probability is defined by the distribution.
    virtual double GetRandom() = 0;
};

/// Class for a distribution with a single 'value' that has probability 1.0.
/// (that is, the distribution has a spike corresponding to 'value' and zero elsewhere).
class ChApi ChConstantDistribution : public ChDistribution {
  public:
    ChConstantDistribution(double value) : m_value(value) {}

    /// Compute a random value whose probability is defined by the distribution.
    /// In this very simple case, returns always the single value.
    virtual double GetRandom() override { return m_value; }

  private:
    double m_value;
};

/// Class for a distribution with uniform probability between a lower 'min' value and upper 'max' value.
class ChApi ChUniformDistribution : public ChDistribution {
  public:
    ChUniformDistribution(double min = 0.0, double max = 1.0);

    /// Compute a random value whose probability is defined by the distribution,
    /// that is a value between min and max.
    virtual double GetRandom() override;

  private:
    std::random_device m_rand_device;                       ///< random number generator
    std::mt19937 m_generator;                               ///< Mersenne-Twister pseudo-random engine
    std::uniform_real_distribution<double> m_distribution;  ///< uniform distribution
};

/// Class that generates the Gauss Normal distribution (the 'bell' distribution).
class ChApi ChNormalDistribution : public ChDistribution {
  public:
    /// Create a Normal distribution with assigned mean and standard deviation.
    ChNormalDistribution(double mean = 0, double std_dev = 1);

    /// Compute a random value whose probability density is the normal distribution.
    virtual double GetRandom() override;

    double GetMean() const { return m_mean; }

    double GetSTD() const { return m_std_dev; }

  private:
    std::random_device m_rand_device;                 ///< random number generator
    std::mt19937 m_generator;                         ///< Mersenne-Twister pseudo-random engine
    std::normal_distribution<double> m_distribution;  ///< normal distribution

    double m_mean = 0;     ///< mean value
    double m_std_dev = 0;  ///< standard deviation
};

/// Class that generates a Weibull distribution.
/// It can be used for example to describe particle size distribution, as in the subcase of Rosin & Rammler
/// distribution.
class ChApi ChWeibullDistribution : public ChDistribution {
  public:
    /// Create a Weibull distribution with assigned shape and scale parameters.
    /// Notes:
    /// - the larger the scale, the more horizontally stretched is the distribution
    /// - for shape param < 1, there is a vertical peak at 0
    /// - for shape param = 1, you get an exponential distribution
    /// - for shape param > 1, you get an asymmetric bell shape
    ChWeibullDistribution(double shape_param, double scale_param);

    /// Compute a random value whose probability density is the Weibull distribution.
    virtual double GetRandom() override;

    double GetShapeParam() const { return m_shape_param; }

    double GetScaleParam() const { return m_scale_param; }

  private:
    std::random_device m_rand_device;                  ///< random number generator
    std::mt19937 m_generator;                          ///< Mersenne-Twister pseudo-random engine
    std::weibull_distribution<double> m_distribution;  ///< weibull distribution

    double m_shape_param = 1;
    double m_scale_param = 1;
};

/// Class that generates the Zhang distribution, a modified exponential distribution.
/// It is used to describe particle size distribution, with minimum particle size.
class ChApi ChZhangDistribution : public ChDistribution {
  public:
    /// Create the Zhang distribution with average and minimum particle size.
    /// Usually is average : minimum = 3.25 : 1
    ChZhangDistribution(double average_size, double minimum_size);

    /// Compute a random value whose probability density is the Weibull distribution.
    /// It uses the "Smirnov transform" (inverse probability integral transform)
    virtual double GetRandom() override;

    double GetMinSize() const { return m_min_size; }

    double GetAverageSize() const { return (m_min_size + (1.0 / m_lambda_r)); }

  private:
    std::random_device m_rand_device;
    std::mt19937 m_generator;
    std::uniform_real_distribution<double> m_distribution;

    double m_min_size;
    double m_lambda_r;
};

/// Class that can be used to generate sample numbers according to a probability distribution.
/// Probability distribution is defined with x,y points, at least a dozen of pairs to be more
/// precise in the reconstruction of probability.
/// It uses the "Smirnov transform" (inverse probability integral transform)
class ChApi ChContinuumDistribution : public ChDistribution {
  public:
    /// Create an object that can be used to generate sample numbers according to a generic
    /// probability distribution. The probability distribution is a curve represented
    /// by simplified x,y pairs of points. The integral of the probability curve
    /// must be unit, i.e normalized (but if not, a normalization will be enforced)
    /// Note: too few points means approximate results, but too many points might give a
    /// small performance overhead when calling GetRandom().
    ChContinuumDistribution(ChVectorDynamic<>& x, ChVectorDynamic<>& y);

    ~ChContinuumDistribution() {}

    /// Compute a random value whose probability is the probability curve that has
    /// been entered with x,y points during the creation of this object.
    virtual double GetRandom() override;

    const ChVectorDynamic<>& GetProbabilityXPoints() const { return m_x; }

    const ChVectorDynamic<>& GetProbabilityYPoints() const { return m_y; }

    const ChVectorDynamic<>& GetProbabilityCDFCumulativeX() const { return m_cdf_x; }

    const ChVectorDynamic<>& GetProbabilityCDFCumulativeY() const { return m_cdf_y; }

  private:
    std::random_device m_rand_device;
    std::mt19937 m_generator;
    std::uniform_real_distribution<double> m_distribution;

    ChVectorDynamic<> m_x;
    ChVectorDynamic<> m_y;
    ChVectorDynamic<> m_cdf_x;
    ChVectorDynamic<> m_cdf_y;
};

/// Class that can be used to generate sample numbers according to a discrete probability distribution.
/// Probability distribution is defined with discrete values, each with a percentual of probability.
class ChApi ChDiscreteDistribution : public ChDistribution {
  public:
    /// Create an object that can be used to generate sample numbers according to a discrete
    /// probability distribution. Probability distribution is defined with N discrete
    /// values, each with a percentual of probability. The sum of the N probability values
    /// must be unit, i.e normalized (but if not, a normalization will be enforced)
    /// For example, to get '12.3' for 30% of the times you call GetRandom(), and '150' for
    /// the remaining 70% of the times, create ChDiscreteDistribution with
    /// x = [12.3; 150] and y = [0.3; 0.7]
    ChDiscreteDistribution(ChVectorDynamic<>& discrete_values, ChVectorDynamic<>& probabilities);

    ~ChDiscreteDistribution() {}

    /// Compute a random value, according to the discrete probability values entered
    /// when you created this object
    virtual double GetRandom() override;

    const ChVectorDynamic<>& GetProbabilityXPoints() const { return m_x; }
    const ChVectorDynamic<>& GetProbabilityYPoints() const { return m_y; }
    const ChVectorDynamic<>& GetProbabilityCDFCumulativeY() const { return m_cdf_y; }

  private:
    std::random_device m_rand_device;
    std::mt19937 m_generator;
    std::uniform_real_distribution<double> m_distribution;

    ChVectorDynamic<> m_x;
    ChVectorDynamic<> m_y;
    ChVectorDynamic<> m_cdf_y;
};

}  // end namespace chrono

#endif  // !CH_RANDOM_H
