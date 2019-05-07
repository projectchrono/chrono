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

#ifndef CHDISTRIBUTION_H
#define CHDISTRIBUTION_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

/// Base class for all random distributions.
class ChApi ChDistribution {
  public:
    /// Default destructor for distribution object
    virtual ~ChDistribution() {}

    /// Compute a random value whose probability is defined by the distribution.
    virtual double GetRandom() = 0;
};

/// Class for a distribution with a single 'value' that has probability 1.0
/// (that is, the distribution has a spike corresponding to 'value' and zero elsewhere).
class ChApi ChConstantDistribution : public ChDistribution {
  public:
    ChConstantDistribution(double mvalue) : value(mvalue) {}

    /// Compute a random value whose probability is defined by the distribution.
    /// In this very simple case, returns always the single value.
    virtual double GetRandom() override { return value; }

  private:
    double value;
};

/// Class for a distribution with uniform probability between a lower 'min' value and upper 'max' value
/// (that is, the distribution looks like a rectangle)
class ChApi ChMinMaxDistribution : public ChDistribution {
  public:
    ChMinMaxDistribution(double mmin, double mmax);

    /// Compute a random value whose probability is defined by the distribution,
    /// that is a value between min and max.
    virtual double GetRandom() override;

  private:
    double min;
    double max;
};

/// Class that generates the Gauss normal distribution (the 'bell' distribution)
/// using the Box–Muller transform.
class ChApi ChNormalDistribution : public ChDistribution {
  public:
    /// Create the normal distribution with assigned variance.
    /// The mean is 0 by default, but you can offset the curve if
    /// you want by providing a mean.
    ChNormalDistribution(double mvariance, double mmean = 0);

    /// Compute a random value whose probability density is the normal distribution.
    /// It uses the Box–Muller transform.
    virtual double GetRandom() override;

    double GetMean() const { return mean; }
    double GetVariance() const { return variance; }

  private:
    double mean;
    double variance;

    bool hasSpare;
    double rand1;
    double rand2;
};

/// Class that generates the Weibull distribution.
/// It can be used for example to describe particle size distribution,
/// as in the subcase of Rosin & Rammler distribution.
class ChApi ChWeibullDistribution : public ChDistribution {
  public:
    /// Create the Weibull distribution with assigned scale factor 'lambda' and with shape 'k'.
    /// The larger 'lambda' is, the more horizontally stretched is the distribution.
    /// For k<1, there is a vertical peak at 0.
    /// For k=1, you get the exponential distribution.
    /// For k>1, you get an asymmetric bell shape.
    ChWeibullDistribution(double mlambda, double mk);

    /// Compute a random value whose probability density is the Weibull distribution.
    /// It uses the "Smirnov transform" (inverse probability integral transform)
    virtual double GetRandom() override;

    double GetK() const { return k; }
    double GetLambda() const { return lambda; }

  private:
    double k;
    double lambda;
};

/// Class that generates the Zhang distribution, a modified exponential distribution.
/// It is used to describe particle size distribution, with minimum part.size.
class ChApi ChZhangDistribution : public ChDistribution {
  public:
    /// Create the Zhang distribution with average part. size
    /// and minimum particle size. Usually average:minimum = 3.25:1
    ChZhangDistribution(double average_size, double minimum_size);

    /// Compute a random value whose probability density is the Weibull distribution.
    /// It uses the "Smirnov transform" (inverse probability integral transform)
    virtual double GetRandom() override;

    double GetMinSize() const { return minsize; }
    double GetAverageSize() const { return (minsize + (1.0 / lambdar)); }

  private:
    double minsize;
    double lambdar;
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
    ChContinuumDistribution(ChMatrix<>& mx, ChMatrix<>& my);

    ~ChContinuumDistribution();

    /// Compute a random value whose probability is the probability curve that has
    /// been entered with x,y points during the creation of this object.
    virtual double GetRandom() override;

    const ChMatrix<>& GetProbabilityXpoints() const { return *x; }
    const ChMatrix<>& GetProbabilityYpoints() const { return *y; }
    const ChMatrix<>& GetProbabilityCDFcumulativeX() const { return *cdf_x; }
    const ChMatrix<>& GetProbabilityCDFcumulativeY() const { return *cdf_y; }

  private:
    ChMatrix<>* x;
    ChMatrix<>* y;

    ChMatrix<>* cdf_x;
    ChMatrix<>* cdf_y;
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
    /// the remaining 70% of the times, create  ChDiscreteDistribution with
    /// mx = [12.3; 150] and my = [0.3; 0.7]
    ChDiscreteDistribution(ChMatrix<>& mx, ChMatrix<>& my);

    ~ChDiscreteDistribution();

    /// Compute a random value, according to the discrete probability values entered
    /// when you created this object
    virtual double GetRandom() override;

    const ChMatrix<>& GetProbabilityXpoints() const { return *x; }
    const ChMatrix<>& GetProbabilityYpoints() const { return *y; }
    const ChMatrix<>& GetProbabilityCDFcumulativeY() const { return *cdf_y; }

  private:
    ChMatrix<>* x;
    ChMatrix<>* y;
    ChMatrix<>* cdf_y;
};

}  // end namespace chrono

#endif
