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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHDISTRIBUTION_H
#define CHDISTRIBUTION_H

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

namespace chrono {

/// Base class for all random distributions

class ChDistribution {
  public:
	/// Default destructor for distribution object
    virtual ~ChDistribution(){};
    /// Compute a random value whose probability is defined by the distribution.
    /// MUST be implemented by children classes.
    virtual double GetRandom() = 0;

};

/// Class for a distribution with a single 'value' that
/// has probability 1.0 (that is, the distribution has
/// a spike corresponding to 'value' and zero elsewhere).

class ChConstantDistribution : public ChDistribution {
  public:
    ChConstantDistribution(double mvalue) { value = mvalue; }

    /// Compute a random value whose probability is defined by the distribution.
    /// In this very simple case, however, returns always the single value.
    virtual double GetRandom() { return value; }

  private:
    double value;
};

/// Class for a distribution with uniform probability between
/// a lower 'min' value and upper 'max' value (that is, the
/// distribution looks like a rectangle)

class ChMinMaxDistribution : public ChDistribution {
  public:
    ChMinMaxDistribution(double mmin, double mmax) {
        min = mmin;
        max = mmax;
    }

    /// Compute a random value whose probability is defined by the distribution,
    /// that is a value between min and max.
    virtual double GetRandom() { return min + (::chrono::ChRandom() * (max - min)); }

  private:
    double min;
    double max;
};

/// Class that generates the Gauss normal distribution (the 'bell' distribution)
/// using the Box–Muller transform.

class ChNormalDistribution : public ChDistribution {
  public:
    /// Create the normal distribution with assigned variance.
    /// The mean is 0 by default, but you can offset the curve if
    /// you want by providing a mean.
    ChNormalDistribution(double mvariance, double mmean = 0) {
        variance = mvariance;
        mean = mmean;
        hasSpare = false;
    }

    /// Compute a random value whose probability density is the normal distribution.
    /// It uses the Box–Muller transform.
    virtual double GetRandom() {
        if (hasSpare) {
            hasSpare = false;
            return (mean + sqrt(variance * rand1) * sin(rand2));
        }

        hasSpare = true;

        rand1 = ::chrono::ChRandom();
        if (rand1 < 1e-100)
            rand1 = 1e-100;
        rand1 = -2.0 * log(rand1);
        rand2 = ::chrono::ChRandom() * CH_C_2PI;

        return (mean + sqrt(variance * rand1) * cos(rand2));
    }

    double GetMean() { return mean; }
    double GetVariance() { return variance; }

  private:
    double mean;
    double variance;

    bool hasSpare;
    double rand1;
    double rand2;
};

/// Class that generates the Weibull distribution
/// It can be used for example to describe particle size
/// distribution, as in the subcase of Rosin & Rammler distribution.

class ChWeibullDistribution : public ChDistribution {
  public:
    /// Create the Weibull distribution with assigned scale
    /// factor 'lambda' and with shape 'k'.
    /// The larger 'lambda' is, the more horizontally stretched is the distribution.
    /// For k<1, there is a vertical peak at 0.
    /// For k=1, you get the exponential distribution
    /// For k>1, you get an asymmetric bell shape
    ChWeibullDistribution(double mlambda, double mk) {
        lambda = mlambda;
        k = mk;
    }

    /// Compute a random value whose probability density is the Weibull distribution.
    /// It uses the "Smirnov transform" (inverse probability integral transform)
    virtual double GetRandom() {
        double rand = ::chrono::ChRandom();
        if (rand < 1e-100)
            rand = 1e-100;
        return lambda * pow(-log(rand), (1.0 / k));
    }

    double GetK() { return k; }
    double GetLambda() { return lambda; }

  private:
    double k;
    double lambda;
};

/// Class that generates the Zhang distribution, a modified exponential distribution.
/// It is used to describe particle size distribution, with minimum part.size.

class ChZhangDistribution : public ChDistribution {
  public:
    /// Create the Zhang distribution with average part. size
    /// and minimum particle size. Usually average:minimum = 3.25:1
    ChZhangDistribution(double average_size, double minimum_size) {
        minsize = minimum_size;
        lambdar = 1.0 / (average_size - minimum_size);
    }

    /// Compute a random value whose probability density is the Weibull distribution.
    /// It uses the "Smirnov transform" (inverse probability integral transform)
    virtual double GetRandom() {
        double rand = ::chrono::ChRandom();
        if (rand < 1e-100)
            rand = 1e-100;
        return (minsize + (1.0 / lambdar) * (-log(rand)));
    }

    double GetMinSize() { return minsize; }
    double GetAverageSize() { return (minsize + (1.0 / lambdar)); }

  private:
    double minsize;
    double lambdar;
};

/// Class that can be used to generate sample numbers according to a
/// probability distribution. Probability distribution is defined with x,y points,
/// at least a dozen of pairs to be more precise in the reconstruction of probability.
/// It uses the "Smirnov transform" (inverse probability integral transform)

class ChContinuumDistribution : public ChDistribution {
  public:
    /// Create an object that can be used to generate sample numbers according to a generic
    /// probability distribution. The probability distribution is a curve represented
    /// by simplified x,y pairs of points. The integral of the probability curve
    /// must be unit, i.e normalized (but if not, a normalization will be enforced)
    /// Note: too few points means approximate results, but too many points might give a
    /// small performance overhead when calling GetRandom().
    ChContinuumDistribution(ChMatrix<>& mx, ChMatrix<>& my) {
        if (mx.GetRows() != my.GetRows())
            throw ChException("Probability curve must have same number of rows in abscysse and ordinates");
        if ((mx.GetColumns() != 1) || (my.GetColumns() != 1))
            throw ChException("Probability curve must be column-vectors as input");

        x = new ChMatrixDynamic<>;
        y = new ChMatrixDynamic<>;
        cdf_x = new ChMatrixDynamic<>;
        cdf_y = new ChMatrixDynamic<>;

        *x = mx;
        *y = my;

        *cdf_x = mx;
        *cdf_y = my;

        // compute CDF
        double integral = 0;
        for (int i = 0; i < x->GetRows() - 1; i++) {
            integral += 0.5 * ((*y)(i) + (*y)(i + 1)) * ((*x)(i + 1) - (*x)(i));
            (*cdf_y)(i) = integral;
            (*cdf_x)(i) = 0.5 * ((*x)(i + 1) + (*x)(i));
        }
        // normalize if P(x) had not unit integral
        double totintegral = (*cdf_y)(x->GetRows() - 2);
        if (totintegral != 1.0) {
            for (int i = 0; i < x->GetRows() - 1; i++) {
                (*cdf_y)(i) *= 1. / totintegral;
            }
        }
        (*cdf_x)(x->GetRows() - 1) = (*x)(x->GetRows() - 1);
        (*cdf_y)(x->GetRows() - 1) = 1.0;
    }

    ~ChContinuumDistribution() {
        delete x;
        delete y;
        delete cdf_x;
        delete cdf_y;
    }

    /// Compute a random value whose probability is the probability curve that has
    /// been entered with x,y points during the creation of this object.
    virtual double GetRandom() {
        double mx1 = (*x)(0);
        double mx2 = (*cdf_x)(0);
        double my1 = 0;
        double my2 = (*cdf_y)(0);

        double rand = ChRandom();
        for (int i = 0; i < x->GetRows() - 1; i++) {
            if ((rand <= (*cdf_y)(i + 1)) && (rand > (*cdf_y)(i))) {
                mx1 = (*cdf_x)(i);
                mx2 = (*cdf_x)(i + 1);
                my1 = (*cdf_y)(i);
                my2 = (*cdf_y)(i + 1);
                break;
            }
        }
        // linear interp
        double val = mx1 + ((rand - my1) / (my2 - my1)) * (mx2 - mx1);
        return val;
    }

    ChMatrix<>* GetProbabilityXpoints() { return x; }
    ChMatrix<>* GetProbabilityYpoints() { return y; }
    ChMatrix<>* GetProbabilityCDFcumulativeX() { return cdf_x; }
    ChMatrix<>* GetProbabilityCDFcumulativeY() { return cdf_y; }

  private:
    ChMatrix<>* x;
    ChMatrix<>* y;

    ChMatrix<>* cdf_x;
    ChMatrix<>* cdf_y;
};

/// Class that can be used to generate sample numbers according to a discrete
/// probability distribution. Probability distribution is defined with discrete
/// values, each with a percentual of probability.

class ChDiscreteDistribution : public ChDistribution {
  public:
    /// Create an object that can be used to generate sample numbers according to a discrete
    /// probability distribution. Probability distribution is defined with N discrete
    /// values, each with a percentual of probability. The sum of the N probability values
    /// must be unit, i.e normalized (but if not, a normalization will be enforced)
    /// For example, to get '12.3' for 30% of the times you call GetRandom(), and '150' for
    /// the remaining 70% of the times, create  ChDiscreteDistribution with
    /// mx = [12.3; 150] and my = [0.3; 0.7]
    ChDiscreteDistribution(ChMatrix<>& mx, ChMatrix<>& my) {
        if (mx.GetRows() != my.GetRows())
            throw ChException("Probability values and percentages must have the same size");
        if ((mx.GetColumns() != 1) || (my.GetColumns() != 1))
            throw ChException("Probability values and percentages must be column-vectors as input");

        x = new ChMatrixDynamic<>;
        y = new ChMatrixDynamic<>;
        cdf_y = new ChMatrixDynamic<>;

        *x = mx;
        *y = my;
        *cdf_y = my;

        // compute CDF
        double integral = 0;
        for (int i = 0; i < x->GetRows(); i++) {
            integral += (*y)(i);
            (*cdf_y)(i) = integral;
        }
        // normalize if P(x) had not unit integral
        double totintegral = (*cdf_y)(x->GetRows() - 1);
        if (totintegral != 1.0) {
            for (int i = 0; i < x->GetRows(); i++) {
                (*cdf_y)(i) *= 1. / totintegral;
            }
        }
        (*cdf_y)(x->GetRows() - 1) = 1.0;
    }

    ~ChDiscreteDistribution() {
        delete x;
        delete y;
        delete cdf_y;
    }

    /// Compute a random value, according to the discrete probability values entered
    /// when you created this object
    virtual double GetRandom() {
        double rand = ChRandom();
        double lastval = 0;
        for (int i = 0; i < x->GetRows(); i++) {
            if ((rand <= (*cdf_y)(i)) && (rand > lastval)) {
                return (*x)(i);
            }
            lastval = (*cdf_y)(i);
        }
        return 0;
    }

    ChMatrix<>* GetProbabilityXpoints() { return x; }
    ChMatrix<>* GetProbabilityYpoints() { return y; }
    ChMatrix<>* GetProbabilityCDFcumulativeY() { return cdf_y; }

  private:
    ChMatrix<>* x;
    ChMatrix<>* y;
    ChMatrix<>* cdf_y;
};

}  // end namespace chrono

#endif
