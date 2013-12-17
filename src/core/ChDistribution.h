//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHDISTRIBUTION_H
#define CHDISTRIBUTION_H


#include "core/ChMathematics.h"
#include "core/ChVector.h"
#include "core/ChMatrix.h"



/// Base class for all random distributions

class ChDistribution 
{
public:
		/// Compute a random value whose probability is defined by the distribution.
		/// MUST be implemented by children classes.
	virtual double GetRandom() = 0;
};



/// Class that can be used to generate sample numbers according to a 
/// probability distribution. Probability distribution is defined with x,y points,
/// at least a dozen of pairs to be more precise in the reconstruction of probability.

class ChContinuumDistribution : public ChDistribution
{
public:

		/// Create an object that can be used to generate sample numbers according to a 
		/// probability distribution. The probability distribution is a curve represented
		/// by simplified x,y pairs of points. The integral of the probability curve
		/// must be unit, i.e normalized (but if not, a normalization will be enforced)
		/// Note: too few points means approximate results, but too many points might give a 
		/// small performance overhead when calling GetRandom().
	ChContinuumDistribution (ChMatrix<>& mx, ChMatrix<>& my)
	{
	
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
		for (int i = 0; i< x->GetRows()-1; i++)
		{
			integral += 0.5*( (*y)(i) + (*y)(i+1) ) * ( (*x)(i+1) - (*x)(i) );
			(*cdf_y)(i) = integral;
			(*cdf_x)(i) = 0.5* ( (*x)(i+1) + (*x)(i) );
		}
			// normalize if P(x) had not unit integral
		double totintegral = (*cdf_y)(x->GetRows()-2);
		if (totintegral != 1.0)
		{
			for (int i = 0; i< x->GetRows()-1; i++)
			{
				(*cdf_y)(i) *= 1./totintegral;
			}
		}
		(*cdf_x)(x->GetRows()-1) = (*x)(x->GetRows()-1);
		(*cdf_y)(x->GetRows()-1) = 1.0;
		
	}

	~ChContinuumDistribution()
	{
		delete x;
		delete y;
		delete cdf_x;
		delete cdf_y;
	}

		/// Compute a random value whose probability is the probability curve that has 
		/// been entered with x,y points during the creation of this object.
	virtual double GetRandom()
	{
		double mx1 =  (*x)(0);
		double mx2 =  (*cdf_x)(0);
		double my1 =  0;
		double my2 =  (*cdf_y)(0);

		double rand = ChRandom();
		for (int i = 0; i< x->GetRows()-1; i++)
		{
			if (( rand <= (*cdf_y)(i+1) ) &&
				( rand >  (*cdf_y)(i)   ) )
			{
				mx1 = (*cdf_x)(i);
				mx2 = (*cdf_x)(i+1);
				my1 = (*cdf_y)(i);
				my2 = (*cdf_y)(i+1);
				break;
			}
		}
		// linear interp
		double val = mx1 +  ((rand - my1)/(my2-my1)) * (mx2-mx1);
		return val;
	}

	ChMatrix<>* GetProbabilityXpoints() {return x;}
	ChMatrix<>* GetProbabilityYpoints() {return y;}
	ChMatrix<>* GetProbabilityCDFcumulativeX() {return cdf_x;}
	ChMatrix<>* GetProbabilityCDFcumulativeY() {return cdf_y;}

private:
	ChMatrix<>* x;
	ChMatrix<>* y;

	ChMatrix<>* cdf_x;
	ChMatrix<>* cdf_y;
};





/// Class that can be used to generate sample numbers according to a discrete
/// probability distribution. Probability distribution is defined with discrete
/// values, each with a percentual of probability.

class ChDiscreteDistribution : public ChDistribution
{
public:

		/// Create an object that can be used to generate sample numbers according to a discrete
		/// probability distribution. Probability distribution is defined with N discrete
		/// values, each with a percentual of probability. The sum of the N probability values
		/// must be unit, i.e normalized (but if not, a normalization will be enforced)
		/// For example, to get '12.3' for 30% of the times you call GetRandom(), and '150' for
		/// the remaining 70% of the times, create  ChDiscreteDistribution with 
		/// mx = [12.3; 150] and my = [0.3; 0.7]
	ChDiscreteDistribution (ChMatrix<>& mx, ChMatrix<>& my)
	{
	
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
		for (int i = 0; i< x->GetRows(); i++)
		{
			integral += (*y)(i);
			(*cdf_y)(i) = integral;
		}
			// normalize if P(x) had not unit integral
		double totintegral = (*cdf_y)(x->GetRows()-1);
		if (totintegral != 1.0)
		{
			for (int i = 0; i< x->GetRows(); i++)
			{
				(*cdf_y)(i) *= 1./totintegral;
			}
		}
		(*cdf_y)(x->GetRows()-1) = 1.0;
		
	}

	~ChDiscreteDistribution()
	{
		delete x;
		delete y;
		delete cdf_y;
	}

		/// Compute a random value, according to the discrete probabilty values entered
		/// when you created this object
	virtual double GetRandom()
	{
		double rand = ChRandom();
		double lastval = 0;
		for (int i = 0; i< x->GetRows(); i++)
		{
			if (( rand <= (*cdf_y)(i) ) &&
				( rand >  lastval   ) )
			{
				return (*x)(i);
			}
			lastval = (*cdf_y)(i);
		}
		return 0;
	}

	ChMatrix<>* GetProbabilityXpoints() {return x;}
	ChMatrix<>* GetProbabilityYpoints() {return y;}
	ChMatrix<>* GetProbabilityCDFcumulativeY() {return cdf_y;}

private:
	ChMatrix<>* x;
	ChMatrix<>* y;
	ChMatrix<>* cdf_y;
};




#endif  // END of ChMath.h 
