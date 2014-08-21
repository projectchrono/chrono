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

#ifndef CHRANDOMSHAPECREATOR_H
#define CHRANDOMSHAPECREATOR_H


#include "core/ChMathematics.h"
#include "core/ChVector.h"
#include "core/ChMatrix.h"
#include "core/ChDistribution.h"
#include "core/ChSmartpointers.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "geometry/ChCCylinder.h"
#include "physics/ChBodyEasy.h"


namespace chrono {
namespace particlefactory {


// Forward reference
class ChRandomShapeCreator;


	/// Inherit from this class and pass an object to the PostCreation() 
	/// functions of particle creator, to have the callback executed 
	/// per each created particle. Ex. use this to add optional assets, custom
	/// materials or settings.
class ChCallbackPostCreation
{
public:
		/// Implement this function if you want to provide the post creation callback.
	virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) = 0;
};



	/// BASE class for generators of random ChBody shapes
class ChRandomShapeCreator : public ChShared
{
public:
	ChRandomShapeCreator() 
		{
			callback_post_creation = 0;
			add_collision_shape = true;
			add_visualization_asset = true;
		}

			/// Function that creates a random ChBody particle each
			/// time it is called. 
			/// Note: it MUST BE IMPLEMENTED by children classes!
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) = 0;

			/// This function does RandomGenerate and also executes the
			/// the custom callback, if provided. Usually no need to override.
	virtual ChSharedPtr<ChBody> RandomGenerateAndCallbacks(ChCoordsys<> mcoords)
		{
			ChSharedPtr<ChBody> mbody = this->RandomGenerate(mcoords);

			if (callback_post_creation)
				callback_post_creation->PostCreation( mbody, mcoords, *this);
			return mbody;
		}
			/// Set the callback function to execute at each 
			/// each particle generation
	void SetCallbackPostCreation(ChCallbackPostCreation* mc) { callback_post_creation = mc;}

			/// Set if the created particles must include the collision	
			/// shape(s). Note that this is ON by default. Switching off will
			/// turn off the collision.
	void SetAddCollisionShape(bool addcoll) { this->add_collision_shape = addcoll;}

			/// Set if the created particles must include the visualization
			/// asset(s). This is ON by default. Switching off might be more 
			/// memory efficient for very large simulations that are batch-processed only.
	void SetAddVisualizationAsset(bool addvisual) { this->add_visualization_asset = addvisual;}

protected:
	ChCallbackPostCreation* callback_post_creation;
	bool add_collision_shape;
	bool add_visualization_asset;
};




	/// Class for generating spheres with variable radius
	/// and density. By default uses constant distributions 
	/// (all spheres with default radius 0.01) but you can provide your distributions.
class ChRandomShapeCreatorSpheres : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorSpheres() 
	{
		// defaults
		diameter = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.02));
		density  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double mrad = 0.5*diameter->GetRandom();
		ChSharedPtr<ChBodyEasySphere> mbody(new ChBodyEasySphere(mrad, 
											density->GetRandom(), 
											this->add_collision_shape, 
											this->add_visualization_asset));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the random diameter.
	void SetDiameterDistribution(ChSmartPtr<ChDistribution> mdistr) {diameter = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> diameter;
	ChSmartPtr<ChDistribution> density;
};




	/// Class for generating boxes with variable sizes
	/// and density. By default uses constant distributions 
	/// (all boxes with default fixed sizes) but you can provide your distributions.
class ChRandomShapeCreatorBoxes : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorBoxes() 
	{
		// defaults
		x_size      = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		sizeratioYZ = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1.0));
		sizeratioZ  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1.0));
		density     = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double sx = fabs(x_size->GetRandom());
		double sy = fabs(sx * sizeratioYZ->GetRandom());
		double sz = fabs(sx * sizeratioYZ->GetRandom()*sizeratioZ->GetRandom());
		ChSharedPtr<ChBodyEasyBox> mbody(new ChBodyEasyBox(
							sx, 
							sy, 
							sz, 
							fabs(density->GetRandom()), 
							this->add_collision_shape, 
							this->add_visualization_asset));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the x size, that is the longest axis.
	void SetXsizeDistribution(ChSmartPtr<ChDistribution> mdistr) {x_size = mdistr;}
			/// Set the statistical distribution for scaling on both Y,Z widths (the lower <1, the thinner, as a needle).
	void SetSizeRatioYZDistribution(ChSmartPtr<ChDistribution> mdistr) {sizeratioYZ = mdistr;}
			/// Set the statistical distribution for scaling on Z width (the lower <1, the flatter, as a chip).
	void SetSizeRatioZDistribution(ChSmartPtr<ChDistribution> mdistr) {sizeratioZ = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> x_size;
	ChSmartPtr<ChDistribution> sizeratioYZ;
	ChSmartPtr<ChDistribution> sizeratioZ;
	ChSmartPtr<ChDistribution> density;
};



	/// Class for generating cylinders with variable diameter
	/// and length. By default uses constant distributions 
	/// (all cylinders are equal) but you can provide your distributions.
class ChRandomShapeCreatorCylinders : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorCylinders() 
	{
		// defaults
		diameter       = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.02));
		length_factor  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(2.0));
		density = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double rad    = 0.5*diameter->GetRandom();
		double height = length_factor->GetRandom() * 2.0*rad;
		ChSharedPtr<ChBodyEasyCylinder> mbody(new ChBodyEasyCylinder(
							rad,
							height, 
							density->GetRandom(), 
							this->add_collision_shape, 
							this->add_visualization_asset));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the statistical distribution for the diameter.
	void SetDiameterDistribution(ChSmartPtr<ChDistribution> mdistr) {diameter = mdistr;}
			/// Set the statistical distribution for the length ratio (length = diameter*length_factor).
	void SetLengthFactorDistribution(ChSmartPtr<ChDistribution> mdistr) {length_factor = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	ChSmartPtr<ChDistribution> diameter;
	ChSmartPtr<ChDistribution> length_factor;
	ChSmartPtr<ChDistribution> density;
};



	/// Class for generating convex hulls with variable chordal size
	/// and aspect ratios. By default uses constant distributions 
	/// (all hulss are equally sized) but you can provide your distributions.
class ChRandomShapeCreatorConvexHulls : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorConvexHulls() 
	{
		// defaults
		npoints = 6;
		chord       = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.01));
		sizeratioYZ = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1.0));
		sizeratioZ  = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1.0));
		density     = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double mchord    = chord->GetRandom();
		double msizeratioYZ = sizeratioYZ->GetRandom();
		double msizeratioZ  = sizeratioZ->GetRandom();
		
		std::vector< ChVector<> > points;
		points.resize(npoints);
		double hsizex=0;
		double hsizey=0;
		double hsizez=0;
		for (int ip = 0; ip < npoints; ++ip)
		{
			points[ip].x= ChRandom();
			points[ip].y= ChRandom();
			points[ip].z= ChRandom();
			if (fabs(points[ip].x) > hsizex) hsizex=fabs(points[ip].x);
			if (fabs(points[ip].y) > hsizey) hsizey=fabs(points[ip].y);
			if (fabs(points[ip].z) > hsizez) hsizez=fabs(points[ip].z);
		}
		for (int ip = 0; ip < npoints; ++ip)
		{
			points[ip].x *= 0.5*mchord/hsizex;
			points[ip].y *= msizeratioYZ*(0.5*mchord/hsizey);
			points[ip].z *= msizeratioYZ*(0.5*mchord/hsizez)*msizeratioZ;
		}

		ChSharedPtr<ChBodyEasyConvexHull> mbody(new ChBodyEasyConvexHull(
							points, 
							density->GetRandom(), 
							this->add_collision_shape, 
							this->add_visualization_asset));
		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Set the number of random vertexes used to generate each random convex hull.
			/// Note that the final number of vertexes in the hull might be lower since
			/// random points that fall inside the convex hull are not used.
	void SetNpoints(int mnpoints) {npoints = mnpoints;}
			/// Set the statistical distribution for the radius.
	void SetChordDistribution(ChSmartPtr<ChDistribution> mdistr) {chord = mdistr;}
			/// Set the statistical distribution for scaling on both Y,Z widths (the lower <1, the thinner, as a needle).
	void SetSizeRatioYZDistribution(ChSmartPtr<ChDistribution> mdistr) {sizeratioYZ = mdistr;}
			/// Set the statistical distribution for scaling on Z width (the lower <1, the flatter, as a chip).
	void SetSizeRatioZDistribution(ChSmartPtr<ChDistribution> mdistr) {sizeratioZ = mdistr;}

			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	int npoints;
	ChSmartPtr<ChDistribution> chord;
	ChSmartPtr<ChDistribution> sizeratioYZ;
	ChSmartPtr<ChDistribution> sizeratioZ;
	ChSmartPtr<ChDistribution> density;
};




	/// Class for generating worm-like particles, optionally helically twisted.
	/// This can be used, for example, to generate shavings or chips as 
	/// those created in milling and grinding processes, etc.
	/// This basic class uses a row of small spheres to approximate the
	/// chip shape, in sake of high computational performance in collisions
	/// etc.
class ChRandomShapeCreatorShavings : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorShavings() 
	{
		// defaults
		spacing_factor = 0.5;
		diameter    = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.02));
		twistU      = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.00));
		twistV      = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(0.00));
		lengthratio = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(3.0));
		density     = ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000));
	}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
	{
		double mtwistU		= twistU->GetRandom();
		double mtwistV		= twistV->GetRandom();
		double mdiameter    = diameter->GetRandom();
		double mlengthratio = ChMax(1.0,lengthratio->GetRandom());
		double mlength		= mdiameter * mlengthratio;
		double mlengthsweep	= mlength-mdiameter;
		double targetinterval = mdiameter*spacing_factor;
		double nintervals   = ceil(mlengthsweep/targetinterval);
		double realinterval  = mlengthsweep / nintervals;
		unsigned int npoints = (unsigned int)nintervals+1;

		std::vector< ChVector<> > points;
		points.resize(npoints);
		std::vector< double > radii;
		radii.resize(npoints);
		
		ChFrame<> localframe;
		for (unsigned int ip = 0; ip < npoints; ++ip)
		{
			radii[ip] = 0.5*mdiameter;
			points[ip]= localframe.GetPos();
			// compute displacement to next sphere:
			ChFrame<> displacement;
			displacement.SetPos( ChVector<>(realinterval, 0,0)); // shift on x
			ChQuaternion<> mrotU;
			mrotU.Q_from_AngY(realinterval*mtwistU);
			ChQuaternion<> mrotV;
			mrotV.Q_from_AngX(realinterval*mtwistV);
			displacement.SetRot(mrotU % mrotV); // rotate on z and y
			localframe.ConcatenatePostTransformation(displacement);
		}
		
		ChSharedPtr<ChBodyEasyClusterOfSpheres> mbody(new ChBodyEasyClusterOfSpheres(
							points, 
							radii, 
							density->GetRandom(), 
							this->add_collision_shape, 
							this->add_visualization_asset));

		//GetLog() << "Diameter:" << mdiameter << " length:" << mlength << " mass:" << mbody->GetMass() << "\n  inertiaXX" << mbody->GetInertiaXX() << "\n inertiaXY:" <<  mbody->GetInertiaXY() << "\n";

		mbody->SetCoord(mcoords);
		return mbody;
	};

			/// Since these worm-like structures are approximated with spheres,
			/// this settings tell how many spheres do you want along the line.
			/// To keep things more intuitive, this 'spacing' value means the 
			/// following: spacing between spheres = diameter * spacing_factor.
			/// The lower, the more precise the approximation, but the more the
			/// computational time for collisions. Usually, good values are in 0.4-0.8 range.
			/// The spacing might be adjusted automatically to match the end of the shaving, btw.
	void SetSpheresSpacingFactor(double md) {spacing_factor = md;}
			/// Set the statistical distribution for the diameter.
	void SetDiameterDistribution(ChSmartPtr<ChDistribution> mdistr) {diameter = mdistr;}
			/// Set the statistical distribution for the length/diameter ratio.
	void SetLengthRatioDistribution(ChSmartPtr<ChDistribution> mdistr) {lengthratio = mdistr;}
			/// Set the statistical distribution for the twist curvature of line (u direction). Curvature=1/radius
	void SetTwistDistributionU(ChSmartPtr<ChDistribution> mdistr) {twistU = mdistr;}
			/// Set the statistical distribution for the twist curvature of line (v direction). Curvature=1/radius
	void SetTwistDistributionV(ChSmartPtr<ChDistribution> mdistr) {twistV = mdistr;}
			/// Set the statistical distribution for the random density.
	void SetDensityDistribution(ChSmartPtr<ChDistribution> mdistr) {density = mdistr;}

private:
	double spacing_factor;
	ChSmartPtr<ChDistribution> diameter;
	ChSmartPtr<ChDistribution> twistU;
	ChSmartPtr<ChDistribution> twistV;
	ChSmartPtr<ChDistribution> lengthratio;
	ChSmartPtr<ChDistribution> density;
};




	/// Class for generating spheres from different families, 
	/// each with given probability. It 'mixes' different 
	/// ChRandomShapeGenerator sources (among these you can 
	/// also put other ChRandomShapeCreatorFromFamilies to create 
	/// tree-like families).
	/// This can be used to make bi-modal or multi-modal distributions, 
	/// for example suppose that you need a mixture of 30% spheres, with
	/// their own size distribution, and 70% cubes, with their own distribution.
class ChRandomShapeCreatorFromFamilies : public ChRandomShapeCreator
{
public:
	ChRandomShapeCreatorFromFamilies() 
		{
			// defaults
			Reset();
		}

			/// Function that creates a random ChBody particle each
			/// time it is called.
	virtual ChSharedPtr<ChBody> RandomGenerate(ChCoordsys<> mcoords) 
		{
			if (family_generators.size() ==0) 
				throw ChException("Error, cannot randomize particles from a zero length vector of samples");

			ChSharedPtr<ChBody> sample; // default null
			double rand = ::chrono::ChRandom();
			for (unsigned int i=0; i <  cumulative_probability.size(); ++i)
			{
				if ( rand < cumulative_probability[i] )
				{
					sample = family_generators[i]->RandomGenerateAndCallbacks(mcoords);
					break;
				}
			}

			sample->SetCoord(mcoords);
			return sample;
		};
	
			/// Call this BEFORE adding a set of samples via AddSample()
	void Reset() 
		{
			family_probability.clear();
			cumulative_probability.clear();
			sum = 0;
			family_generators.clear();
		}
			/// Call this multiple times to add a set of samples.
			/// Each sample is a body with a given probability. 
			/// Finally, use Setup() after you used	AddSample N times	
			/// The sum of probabilities should be 1; otherwise will be normalized.
	void AddFamily(ChSharedPtr<ChRandomShapeCreator> family_generator, double mprobability)
		{
			family_probability.push_back(mprobability);
			sum += mprobability;
			cumulative_probability.push_back(sum);
			family_generators.push_back(family_generator);
		}
			/// Call this when you finished adding samples via AddSample()
	void Setup()
		{
			if (family_probability.size() ==0) 
				return;
			// normalize if integral of atomic probabilities not unitary
			double scale = 1.0/sum;
			for (unsigned int i= 0; i< family_probability.size(); ++i)
			{
				family_probability[i] *= scale;
				cumulative_probability[i]*= scale;
			}
		}
private:
	std::vector<double> family_probability;
	std::vector<double> cumulative_probability;
	double sum;
	std::vector< ChSharedPtr<ChRandomShapeCreator> > family_generators;
};



} // end of namespace particlefactory
} // end of namespace chrono

#endif  
