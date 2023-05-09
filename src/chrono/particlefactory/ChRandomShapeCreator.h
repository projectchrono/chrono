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

//// RADU
//// What should be done with contact materials?

#ifndef CHRANDOMSHAPECREATOR_H
#define CHRANDOMSHAPECREATOR_H

#include <memory>

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChDistribution.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"

namespace chrono {
namespace particlefactory {

// Forward reference
class ChRandomShapeCreator;


/// BASE class for generators of random ChBody shapes
class ChRandomShapeCreator {
  public:
    ChRandomShapeCreator() {
        callback_post_creation = nullptr;
        add_collision_shape = true;
        add_visualization_asset = true;
    }

    virtual ~ChRandomShapeCreator() {}

    /// Function that creates a random ChBody particle each
    /// time it is called.
    /// Note: it MUST BE IMPLEMENTED by children classes!
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) = 0;

    /// This function does RandomGenerate and also executes the
    /// the custom callback, if provided. Usually no need to override.
    virtual std::shared_ptr<ChBody> RandomGenerateAndCallbacks(ChCoordsys<> mcoords) {
        std::shared_ptr<ChBody> mbody = this->RandomGenerate(mcoords);

        if (callback_post_creation)
            callback_post_creation->OnAddBody(mbody, mcoords, *this);
        return mbody;
    }

    /// Class to be used as a callback interface for some user-defined action to be
    /// taken each time a body is generated and added to the system.
    class AddBodyCallback {
      public:
        virtual ~AddBodyCallback() {}

        /// Callback used to process bodies as they are created and added to the system.
        /// A derived class must implement this function. It can be used to modify the
        /// specified body (e.g., add optional assets, adjust material properties, etc)
        virtual void OnAddBody(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) = 0;
    };

    /// Set the callback function to execute at each
    /// each particle generation
    void RegisterAddBodyCallback(std::shared_ptr<AddBodyCallback> callback) { callback_post_creation = callback; }

    /// Set if the created particles must include the collision
    /// shape(s). Note that this is ON by default. Switching off will
    /// turn off the collision.
    void SetAddCollisionShape(bool addcoll) { this->add_collision_shape = addcoll; }

    /// Set if the created particles must include the visualization
    /// asset(s). This is ON by default. Switching off might be more
    /// memory efficient for very large simulations that are batch-processed only.
    void SetAddVisualizationAsset(bool addvisual) { this->add_visualization_asset = addvisual; }

  protected:
    std::shared_ptr<AddBodyCallback> callback_post_creation;
    bool add_collision_shape;
    bool add_visualization_asset;
};

/// Class for generating spheres with variable radius
/// and density. By default uses constant distributions
/// (all spheres with default radius 0.01) but you can provide your distributions.
class ChRandomShapeCreatorSpheres : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorSpheres() {
        // defaults
        diameter = chrono_types::make_shared<ChConstantDistribution>(0.02);
        density = chrono_types::make_shared<ChConstantDistribution>(1000);

        material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    /// Function that creates a random ChBody particle each
    /// time it is called.
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        double mrad = 0.5 * diameter->GetRandom();
        auto mbody = chrono_types::make_shared<ChBodyEasySphere>(
            mrad, density->GetRandom(), this->add_visualization_asset, this->add_collision_shape, material);
        mbody->SetCoord(mcoords);
        return mbody;
    };

    /// Set the statistical distribution for the random diameter.
    void SetDiameterDistribution(std::shared_ptr<ChDistribution> mdistr) { diameter = mdistr; }

    /// Set the statistical distribution for the random density.
    void SetDensityDistribution(std::shared_ptr<ChDistribution> mdistr) { density = mdistr; }

  private:
    std::shared_ptr<ChDistribution> diameter;
    std::shared_ptr<ChDistribution> density;

    std::shared_ptr<ChMaterialSurfaceNSC> material;
};

/// Class for generating boxes with variable sizes
/// and density. By default uses constant distributions
/// (all boxes with default fixed sizes) but you can provide your distributions.
class ChRandomShapeCreatorBoxes : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorBoxes() {
        // defaults
        x_size = chrono_types::make_shared<ChConstantDistribution>(0.01);
        sizeratioYZ = chrono_types::make_shared<ChConstantDistribution>(1.0);
        sizeratioZ = chrono_types::make_shared<ChConstantDistribution>(1.0);
        density = chrono_types::make_shared<ChConstantDistribution>(1000);

        material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    /// Function that creates a random ChBody particle each
    /// time it is called.
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        double sx = fabs(x_size->GetRandom());
        double sy = fabs(sx * sizeratioYZ->GetRandom());
        double sz = fabs(sx * sizeratioYZ->GetRandom() * sizeratioZ->GetRandom());
        auto mbody = chrono_types::make_shared<ChBodyEasyBox>(
            sx, sy, sz, fabs(density->GetRandom()), this->add_visualization_asset, this->add_collision_shape, material);
        mbody->SetCoord(mcoords);
        return mbody;
    };

    /// Set the statistical distribution for the x size, that is the longest axis.
    void SetXsizeDistribution(std::shared_ptr<ChDistribution> mdistr) { x_size = mdistr; }
    /// Set the statistical distribution for scaling on both Y,Z widths (the lower <1, the thinner, as a needle).
    void SetSizeRatioYZDistribution(std::shared_ptr<ChDistribution> mdistr) { sizeratioYZ = mdistr; }
    /// Set the statistical distribution for scaling on Z width (the lower <1, the flatter, as a chip).
    void SetSizeRatioZDistribution(std::shared_ptr<ChDistribution> mdistr) { sizeratioZ = mdistr; }

    /// Set the statistical distribution for the random density.
    void SetDensityDistribution(std::shared_ptr<ChDistribution> mdistr) { density = mdistr; }

  private:
    std::shared_ptr<ChDistribution> x_size;
    std::shared_ptr<ChDistribution> sizeratioYZ;
    std::shared_ptr<ChDistribution> sizeratioZ;
    std::shared_ptr<ChDistribution> density;

    std::shared_ptr<ChMaterialSurfaceNSC> material;
};

/// Class for generating cylinders with variable diameter
/// and length. By default uses constant distributions
/// (all cylinders are equal) but you can provide your distributions.
class ChRandomShapeCreatorCylinders : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorCylinders() {
        // defaults
        diameter = chrono_types::make_shared<ChConstantDistribution>(0.02);
        length_factor = chrono_types::make_shared<ChConstantDistribution>(2.0);
        density = chrono_types::make_shared<ChConstantDistribution>(1000);

        material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    /// Function that creates a random ChBody particle each
    /// time it is called.
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        double rad = 0.5 * diameter->GetRandom();
        double height = length_factor->GetRandom() * 2.0 * rad;
        auto mbody = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, rad, height,
                                                                   density->GetRandom(), this->add_visualization_asset,
                                                                   this->add_collision_shape, material);
        mbody->SetCoord(mcoords);
        return mbody;
    };

    /// Set the statistical distribution for the diameter.
    void SetDiameterDistribution(std::shared_ptr<ChDistribution> mdistr) { diameter = mdistr; }
    /// Set the statistical distribution for the length ratio (length = diameter*length_factor).
    void SetLengthFactorDistribution(std::shared_ptr<ChDistribution> mdistr) { length_factor = mdistr; }

    /// Set the statistical distribution for the random density.
    void SetDensityDistribution(std::shared_ptr<ChDistribution> mdistr) { density = mdistr; }

  private:
    std::shared_ptr<ChDistribution> diameter;
    std::shared_ptr<ChDistribution> length_factor;
    std::shared_ptr<ChDistribution> density;

    std::shared_ptr<ChMaterialSurfaceNSC> material;
};

/// Class for generating convex hulls with variable chordal size
/// and aspect ratios. By default uses constant distributions
/// (all hulls are equally sized) but you can provide your distributions.
class ChRandomShapeCreatorConvexHulls : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorConvexHulls() {
        // defaults
        npoints = 6;
        chord = chrono_types::make_shared<ChConstantDistribution>(0.01);
        sizeratioYZ = chrono_types::make_shared<ChConstantDistribution>(1.0);
        sizeratioZ = chrono_types::make_shared<ChConstantDistribution>(1.0);
        density = chrono_types::make_shared<ChConstantDistribution>(1000);

        material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    /// Function that creates a random ChBody particle each
    /// time it is called.
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        double mchord = chord->GetRandom();
        double msizeratioYZ = sizeratioYZ->GetRandom();
        double msizeratioZ = sizeratioZ->GetRandom();

        std::vector<ChVector<> > points;
        points.resize(npoints);
        double hsizex = 0;
        double hsizey = 0;
        double hsizez = 0;
        for (int ip = 0; ip < npoints; ++ip) {
            points[ip].x() = ChRandom();
            points[ip].y() = ChRandom();
            points[ip].z() = ChRandom();
            if (fabs(points[ip].x()) > hsizex)
                hsizex = fabs(points[ip].x());
            if (fabs(points[ip].y()) > hsizey)
                hsizey = fabs(points[ip].y());
            if (fabs(points[ip].z()) > hsizez)
                hsizez = fabs(points[ip].z());
        }
        for (int ip = 0; ip < npoints; ++ip) {
            points[ip].x() *= 0.5 * mchord / hsizex;
            points[ip].y() *= msizeratioYZ * (0.5 * mchord / hsizey);
            points[ip].z() *= msizeratioYZ * (0.5 * mchord / hsizez) * msizeratioZ;
        }
        auto mbody = chrono_types::make_shared<ChBodyEasyConvexHull>(
            points, density->GetRandom(), this->add_visualization_asset, this->add_collision_shape, material);
        mbody->SetCoord(mcoords);
        return mbody;
    };

    /// Set the number of random vertexes used to generate each random convex hull.
    /// Note that the final number of vertexes in the hull might be lower since
    /// random points that fall inside the convex hull are not used.
    void SetNpoints(int mnpoints) { npoints = mnpoints; }
    /// Set the statistical distribution for the radius.
    void SetChordDistribution(std::shared_ptr<ChDistribution> mdistr) { chord = mdistr; }
    /// Set the statistical distribution for scaling on both Y,Z widths (the lower <1, the thinner, as a needle).
    void SetSizeRatioYZDistribution(std::shared_ptr<ChDistribution> mdistr) { sizeratioYZ = mdistr; }
    /// Set the statistical distribution for scaling on Z width (the lower <1, the flatter, as a chip).
    void SetSizeRatioZDistribution(std::shared_ptr<ChDistribution> mdistr) { sizeratioZ = mdistr; }

    /// Set the statistical distribution for the random density.
    void SetDensityDistribution(std::shared_ptr<ChDistribution> mdistr) { density = mdistr; }

  private:
    int npoints;
    std::shared_ptr<ChDistribution> chord;
    std::shared_ptr<ChDistribution> sizeratioYZ;
    std::shared_ptr<ChDistribution> sizeratioZ;
    std::shared_ptr<ChDistribution> density;

    std::shared_ptr<ChMaterialSurfaceNSC> material;
};

/// Class for generating worm-like particles, optionally helically twisted.
/// This can be used, for example, to generate shavings or chips as
/// those created in milling and grinding processes, etc.
/// This basic class uses a row of small spheres to approximate the
/// chip shape, in sake of high computational performance in collisions
/// etc.
class ChRandomShapeCreatorShavings : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorShavings() {
        // defaults
        spacing_factor = 0.5;
        diameter = chrono_types::make_shared<ChConstantDistribution>(0.02);
        twistU = chrono_types::make_shared<ChConstantDistribution>(0.00);
        twistV = chrono_types::make_shared<ChConstantDistribution>(0.00);
        lengthratio = chrono_types::make_shared<ChConstantDistribution>(3.0);
        density = chrono_types::make_shared<ChConstantDistribution>(1000);

        material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    }

    /// Function that creates a random ChBody particle each
    /// time it is called.
    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        double mtwistU = twistU->GetRandom();
        double mtwistV = twistV->GetRandom();
        double mdiameter = diameter->GetRandom();
        double mlengthratio = ChMax(1.0, lengthratio->GetRandom());
        double mlength = mdiameter * mlengthratio;
        double mlengthsweep = mlength - mdiameter;
        double targetinterval = mdiameter * spacing_factor;
        double nintervals = ceil(mlengthsweep / targetinterval);
        double realinterval = mlengthsweep / nintervals;
        unsigned int npoints = (unsigned int)nintervals + 1;

        std::vector<ChVector<> > points;
        points.resize(npoints);
        std::vector<double> radii;
        radii.resize(npoints);

        ChFrame<> localframe;
        for (unsigned int ip = 0; ip < npoints; ++ip) {
            radii[ip] = 0.5 * mdiameter;
            points[ip] = localframe.GetPos();
            // compute displacement to next sphere:
            ChFrame<> displacement;
            displacement.SetPos(ChVector<>(realinterval, 0, 0));  // shift on x
            ChQuaternion<> mrotU;
            mrotU.Q_from_AngY(realinterval * mtwistU);
            ChQuaternion<> mrotV;
            mrotV.Q_from_AngX(realinterval * mtwistV);
            displacement.SetRot(mrotU % mrotV);  // rotate on z and y
            localframe.ConcatenatePostTransformation(displacement);
        }

        auto mbody = chrono_types::make_shared<ChBodyEasyClusterOfSpheres>(
            points, radii, density->GetRandom(), this->add_visualization_asset, this->add_collision_shape, material);

        // GetLog() << "Diameter:" << mdiameter << " length:" << mlength << " mass:" << mbody->GetMass() << "\n
        // inertiaXX" << mbody->GetInertiaXX() << "\n inertiaXY:" <<  mbody->GetInertiaXY() << "\n";

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
    void SetSpheresSpacingFactor(double md) { spacing_factor = md; }
    /// Set the statistical distribution for the diameter.
    void SetDiameterDistribution(std::shared_ptr<ChDistribution> mdistr) { diameter = mdistr; }
    /// Set the statistical distribution for the length/diameter ratio.
    void SetLengthRatioDistribution(std::shared_ptr<ChDistribution> mdistr) { lengthratio = mdistr; }
    /// Set the statistical distribution for the twist curvature of line (u direction). Curvature=1/radius
    void SetTwistDistributionU(std::shared_ptr<ChDistribution> mdistr) { twistU = mdistr; }
    /// Set the statistical distribution for the twist curvature of line (v direction). Curvature=1/radius
    void SetTwistDistributionV(std::shared_ptr<ChDistribution> mdistr) { twistV = mdistr; }
    /// Set the statistical distribution for the random density.
    void SetDensityDistribution(std::shared_ptr<ChDistribution> mdistr) { density = mdistr; }

  private:
    double spacing_factor;
    std::shared_ptr<ChDistribution> diameter;
    std::shared_ptr<ChDistribution> twistU;
    std::shared_ptr<ChDistribution> twistV;
    std::shared_ptr<ChDistribution> lengthratio;
    std::shared_ptr<ChDistribution> density;

    std::shared_ptr<ChMaterialSurfaceNSC> material;
};

/// Class for generating spheres from different families,
/// each with given probability. It 'mixes' different
/// ChRandomShapeGenerator sources (among these you can
/// also put other ChRandomShapeCreatorFromFamilies to create
/// tree-like families).
/// This can be used to make bi-modal or multi-modal distributions,
/// for example suppose that you need a mixture of 30% spheres, with
/// their own size distribution, and 70% cubes, with their own distribution.
class ChRandomShapeCreatorFromFamilies : public ChRandomShapeCreator {
  public:
    ChRandomShapeCreatorFromFamilies() {
        // defaults
        probability_mode = PARTICLE_PROBABILITY;

        Reset();
    }

    enum eChFamilyProbabilityMode {
        PARTICLE_PROBABILITY,
        MASS_PROBABILITY,
    };
    /// Function that creates a random ChBody particle each
    /// time it is called.

    virtual std::shared_ptr<ChBody> RandomGenerate(ChCoordsys<> mcoords) override {
        if (family_generators.size() == 0)
            throw ChException("Error, cannot randomize particles from a zero length vector of samples");

        std::shared_ptr<ChBody> sample;  // default null

        // normalize probability of already generated particles
        generated_probability.resize(generated_stats.size());
        double sum = 0;
        for (unsigned int i = 0; i < generated_stats.size(); ++i) {
            sum += generated_stats[i];
        }
        if (sum > 0) {
            for (unsigned int i = 0; i < generated_stats.size(); ++i) {
                generated_probability[i] = generated_stats[i] / sum;
            }
        }
        // Scan families, starting from randomized index, and see which is
        // 'lagging behind'.
        unsigned int tested_family = (int)floor((double)(family_generators.size() - 1) * ChRandom());
        unsigned int ntests = 0;
        while (true) {
            // windup tested family to scan all families if reached end
            if (tested_family >= family_generators.size())
                tested_family = 0;

            // it should never cycle more than once all the families, but for more safety:
            if (ntests >= family_generators.size())
                break;

            if (generated_probability[tested_family] < family_probability[tested_family])
                break;  // Found family to be incremented!

            ++ntests;
            ++tested_family;
        }

        // Generate particle
        sample = family_generators[tested_family]->RandomGenerateAndCallbacks(mcoords);

        if (probability_mode == PARTICLE_PROBABILITY)
            generated_stats[tested_family] += 1;
        if (probability_mode == MASS_PROBABILITY)
            generated_stats[tested_family] += sample->GetMass();

        sample->SetCoord(mcoords);
        return sample;
    };

    /// Call this BEFORE adding a set of samples via AddSample()
    void Reset() {
        family_probability.clear();
        cumulative_probability.clear();
        generated_stats.clear();
        generated_probability.clear();
        sum = 0;
        family_generators.clear();
    }
    /// Call this multiple times to add a set of samples.
    /// Each sample is a body with a given probability.
    /// Finally, use Setup() after you used	AddSample N times
    /// The sum of probabilities should be 1; otherwise will be normalized.
    void AddFamily(std::shared_ptr<ChRandomShapeCreator> family_generator, double mprobability) {
        family_probability.push_back(mprobability);
        sum += mprobability;
        cumulative_probability.push_back(sum);
        family_generators.push_back(family_generator);
        generated_stats.push_back(0.);
        generated_probability.push_back(0.);
    }
    /// Call this when you finished adding samples via AddSample()
    void Setup() {
        if (family_probability.size() == 0)
            return;
        // normalize if integral of atomic probabilities not unitary
        double scale = 1.0 / sum;
        for (unsigned int i = 0; i < family_probability.size(); ++i) {
            family_probability[i] *= scale;
            cumulative_probability[i] *= scale;
            generated_stats[i] = 0.;
            generated_probability[i] = 0.;
        }
    }
    /// Choose how the probability of each family must be considered:
    /// either in terms of number of particles, as by default,
    /// or either in terms of mass.
    void SetProbabilityMode(eChFamilyProbabilityMode mymode) {
        this->probability_mode = mymode;
        for (unsigned int i = 0; i < generated_stats.size(); ++i) {
            generated_stats[i] = 0.;
            generated_probability[i] = 0.;
        }
    }
    /// Report if the probability of each family is in terms of n.of particles or mass
    eChFamilyProbabilityMode GetProbabilityMode() { return this->probability_mode; }

    /// For debugging. For various reasons (ex. very odd masses in generated particles), especially with a
    /// small number of particles, the desired percentuals of masses or n.of particles for
    /// the families are only approximated. Use this vector to get the actual percentuals obtained so far.
    std::vector<double>& GetObtainedPercentuals() { return generated_probability; }

  private:
    eChFamilyProbabilityMode probability_mode;
    std::vector<double> family_probability;
    std::vector<double> cumulative_probability;
    std::vector<double> generated_stats;
    std::vector<double> generated_probability;
    double sum;
    std::vector<std::shared_ptr<ChRandomShapeCreator> > family_generators;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
