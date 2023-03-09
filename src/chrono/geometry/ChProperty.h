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

#ifndef CHC_PROPERTY_H
#define CHC_PROPERTY_H

#include <array>
#include <cmath>
#include <map>

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector2.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/geometry/ChTriangleMesh.h"

namespace chrono {
namespace geometry {

/// Base class for properties to attach to vertexes or particles etc.
/// as arrays of data.
/// These properties are virtual classes so that one can store a list of
/// them in a triangle mesh, or a particle cluster, or a glyph array.
/// These properties are particularly useful for storing data that later must
/// be used in postprocessing, for example a scalar property could be the
/// temperature of particles, to render as falsecolor in Blender or similar tools.

class ChApi ChProperty {
public:
    ChProperty() {};
    virtual ~ChProperty() {};
    
    /// Cloning: 
    virtual ChProperty* clone() = 0;

    /// Get current size of data array.
    virtual size_t GetSize() = 0;

    /// Resize data array to some amount. All internal data will be reset.
    virtual void SetSize(const size_t msize) = 0;

    /// Name of this property.
    std::string name;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) { marchive << CHNVP(name); };
    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) {  marchive >> CHNVP(name); };
};

/// Templated property: a generic array of items of type T.
template <class T>
class ChApi ChPropertyT: public ChProperty {
public:
    ChPropertyT() { min = 0; max = 1.0; };
    ~ChPropertyT() {};
    ChPropertyT(const ChPropertyT& other) { data = other.data; name = other.name; min = other.min; max = other.max; }

    /// Cloning: 
    ChProperty* clone() override { return new ChPropertyT<T>(*this); };
    
    size_t GetSize() override {
        return data.size();
    };
    void SetSize(const size_t msize) override {
        return data.resize(msize);
    };

    /// The data array
    std::vector<T> data;
    
    /// min-max values that can be used to store info on desired falsecolor scale
    double min;
    double max;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override { 
        ChProperty::ArchiveOUT(marchive);
        marchive << CHNVP(data);
        marchive << CHNVP(min);
        marchive << CHNVP(max);
    };
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        ChProperty::ArchiveIN(marchive);
        marchive >> CHNVP(data);
        marchive >> CHNVP(min);
        marchive >> CHNVP(max);
    };
};

/// Data is an array of floats 
class ChApi ChPropertyScalar : public ChPropertyT<double> {};

/// Data is an array of vectors 
class ChApi ChPropertyVector : public ChPropertyT<ChVector<>> {};

/// Data is an array of quaternions (for rotations) 
class ChApi ChPropertyQuaternion : public ChPropertyT<ChQuaternion<>> {};

/// Data is an array of colors 
class ChApi ChPropertyColor : public ChPropertyT<ChColor> {};



}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChPropertyScalar, 0)
CH_CLASS_VERSION(geometry::ChPropertyVector, 0)
CH_CLASS_VERSION(geometry::ChPropertyQuaternion, 0)
CH_CLASS_VERSION(geometry::ChPropertyColor, 0)

}  // end namespace chrono

#endif
