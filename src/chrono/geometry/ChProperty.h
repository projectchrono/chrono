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

/// @addtogroup chrono_geometry
/// @{

/// Base class for properties to attach to vertices or particles as arrays of data.
/// These properties are virtual classes so that one can store a list of them in a triangle mesh, or a particle cluster,
/// or a glyph array. They are particularly useful for storing data that later must be used in postprocessing, for
/// example a scalar property could be the temperature of particles to be rendered as falsecolor.
class ChApi ChProperty {
  public:
    ChProperty() {}
    virtual ~ChProperty() {}

    virtual ChProperty* clone() = 0;

    /// Get current size of data array.
    virtual size_t GetSize() = 0;

    /// Resize data array to some amount. All internal data will be reset.
    virtual void SetSize(const size_t msize) = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    std::string name;  ///< name of this property
};

/// Templated property: a generic array of items of type T.
template <class T>
class ChApi ChPropertyT : public ChProperty {
  public:
    ChPropertyT() : min(0), max(1) {}

    ChPropertyT(const ChPropertyT& other) {
        data = other.data;
        name = other.name;
        min = other.min;
        max = other.max;
    }

    ~ChPropertyT() {}

    ChProperty* clone() override { return new ChPropertyT<T>(*this); }

    size_t GetSize() override { return data.size(); }

    void SetSize(const size_t msize) override { return data.resize(msize); }

    virtual void ArchiveOut(ChArchiveOut& archive_out) override {
        ChProperty::ArchiveOut(archive_out);
        archive_out << CHNVP(data);
        archive_out << CHNVP(min);
        archive_out << CHNVP(max);
    }

    virtual void ArchiveIn(ChArchiveIn& archive_in) override {
        ChProperty::ArchiveIn(archive_in);
        archive_in >> CHNVP(data);
        archive_in >> CHNVP(min);
        archive_in >> CHNVP(max);
    }

    // min-max values that can be used to store info on desired falsecolor scale
    double min;
    double max;

    std::vector<T> data;  ///< data array
};

/// Data is an array of floats
class ChApi ChPropertyScalar : public ChPropertyT<double> {};

/// Data is an array of vectors
class ChApi ChPropertyVector : public ChPropertyT<ChVector3d> {};

/// Data is an array of quaternions (for rotations)
class ChApi ChPropertyQuaternion : public ChPropertyT<ChQuaternion<>> {};

/// Data is an array of colors
class ChApi ChPropertyColor : public ChPropertyT<ChColor> {};

/// @} chrono_geometry

CH_CLASS_VERSION(ChPropertyScalar, 0)
CH_CLASS_VERSION(ChPropertyVector, 0)
CH_CLASS_VERSION(ChPropertyQuaternion, 0)
CH_CLASS_VERSION(ChPropertyColor, 0)

}  // end namespace chrono

#endif
