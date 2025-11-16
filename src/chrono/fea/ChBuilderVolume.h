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

#ifndef CHBUILDERVOLUME_H
#define CHBUILDERVOLUME_H

#include "chrono/core/ChApiCE.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementHexahedron8Face.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"
#include "chrono/fea/ChFieldElementTetrahedron4Face.h"

namespace chrono {

namespace fea {

/// @addtogroup chrono_fea
/// @{


// ----------------------------------------------------------------------------------


/// Helper class to store 3d arrays of items, with i,j,k indexing

template <class T>
class Ch3DArrayOfItems {
    
public:
    Ch3DArrayOfItems() {}

    Ch3DArrayOfItems(size_t n_, size_t m_, size_t k_)
        : n(n_), m(m_), k(k_), data(n_* m_* k_) {}

    void Resize(size_t n_, size_t m_, size_t k_) {
        n = n_; 
        m = m_;
        k = k_;
        data.resize(n_ * m_ * k_);
    };

    /// Access contained item at i,j,j index in the 3d array
    T& at(size_t i, size_t j, size_t l) {
        return data[i * m * k + j * k + l];
    }
      
    /// Low level access (ex. for quick iteration on all items)
    std::vector<T>& list() {
        return data;
    }

private: 
    std::vector<T> data;
    size_t n, m, k;
};


// ----------------------------------------------------------------------------------


/// Helper class to store 2d arrays of items, with i,j indexing

template <class T>
class Ch2DArrayOfItems {

public:
    Ch2DArrayOfItems() {}

    Ch2DArrayOfItems(size_t n_, size_t m_)
        : n(n_), m(m_), k(k_), data(n_* m_) {}

    void Resize(size_t n_, size_t m_) {
        n = n_;
        m = m_;
        data.resize(n_ * m_);
    };

    /// Access contained item at i,j,j index in the 3d array
    T& at(size_t i, size_t j) {
        return data[i * m + j ];
    }

    /// Low level access (ex. for quick iteration on all items)
    std::vector<T>& list() {
        return data;
    }

private:
    std::vector<T> data;
    size_t n, m;
};


class Ch3DArrayOfNodes : public Ch3DArrayOfItems<std::shared_ptr<ChNodeFEAfieldXYZ>> {};

class Ch3DArrayOfHexa8 : public Ch3DArrayOfItems<std::shared_ptr<ChFieldElementHexahedron8>> {};



// ----------------------------------------------------------------------------------



/// For testing purposes.
/// Utility class for creating a box filled with a 3d structured grid of hexahedral
/// finite elements. Adding nodes and finite elements to fields and domains is up to you.

class ChApi ChBuilderVolumeBox {

public:

    void BuildVolume(const ChFrame<>& frame,  ///< origin and rotation of the box being meshed
                    int nlayers_x,
                    int nlayers_y,
                    int nlayers_z,
                    double W_x,
                    double W_y,
                    double W_z);

    // results here:

    Ch3DArrayOfNodes nodes;

    Ch3DArrayOfHexa8  elements;

};


// ----------------------------------------------------------------------------------

/// For testing purposes.
/// Utility class for creating a box filled with a 3d structured grid of tetrahedral
/// finite elements. Adding nodes and finite elements to fields and domains is up to you.

class ChApi ChBuilderVolumeBoxTetra {

public:

    void BuildVolume(const ChFrame<>& frame,  ///< origin and rotation of the box being meshed
        int nlayers_x,
        int nlayers_y,
        int nlayers_z,
        double W_x,
        double W_y,
        double W_z);

    // results here:

    Ch3DArrayOfNodes nodes;

    Ch3DArrayOfItems<std::array<std::shared_ptr<ChFieldElementTetrahedron4>, 5 > >  elements; // grouped as 5 tetra evey cubic cell

};


/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
