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

#ifndef CHBARRELSHAPE_H
#define CHBARRELSHAPE_H

#include "chrono/assets/ChVisualShape.h"

namespace chrono {

/// Class for referencing a barrel shape (a lathed arc) that can be visualized in some way.
class ChApi ChBarrelShape : public ChVisualShape {
  public:
    ChBarrelShape();
    ChBarrelShape(double mHlow, double mHsup, double mRvert, double mRhor, double mRoffset);

    ~ChBarrelShape() {}

    double GetHlow() { return Hlow; }
    void SetHlow(double ms) { Hlow = ms; }

    double GetHsup() { return Hsup; }
    void SetHsup(double ms) { Hsup = ms; }

    double GetRvert() { return Rvert; }
    void SetRvert(double ms) { Rvert = ms; }

    double GetRhor() { return Rhor; }
    void SetRhor(double ms) { Rhor = ms; }

    double GetRoffset() { return Roffset; }
    void SetRoffset(double ms) { Roffset = ms; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    // geometry::ChBarrel gbarrel; // maybe in future the following data can be moved into a ChGeometry class?
    double Hlow;
    double Hsup;
    double Rvert;
    double Rhor;
    double Roffset;
};

CH_CLASS_VERSION(ChBarrelShape, 0)

}  // end namespace chrono

#endif
