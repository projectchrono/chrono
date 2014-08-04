//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHVISUALIZATION_H
#define CHVISUALIZATION_H


#include "assets/ChAsset.h"
#include "assets/ChColor.h"
#include "core/ChMath.h"


namespace chrono {

/// Base class for assets that define something about visualization (rendering,
/// post processing, etc.)
/// It contains basic information about position, color, and visibility.

class ChApi ChVisualization : public ChAsset {

public:
  virtual ~ChVisualization() {}

    /// Get/Set visible status flag.
  bool IsVisible() const {return visible;}
  void SetVisible(bool mv) {visible = mv;}

    /// Get/Set the color of the surface. This information could be used by
    /// visualization postprocessing.
  const ChColor& GetColor() const {return color;}
  void SetColor(const ChColor& mc) {color = mc;}

    /// Get/Set the fading amount, 0 <= fading <= 1.
    /// If fading = 0, no transparency of surface,
    /// If fading = 1, surface is completely transparent.
  float GetFading() const {return fading;}
  void SetFading(const float mc) {fading = mc;}

    // DATA
  ChVector<>   Pos;     /// Position of Asset
  ChMatrix33<> Rot;     /// Rotation of Asset

protected:
    /// Constructor.
    /// Protected because ChVisualization should not be constructed directly.
  ChVisualization() : Pos(0), Rot(1), visible(true), fading(0) {}

  bool    visible;
  ChColor color;
  float   fading;

};


} // END_OF_NAMESPACE____


#endif
