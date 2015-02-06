//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATERIALSURFACEDEM_H
#define CHMATERIALSURFACEDEM_H


#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChContactDEM.h"


/// Class for material surface data for DEM contact
namespace chrono {

// Forward references
class ChBodyDEM;


struct ChCompositeMaterialDEM
{
  float E_eff;                ///< Effective elasticity modulus
  float G_eff;                ///< Effective shear modulus
  float mu_eff;               ///< Effective coefficient of friction
  float cr_eff;               ///< Effective coefficient of restitution
  float cohesion_eff;         ///< Effective cohesion force

  float kn;
  float kt;
  float gn;
  float gt;
};


class ChApi ChMaterialSurfaceDEM : public ChShared
{
public:

  float young_modulus;         ///< Young's modulus (elastic modulus)
  float poisson_ratio;         ///< Poisson ratio

  float static_friction;       ///< Static coefficient of friction
  float sliding_friction;      ///< Kinetic coefficient of friction

  float restitution;           ///< Coefficient of restitution

  float cohesion;              ///< Constant cohesion force

  float kn;                    ///< user-specified normal stiffness coefficient
  float kt;                    ///< user-specified tangential stiffness coefficient
  float gn;                    ///< user-specified normal damping coefficient
  float gt;                    ///< user-specified tangential damping coefficient

  ChMaterialSurfaceDEM();
  ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other);
  ~ChMaterialSurfaceDEM() {}

  /// Young's modulus and Poisson ratio.
  float GetYoungModulus() const    { return young_modulus; }
  void  SetYoungModulus(float val) { young_modulus = val; }

  float GetPoissonRatio() const    { return poisson_ratio; }
  void  SetPoissonRatio(float val) { poisson_ratio = val; }

  /// Static and kinetic friction coefficients.
  /// Usually in 0..1 range, rarely above. Default 0.6
  float GetSfriction() const       { return static_friction; }
  void  SetSfriction(float val)    { static_friction = val; }

  float GetKfriction() const       { return sliding_friction; }
  void  SetKfriction(float val)    { sliding_friction = val; }

  /// Set both static friction and kinetic friction at once, with same value.
  void  SetFriction(float val)     { SetSfriction(val); SetKfriction(val); }

  /// Normal restitution coefficient 
  float GetRestitution() const     { return restitution; }
  void  SetRestitution(float val)  { restitution = val; }

  /// Constant cohesion force
  float GetCohesion() const        { return cohesion; }
  void  SetCohesion(float val)     { cohesion = val; }

  /// Stiffness and damping coefficients
  float GetKn() const { return kn; }
  float GetKt() const { return kt; }
  float GetGn() const { return gn; }
  float GetGt() const { return gt; }

  void SetKn(float val) { kn = val; }
  void SetKt(float val) { kt = val; }
  void SetGn(float val) { gn = val; }
  void SetGt(float val) { gt = val; }

  /// Calculate composite material properties
  static ChCompositeMaterialDEM
    CompositeMaterial(const ChSharedPtr<ChMaterialSurfaceDEM>& mat1,
    const ChSharedPtr<ChMaterialSurfaceDEM>& mat2);

  /// Method to allow serializing transient data into in ascii
  /// as a readable item, for example   "chrono::GetLog() << myobject;"
  virtual void StreamOUT(ChStreamOutAscii& mstream)
  {
    mstream << "Material DEM \n";
  }

  /// Method to allow serializing transient data into a persistent
  /// binary archive (ex: a file).
  virtual void StreamOUT(ChStreamOutBinary& mstream)
  {
    // class version number
    mstream.VersionWrite(1);

    // deserialize parent class too
    //ChShared::StreamOUT(mstream); // nothing 

    // stream out all member data
    mstream << young_modulus;
    mstream << poisson_ratio;
    mstream << static_friction;
    mstream << sliding_friction;
    mstream << restitution;
    mstream << cohesion;

    mstream << kn;
    mstream << kt;
    mstream << gn;
    mstream << gt;
  }

  /// Operator to allow deserializing a persistent binary archive (ex: a file)
  /// into transient data.
  virtual void StreamIN(ChStreamInBinary& mstream)
  {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    //ChShared::StreamIN(mstream); // nothing 

    // stream in all member data
    mstream >> young_modulus;
    mstream >> poisson_ratio;
    mstream >> static_friction;
    mstream >> sliding_friction;
    mstream >> restitution;
    mstream >> cohesion;

    mstream >> kn;
    mstream >> kt;
    mstream >> gn;
    mstream >> gt;
  }

};


} // end namespace chrono


#endif
