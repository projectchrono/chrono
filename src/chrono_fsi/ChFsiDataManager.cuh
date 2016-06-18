// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Base class for managing data in chrono_fsi, aka fluid system.//
// =============================================================================

#ifndef CH_FSI_DATAMANAGER_H_
#define CH_FSI_DATAMANAGER_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/custom_math.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>

namespace chrono {
namespace fsi {

// typedef device iterators for shorthand sph operation
typedef thrust::device_vector<Real3>::iterator r3IterD;
typedef thrust::device_vector<Real4>::iterator r4IterD;
typedef thrust::tuple<r3IterD, r3IterD, r4IterD> iterTupleSphD;
typedef thrust::zip_iterator<iterTupleSphD> zipIterSphD;

// typedef host iterators for shorthand
typedef thrust::host_vector<Real3>::iterator r3IterH;
typedef thrust::host_vector<Real4>::iterator r4IterH;
typedef thrust::tuple<r3IterH, r3IterH, r4IterH> iterTupleH;
typedef thrust::zip_iterator<iterTupleH> zipIterSphH;

// typedef device iterators for shorthand rigid operations
typedef thrust::tuple<r3IterD, r4IterD, r3IterD, r4IterD, r3IterD, r3IterD>
    iterTupleRigidD;
typedef thrust::zip_iterator<iterTupleRigidD> zipIterRigidD;

// typedef device iterators for shorthand rigid operations
typedef thrust::tuple<r3IterH, r4IterH, r3IterH, r4IterH, r3IterH, r3IterH>
    iterTupleRigidH;
typedef thrust::zip_iterator<iterTupleRigidH> zipIterRigidH;

// typedef device iterators for shorthand chrono bodies operations
typedef thrust::tuple<r3IterH, r3IterH, r3IterH, r4IterH, r3IterH, r3IterH>
    iterTupleChronoBodiesH;
typedef thrust::zip_iterator<iterTupleChronoBodiesH> zipIterChronoBodiesH;

/**
 * @brief Number of fluid markers, solid bodies, solid markers, boundary markers
 * @details
 * 		The description of each variable is in front of it
 */
// Arman : see if you need all of these guys since you rely on chrono for rigid
// and flex
struct NumberOfObjects {
  int numRigidBodies;      /* Number of rigid bodies */
  int numFlexBodies;       /* Number of Flexible bodies*/
  int numFluidMarkers;     /* Number of fluid SPH markers*/
  int numBoundaryMarkers;  /* Number of boundary SPH markers */
  int startRigidMarkers;   /* */
  int startFlexMarkers;    /* */
  int numRigid_SphMarkers; /* */
  int numFlex_SphMarkers;  /* */
  int numAllMarkers;       /* Total number of SPH markers */
};

class SphMarkerDataD {
public:
  thrust::device_vector<Real3> posRadD;
  thrust::device_vector<Real3> velMasD;
  thrust::device_vector<Real4> rhoPresMuD;

  zipIterSphD iterator();

  // resize
  void resize(int s);

private:
};

class SphMarkerDataH {
public:
  thrust::host_vector<Real3>
      posRadH; // do not set the size here since you are using push back later
  thrust::host_vector<Real3> velMasH;
  thrust::host_vector<Real4> rhoPresMuH;

  zipIterSphH iterator();

  // resize
  void resize(int s);

private:
};

// dummy fsi bodies
class FsiBodiesDataH {
public:
  thrust::host_vector<Real3> posRigid_fsiBodies_H;
  thrust::host_vector<Real4> velMassRigid_fsiBodies_H;
  thrust::host_vector<Real3> accRigid_fsiBodies_H;
  thrust::host_vector<Real4> q_fsiBodies_H;
  thrust::host_vector<Real3> omegaVelLRF_fsiBodies_H;
  thrust::host_vector<Real3> omegaAccLRF_fsiBodies_H;

  zipIterRigidH iterator();

  // resize
  void resize(int s);

private:
};

class FsiBodiesDataD {
public:
  thrust::device_vector<Real3> posRigid_fsiBodies_D;
  thrust::device_vector<Real4> velMassRigid_fsiBodies_D;
  thrust::device_vector<Real3> accRigid_fsiBodies_D;
  thrust::device_vector<Real4> q_fsiBodies_D;
  thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D;
  thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;

  zipIterRigidD iterator();
  void CopyFromH(const FsiBodiesDataH &other);
  FsiBodiesDataD &operator=(const FsiBodiesDataD &other);

  // resize
  void resize(int s);

private:
};

class ProximityDataD {
public:
  thrust::device_vector<uint> gridMarkerHashD;  //(numAllMarkers);
  thrust::device_vector<uint> gridMarkerIndexD; //(numAllMarkers);
  thrust::device_vector<uint>
      cellStartD; //(m_numGridCells); // Index of start cell in sorted list
  thrust::device_vector<uint>
      cellEndD; //(m_numGridCells); // Index of end cell in sorted list
  thrust::device_vector<uint> mapOriginalToSorted;

  // resize
  void resize(int numAllMarkers);

private:
};

class ChronoBodiesDataH {
public:
  ChronoBodiesDataH() {}
  ChronoBodiesDataH(int s);
  thrust::host_vector<Real3> pos_ChSystemH;
  thrust::host_vector<Real3> vel_ChSystemH;
  thrust::host_vector<Real3> acc_ChSystemH;
  thrust::host_vector<Real4> quat_ChSystemH;
  thrust::host_vector<Real3> omegaVelGRF_ChSystemH;
  thrust::host_vector<Real3> omegaAccGRF_ChSystemH;

  zipIterChronoBodiesH iterator();

  // resize
  void resize(int s);

private:
};

// make them classes
class FsiGeneralData {
public:
  // ----------------
  //  host
  // ----------------
  // fluidfsiBodeisIndex
  thrust::host_vector<::int4> referenceArray;
  // ----------------
  //  device
  // ----------------
  // fluid
  thrust::device_vector<Real4> derivVelRhoD;
  thrust::device_vector<Real3> vel_XSPH_D;

  // BCE
  thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;
  thrust::device_vector<uint> rigidIdentifierD;

  // fsi bodies
  thrust::device_vector<Real3> rigid_FSI_ForcesD;
  thrust::device_vector<Real3> rigid_FSI_TorquesD;

private:
};

class CH_FSI_API ChFsiDataManager {
public:
  ChFsiDataManager();
  ~ChFsiDataManager();

  void AddSphMarker(Real3 pos, Real3 vel, Real4 rhoPresMu);
  void ResizeDataManager();

  NumberOfObjects numObjects;

  SphMarkerDataD sphMarkersD1;
  SphMarkerDataD sphMarkersD2;
  SphMarkerDataD sortedSphMarkersD;
  SphMarkerDataH sphMarkersH;

  FsiBodiesDataD fsiBodiesD1;
  FsiBodiesDataD fsiBodiesD2;
  FsiBodiesDataH fsiBodiesH;

  FsiGeneralData fsiGeneralData;

  ProximityDataD markersProximityD;

private:
  void ArrangeDataManager();
  void ConstructReferenceArray();
  void InitNumObjects();
  void CalcNumObjects();
};

} // end namespace fsi
} // end namespace chrono

#endif /* CH_FSI_DATAMANAGER_H_ */
