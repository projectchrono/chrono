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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Utility class for generating fluid markers.
// =============================================================================

#ifndef CH_FSI_UTILS_GENERATORBCE_H
#define CH_FSI_UTILS_GENERATORBCE_H

#include <thrust/host_vector.h>
#include <string>
#include "chrono/ChConfig.h"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChElementCableANCF;
class ChElementShellANCF_3423;
}

namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// This utility function converts a given position and orientation, specified
/// with respect to a body's reference frame, into a frame defined with respect
/// to the body's centroidal frame. Note that by default, a body's reference
/// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const ChVector<>& pos);

/// This utility function converts a given position and orientation, specified
/// with respect to a body's reference frame, into a frame defined with respect
/// to the body's centroidal frame. Note that by default, a body's reference
/// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const Real3& pos3);

/// Create BCE particles from a sphere.
void CreateBCE_On_Sphere(thrust::host_vector<Real4>& posRadBCE, Real rad, std::shared_ptr<SimParams> paramsH);

/// Create BCE particles from the surface of a sphere.
void CreateBCE_On_surface_of_Sphere(thrust::host_vector<Real4>& posRadBCE, Real rad, Real kernel_h);

/// Create BCE particles from a cylinder.
void CreateBCE_On_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                           Real cyl_rad,
                           Real cyl_h,
                           std::shared_ptr<SimParams> paramsH,
                           Real kernel_h,
                           bool cartesian = true);

/// Create BCE particles from a cylinder annulus.
void CreateBCE_On_Cylinder_Annulus(thrust::host_vector<Real4>& posRadBCE,
                                   Real rad_in,
                                   Real rad_out,
                                   Real cyl_h,
                                   std::shared_ptr<SimParams> paramsH,
                                   Real kernel_h,
                                   bool cartesian = true);

/// Create BCE particles from a cone.
void CreateBCE_On_Cone(thrust::host_vector<Real4>& posRadBCE,
                       Real cone_rad,
                       Real cone_h,
                       std::shared_ptr<SimParams> paramsH,
                       Real kernel_h,
                       bool cartesian = true);

/// Create BCE particles from the surface of a cylinder.
void CreateBCE_On_surface_of_Cylinder(thrust::host_vector<Real4>& posRadBCE,
                                      thrust::host_vector<Real3>& normals,
                                      Real cyl_rad,
                                      Real cyl_h,
                                      Real spacing);

/// Create BCE particles from box.
void CreateBCE_On_Box(thrust::host_vector<Real4>& posRadBCE,
                      const Real3& hsize,
                      int face,
                      std::shared_ptr<SimParams> paramsH);

/// Load BCE particles from a file.
void LoadBCE_fromFile(thrust::host_vector<Real4>& posRadBCE, std::string fileName, double scale = 1, double hsml = 1);

/// Create BCE particles from an ANCF cable element.
void CreateBCE_On_ChElementCableANCF(thrust::host_vector<Real4>& posRadBCE,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::shared_ptr<chrono::fea::ChElementCableANCF> cable,
                                     std::vector<int> remove,
                                     bool multiLayer = true,
                                     bool removeMiddleLayer = false,
                                     int SIDE = 1);

/// Create BCE particles from an ANCF shell element.
void CreateBCE_On_ChElementShellANCF(thrust::host_vector<Real4>& posRadBCE,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                                     std::vector<int> remove,
                                     std::vector<int> remove_s,
                                     bool multiLayer = true,
                                     bool removeMiddleLayer = false,
                                     int SIDE = -2,
                                     double kernel_h = 0);

/// @} fsi_utils

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
