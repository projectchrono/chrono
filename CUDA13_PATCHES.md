# Chrono 9.0.1 CUDA 13.0 Compatibility Patches

This document describes the patches applied to Chrono 9.0.1 to make it compatible with CUDA 13.0 (CUDA Toolkit 13.0.48).

## Overview

CUDA 13.0 introduced several breaking API changes that affect Chrono's GPU DEM module:

1. `cudaMemAdvise` API signature changed
2. `cub::Inequality` functor removed from CUB 2.x
3. Managed memory access patterns changed (stricter)

## Applied Patches

### Patch 1: cudaMemAdvise API Change

**File**: `src/chrono_dem/physics/ChSystemDem_impl.cpp`

**Problem**: CUDA 13.0 changed `cudaMemAdvise` from taking an `int` device ID to taking a `cudaMemLocation` struct.

**Solution**: Wrap the device ID in a `cudaMemLocation` struct.

```diff
--- a/src/chrono_dem/physics/ChSystemDem_impl.cpp
+++ b/src/chrono_dem/physics/ChSystemDem_impl.cpp
@@ -1205,11 +1205,12 @@ void ChSystemDem_impl::initializeSpheres() {
     int dev_ID;
     demErrchk(cudaGetDevice(&dev_ID));

-    demErrchk(cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetPreferredLocation, dev_ID));
-    demErrchk(cudaMemAdvise(sphere_data, sizeof(*sphere_data), cudaMemAdviseSetPreferredLocation, dev_ID));
+    // CUDA 13.0+: cudaMemAdvise now requires cudaMemLocation instead of int device ID
+    cudaMemLocation devLoc = {cudaMemLocationTypeDevice, dev_ID};
+    demErrchk(cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetPreferredLocation, devLoc));
+    demErrchk(cudaMemAdvise(sphere_data, sizeof(*sphere_data), cudaMemAdviseSetPreferredLocation, devLoc));
```

**Lines affected**: ~1208-1215

---

### Patch 2: cub::Inequality Removal

**File**: `src/chrono_dem/cuda/ChDem_SMC.cuh`

**Problem**: CUB 2.x (bundled with CUDA 13.0) removed the `cub::Inequality` functor that was used for exclusive scans.

**Solution**: Add a compatible replacement functor.

```diff
--- a/src/chrono_dem/cuda/ChDem_SMC.cuh
+++ b/src/chrono_dem/cuda/ChDem_SMC.cuh
@@ -21,6 +21,16 @@
 namespace chrono {
 namespace dem {

+// CUDA 13.0+ compatibility: cub::Inequality was removed in CUB 2.x
+// This provides a compatible replacement for exclusive scans
+namespace chrono_dem_compat {
+struct Inequality {
+    template <typename T>
+    __host__ __device__ __forceinline__ bool operator()(const T& a, const T& b) const {
+        return a != b;
+    }
+};
+}  // namespace chrono_dem_compat
+
 // Observation: The names of the quantities below are recorded, in words, in ChDemUtilities.h. If you change
 // anything here, update the corresponding human-readble definition there
```

**Additional change**: Replace usage at line ~247:

```diff
-    cub::DeviceScan::ExclusiveSumByKey(..., cub::Inequality());
+    cub::DeviceScan::ExclusiveSumByKey(..., chrono_dem_compat::Inequality());
```

---

### Patch 3: Managed Memory Host Access

**File**: `src/chrono_dem/cuda/ChDem_SMC.cu`

**Problem**: CUDA 13.0 has stricter managed memory access patterns. Direct pointer dereference of managed memory on host after GPU kernels causes "invalid device ordinal" or "illegal memory access" errors.

**Solution**: Use explicit `cudaMemcpy` to copy results to host instead of direct pointer access.

**Affected functions** (4 locations):

1. `computeArray3SquaredSum()` - line ~45-47
2. `GetMaxParticleZ()` - line ~78-81
3. `GetNumParticleAboveZ()` - line ~104-107
4. `GetNumParticleAboveX()` - line ~130-133

```diff
--- a/src/chrono_dem/cuda/ChDem_SMC.cu
+++ b/src/chrono_dem/cuda/ChDem_SMC.cu
@@ -42,7 +42,10 @@ __host__ float ChSystemDem_impl::computeArray3SquaredSum(...) {
     ...
     demErrchk(cudaDeviceSynchronize());
     demErrchk(cudaPeekAtLastError());
-    return *(sphere_data->sphere_stats_buffer + num_spheres);
+    // CUDA 13.0+: Copy result to host instead of direct managed memory access
+    float result;
+    demErrchk(cudaMemcpy(&result, sphere_data->sphere_stats_buffer + num_spheres, sizeof(float), cudaMemcpyDeviceToHost));
+    return result;
 }
```

Similar patterns applied to `GetMaxParticleZ()` (float), `GetNumParticleAboveZ()` (unsigned int), and `GetNumParticleAboveX()` (unsigned int).

---

## Known Limitations

### `GetParticlesKineticEnergy()` - Managed memory robustness

`GetParticlesKineticEnergy()` ultimately calls `ChSystemDem_impl::ComputeTotalKE()`, which uses `computeArray3SquaredSum()` and the `elementalArray3Squared` kernel over the velocity arrays (`pos_X_dt`, `pos_Y_dt`, `pos_Z_dt`).

**Symptoms** (in limited Unified Memory environments): `cudaErrorIllegalAddress` / "an illegal memory access was encountered" when the kernel reads managed velocity arrays.

**Root cause**: the velocity vectors use `cudallocator<float>` (managed allocations). On systems where `cudaDevAttrConcurrentManagedAccess == 0` (common on Windows/WSL2), kernels may not safely dereference managed pointers.

**Mitigation implemented**:

- On full Unified Memory systems (`cudaDevAttrConcurrentManagedAccess == 1`), keep the fast path and launch kernels directly over managed arrays.
- On limited Unified Memory systems (`cudaDevAttrConcurrentManagedAccess == 0`), stage managed vectors into temporary device allocations before launching the kernel and reduce on-device; copy the scalar reduction result back with `cudaMemcpy`.

**Status**: This avoids the known crash mode without requiring a full allocator rewrite. A deeper fix would migrate the DEM-owned velocity arrays to device allocations (and explicitly maintain host mirrors) for all code paths.

---

## Build Configuration

These patches were tested with:

- **CUDA Toolkit**: 13.0.48
- **Chrono Version**: 9.0.1 (tag 9.0.1)
- **GPUs**: RTX 5070 Ti (sm_120), RTX 3090 Ti (sm_86)
- **CUDA Architectures**: 86, 89, 120
- **Compiler**: GCC 11.x with nvcc

**CMake Configuration**:

```bash
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES="86;89;120" \
  -DENABLE_MODULE_DEM=ON \
  -DENABLE_MODULE_MULTICORE=ON \
  -DENABLE_MODULE_PYTHON=ON \
  -DBLAZE_INSTALL_DIR=/path/to/blaze \
  ..
```

---

## Verification

Run the test script to verify patches:

```python
import sys
sys.path.insert(0, 'src/ulf_toolkit/adapters/chrono_gpu_bindings/build')
import chrono_dem_gpu
import numpy as np

# Create system
box = chrono_dem_gpu.ChVector3f(10.0, 10.0, 10.0)
origin = chrono_dem_gpu.ChVector3f(0.0, 0.0, 5.0)
dem = chrono_dem_gpu.ChSystemDem(0.1, 2500.0, box, origin)

# Configure
dem.SetKn_SPH2SPH(1e6)
dem.SetGn_SPH2SPH(1e4)
dem.SetGravitationalAcceleration(chrono_dem_gpu.ChVector3f(0.0, 0.0, -9.81))

# Add particles
positions = np.random.uniform(-4, 4, (100, 3)).astype(np.float32)
positions[:, 2] = np.random.uniform(2, 8, 100)
dem.SetParticles(positions)

# Initialize and run
dem.SetFixedStepSize(0.0001)
dem.Initialize()
for _ in range(10):
    dem.AdvanceSimulation(0.01)

print("GPU DEM working:", dem.GetNumParticles(), "particles")
```

---

## References

- [CUDA Toolkit release notes](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/)
- [CUB (CCCL) documentation](https://nvidia.github.io/cccl/cub/)
- [Chrono API documentation](https://api.projectchrono.org/)

---

*Patches applied: 2026-01-01*
*Maintainer: AXIOM-GULF Project*
