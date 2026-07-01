# External `scm_gpu` HIP library

Chrono does **not** vendor the `scm_gpu` contact-force kernel. Build it separately with ROCm (`hipcc`), then point CMake at the install prefix.

## Build

```bash
cmake -S scm_gpu -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=hipcc \
  -DCMAKE_HIP_COMPILER=hipcc \
  -DCMAKE_HIP_ARCHITECTURES=gfx942 \
  -DCMAKE_INSTALL_PREFIX=/path/to/scm_gpu_install

ninja -C build install
```

Use `gfx90a` for MI210 / MI250X. Host Chrono code remains on **g++**; only the external library and Chrono HIP glue use the HIP toolchain.

## Configure Chrono

API headers (`SCMGpu.h`, `SCMGpuTypes.h`) ship in-tree. Point CMake at the **library** install:

```bash
cmake -S chrono -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_VEHICLE_SCM_GPU=ON \
  -DCHRONO_SCM_GPU_LIB_DIR=/path/to/scm_gpu_install/lib \
  -DCMAKE_HIP_ARCHITECTURES=gfx942
```

If HIP or `scm_gpu_core` is missing, CMake warns and disables SCM GPU automatically.

## Validation

Run the parity and scaling tests shipped with your `scm_gpu` build before enabling `CHRONO_SCM_GPU=1` in production scenarios. Target: rtol `1e-5`, atol `1e-7` vs CPU reference at ≥ 65k hits.
