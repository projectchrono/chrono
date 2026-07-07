# Chrono on AMD GPUs (ROCm)

This guide is for machines whose **GPUs are AMD** (ROCm userland): **CPU PyChrono** next to ML stacks such as **PyTorch ROCm**, optional **HIP**-accelerated **FSI / SPH** (continuum physics), and clear limits so expectations match what Chrono builds today.

---

## 1. Three workflows (pick one)

| Workflow | GPU use in Chrono | NVIDIA CUDA toolkit |
|----------|-------------------|---------------------|
| **A. CPU PyChrono** (core, vehicle, robot, multibody) | None in Chrono | **Not required**. You may still install **PyTorch ROCm** (or other ML stacks) for separate training code. |
| **B. HIP FSI / SPH** (continuum GPU physics) | Yes — set **`CHRONO_GPU_BACKEND=HIP`** and enable **`CH_ENABLE_MODULE_FSI`** | Not used; build with **ROCm** (`hipcc`). |
| **C. Chrono::Sensor**  | N/A | TBD. |

If you only need **simulation + Python bindings** next to a **ROCm** ML stack, **workflow A** is usually enough.

---

## 2. Prerequisites

### All Python binding builds

- **CMake** (see [install Chrono](https://api.projectchrono.org/tutorial_install_chrono.html)), **Ninja** (recommended on Linux).
- **Eigen3** — e.g. `libeigen3-dev` on Debian/Ubuntu, or set `Eigen3_DIR` / `EIGEN3_INCLUDE_DIR` per the core install guide.
- **SWIG** — required for PyChrono; use a distribution package or install a version compatible with your Python.

### Workflow B (HIP / FSI)

- **ROCm** with **HIP** so `hipcc` works.
- **`hipify-perl`** on `PATH` only if you use a mechanical CUDA→HIP pass while developing (see §6).
- Set **`CMAKE_HIP_COMPILER=hipcc`** and **`CHRONO_HIP_ARCHITECTURES`** (or **`CMAKE_HIP_ARCHITECTURES`**) to your GPU ISA, for example:
  - **gfx942** — AMD Instinct MI300-class
  - **gfx90a** — MI200-class
  - **gfx1151** — Ryzen AI Max integrated GPU (Radeon 8060S-class)  
  Use `rocminfo` or vendor documentation for other ASICs.

### Multi-GPU hosts (runtime)

Select devices with **`ROCR_VISIBLE_DEVICES`** (ROCm). On AMD systems, do not rely on `CUDA_VISIBLE_DEVICES`.

---

## 3. Build: CPU PyChrono (workflow A)

Minimal pattern (adjust `CH_ENABLE_MODULE_*` flags to your project):

```bash
sudo apt-get update && sudo apt-get install -y libeigen3-dev swig cmake ninja-build git
git clone https://github.com/projectchrono/chrono.git && cd chrono
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCH_ENABLE_MODULE_PYTHON=ON \
  -DPYTHON_EXECUTABLE="$(which python3)" \
  -DCH_ENABLE_MODULE_CORE=ON \
  -DCH_ENABLE_MODULE_ROBOT=ON
ninja -C build
ninja -C build install   # or set PYTHONPATH per the PYTHON module install guide
python3 -c "import pychrono.core as ch; import pychrono.robot as rb; print('ok', ch.CHRONO_VERSION)"
```

**Sanity check:** `nvcc` is **not** required for this path.

---

## 4. Build: HIP + FSI (workflow B)

Example configuration for **gfx942** (change `CHRONO_HIP_ARCHITECTURES` for your hardware):

```bash
git clone https://github.com/projectchrono/chrono.git && cd chrono
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_HIP_COMPILER=hipcc \
  -DCHRONO_GPU_BACKEND=HIP \
  -DCHRONO_HIP_ARCHITECTURES=gfx942 \
  -DCH_ENABLE_MODULE_FSI=ON \
  -DCH_ENABLE_MODULE_FSI_SPH=ON \
  -DCH_ENABLE_MODULE_PYTHON=ON \
  -DCH_ENABLE_MODULE_CORE=ON
ninja -C build
ninja -C build install
python3 -c "import pychrono.fsi as f; print('fsi ok')"
```

See also the [FSI module installation](https://api.projectchrono.org/module_fsi_installation.html) page and the [HIP section](https://api.projectchrono.org/tutorial_install_chrono.html#hip) of the core install guide.

---

## 5. Build on Windows (native HIP SDK)

The HIP backend also builds **natively on Windows** (no WSL) using the [AMD HIP SDK](https://rocm.docs.amd.com/projects/install-on-windows/en/latest/), version **7.1.1 or newer**. Validated on Windows 11 with a Radeon 8060S (**gfx1151**, Ryzen AI Max): FSI-SPH, CRM terrain, and DEM demos all run on the GPU.

### Prerequisites

- **AMD HIP SDK >= 7.1.1**: provides ROCm clang, the HIP runtime, and the rocThrust / hipCUB / rocPRIM headers.
- **Visual Studio 2022 Build Tools** (C++ workload): supplies the MSVC STL and Windows SDK.
- **CMake >= 3.30** and **Ninja** (both bundled with VS Build Tools).
- **Eigen3**: extract a release and pass `EIGEN3_INCLUDE_DIR`.

### Key differences from Linux

- **One compiler family for everything.** CMake on Windows rejects mixing MSVC `cl` for C++ with clang for HIP. Set `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER`, and `CMAKE_HIP_COMPILER` all to the SDK's `clang++.exe`. It targets the MSVC ABI and uses the MSVC STL, so the result is still a native Windows build.
- **`-DCH_ENABLE_OPENMP=OFF`**: the Windows HIP SDK ships no `libomp`; without this flag the build fails to link with undefined `__kmpc_*` symbols. The GPU modules are unaffected; CPU-side loops run single-threaded.
- Configure and build **inside a VS x64 developer shell**, so that clang and `lld-link` find the MSVC headers and libraries.

### Configure and build (PowerShell, x64 dev shell)

```powershell
& 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\Launch-VsDevShell.ps1' -Arch amd64 -SkipAutomaticLocation
$ROCM = 'C:/Program Files/AMD/ROCm/7.1'
cmake -S . -B build -G Ninja `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_C_COMPILER="$ROCM/bin/clang.exe" `
  -DCMAKE_CXX_COMPILER="$ROCM/bin/clang++.exe" `
  -DCMAKE_HIP_COMPILER="$ROCM/bin/clang++.exe" `
  -DCMAKE_HIP_PLATFORM=amd `
  -DCMAKE_HIP_COMPILER_ROCM_ROOT="$ROCM" `
  -DCMAKE_HIP_ARCHITECTURES=gfx1151 `
  -DCHRONO_GPU_BACKEND=HIP `
  -DCHRONO_HIP_ARCHITECTURES=gfx1151 `
  -DCHRONO_ROCM_ROOT="$ROCM" `
  -DEIGEN3_INCLUDE_DIR='C:/libs/eigen-3.4.0' `
  -DCH_ENABLE_OPENMP=OFF `
  -DCH_ENABLE_MODULE_FSI=ON `
  -DCH_ENABLE_MODULE_DEM=ON `
  -DBUILD_DEMOS=ON
cmake --build build -j 16
```

Change `gfx1151` to your GPU ISA and adjust module flags as needed (e.g. `CH_ENABLE_MODULE_VEHICLE=ON` for the CRM terrain demos).

### Running

- Add the SDK's `bin` directory (e.g. `C:\Program Files\AMD\ROCm\7.1\bin`) to `PATH` so the HIP runtime DLLs resolve.
- Run demos from `<build>\bin`; the Chrono data directory is resolved relative to it.

### Troubleshooting (Windows)

| Symptom | Hint |
|---------|------|
| CMake error "mixes Clang and MSVC or some other CL compatible compiler tool" | Set the C, CXX, and HIP compilers all to the SDK's `clang++.exe` (see above). |
| Link errors with undefined `__kmpc_*` / `omp_*` symbols | Configure with `-DCH_ENABLE_OPENMP=OFF`; the Windows HIP SDK ships no libomp. |
| `lld-link` cannot find MSVC libraries | Configure and build inside a VS x64 developer shell. |

---

## 6. Contributing: CUDA sources and `hipify-perl`

FSI / SPH device code lives under **`src/chrono_fsi/sph/`** (for example **`src/chrono_fsi/sph/physics/*.cu`**). For a **mechanical** pass on one file:

```bash
hipify-perl src/chrono_fsi/sph/physics/SphForceWCSPH.cu > /tmp/SphForceWCSPH.hip.cpp
```

Use the diff as a **checklist**; this repository may compile existing **`.cu`** translation units with **`hipcc`** when **`CHRONO_GPU_BACKEND=HIP`**—follow **`src/chrono_fsi/CMakeLists.txt`** on your branch.

**Typical manual follow-ups** on AMD CDNA (64-wide warps): shuffle / mask patterns, rocPRIM vs CUB shims, device symbol macros, texture vs `__ldg`, and stream ordering. Validate on real hardware before submitting changes.

---

## 7. Troubleshooting

| Symptom | Hint |
|---------|------|
| Guides imply **CUDA** is required for PyChrono on an AMD-only machine | Use **workflow A**; CPU bindings do not require `nvcc`. |
| HIP build errors mentioning **warp** or **mask** | Prefer **`warpSize`**-driven reductions and HIP-friendly shuffles; search issues and HIP port notes for FSI. |
| **`pychrono`** missing after configure | Enable **`CH_ENABLE_MODULE_PYTHON=ON`**, install **SWIG**, set **`PYTHON_EXECUTABLE`**. |

---

## 8. Further reading

- [Installation guides](https://api.projectchrono.org/install_guides.html) (all modules).
- [Install PyChrono](https://api.projectchrono.org/pychrono_installation.html) (conda vs build from source).
- [GPU accelerator support](https://api.projectchrono.org/tutorial_install_chrono.html#gpu) in the core Chrono install chapter.
