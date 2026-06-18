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
- **`hipify-perl`** on `PATH` only if you use a mechanical CUDA→HIP pass while developing (see §5).
- Set **`CMAKE_HIP_COMPILER=hipcc`** and **`CHRONO_HIP_ARCHITECTURES`** (or **`CMAKE_HIP_ARCHITECTURES`**) to your GPU ISA, for example:
  - **gfx942** — AMD Instinct MI300-class
  - **gfx90a** — MI200-class  
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

## 5. Contributing: CUDA sources and `hipify-perl`

FSI / SPH device code lives under **`src/chrono_fsi/sph/`** (for example **`src/chrono_fsi/sph/physics/*.cu`**). For a **mechanical** pass on one file:

```bash
hipify-perl src/chrono_fsi/sph/physics/SphForceWCSPH.cu > /tmp/SphForceWCSPH.hip.cpp
```

Use the diff as a **checklist**; this repository may compile existing **`.cu`** translation units with **`hipcc`** when **`CHRONO_GPU_BACKEND=HIP`**—follow **`src/chrono_fsi/CMakeLists.txt`** on your branch.

**Typical manual follow-ups** on AMD CDNA (64-wide warps): shuffle / mask patterns, rocPRIM vs CUB shims, device symbol macros, texture vs `__ldg`, and stream ordering. Validate on real hardware before submitting changes.

---

## 6. Troubleshooting

| Symptom | Hint |
|---------|------|
| Guides imply **CUDA** is required for PyChrono on an AMD-only machine | Use **workflow A**; CPU bindings do not require `nvcc`. |
| HIP build errors mentioning **warp** or **mask** | Prefer **`warpSize`**-driven reductions and HIP-friendly shuffles; search issues and HIP port notes for FSI. |
| **`pychrono`** missing after configure | Enable **`CH_ENABLE_MODULE_PYTHON=ON`**, install **SWIG**, set **`PYTHON_EXECUTABLE`**. |

---

## 7. Further reading

- [Installation guides](https://api.projectchrono.org/install_guides.html) (all modules).
- [Install PyChrono](https://api.projectchrono.org/pychrono_installation.html) (conda vs build from source).
- [GPU accelerator support](https://api.projectchrono.org/tutorial_install_chrono.html#gpu) in the core Chrono install chapter.
