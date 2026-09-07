# FSI-SPH on AMD Instinct — ROCm / HIP tuning guide

Plain-Markdown companion to the Doxygen page `doxygen/documentation/manuals/fsi/fsi_sph_amd_instinct_tuning.md`.

## Expected improvement (typical ranges)

Approximate wall-clock reduction vs default demo settings. Profile on your hardware.

**Per knob**

| Knob | Range |
|------|------:|
| Skip particle output | 5–15% |
| `--ps_freq 2` | 5–15% |
| Output off + `ps_freq 2` (fluid) | 10–25% |
| `HSA_ENABLE_SDMA=0` (Kernel) | 0–40% |
| `GPU_MAX_HW_QUEUES=4` | 0–5% |
| `-ffast-math` rebuild | 2–8% |
| fast-math + inline rebuild | 5–12% |
| Combined runtime + compile + CLI | 8–20% |

**Per demo (recommended recipe)**

| Demo | Range |
|------|------:|
| DamBreak | 10–25% |
| Kernel | 0–40% |
| ObjectDrop | 3–10% |
| WaveTank | 0–8% |
| BaffleFlow | 10–20% |
| AngleRepose | 5–15% |
| RassorDrum | 0–2% |

**Unified profile:** 5–15% fluid demos; 0–2% RassorDrum.

## Why these changes can be faster

| Change | Mechanism |
|--------|-----------|
| **Disable particle output** | Removes GPU→host copies and CSV writes that stall the pipeline. |
| **`ps_freq=2`** | Lowers O(N) neighbor-search rebuild cadence; forces still integrate each step. |
| **`HSA_ENABLE_SDMA=0`** | Avoids SDMA setup on tiny transfers; better for micro-kernels. |
| **`GPU_MAX_HW_QUEUES=4`** | More in-flight queues → latency hiding on long runs. |
| **`-ffast-math` / inline-all** | Faster force loops and fewer kernel launches. |
| **RassorDrum** | HBM-bound — expect **0–2%** from tuning alone. |

## Quick start

```bash
export ROCM_PATH=${ROCM_PATH:-/opt/rocm}

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DEMOS=ON \
  -DENABLE_MODULE_FSI=ON \
  -DCHRONO_HIP=ON \
  -DCMAKE_HIP_ARCHITECTURES=gfx942   # MI300X; use gfx950 for MI350X

cmake --build build --target demo_FSI-SPH_DamBreak demo_FSI-SPH_Kernels \
  demo_FSI-SPH_ObjectDrop demo_FSI-SPH_WaveTank demo_FSI-SPH_BaffleFlow \
  demo_FSI-SPH_AngleRepose demo_FSI-SPH_RassorDrum -j $(nproc)

cd build
export LD_LIBRARY_PATH=$PWD/lib:${ROCM_PATH}/lib:${ROCM_PATH}/lib64:${LD_LIBRARY_PATH}
export HSA_XNACK=0 HIP_LAUNCH_BLOCKING=0
```

## Run examples

| Demo | Command |
|------|---------|
| DamBreak | `./bin/demo_FSI-SPH_DamBreak --no_vis --quiet --output false --ps_freq 2` |
| Kernel | `HSA_ENABLE_SDMA=0 ./bin/demo_FSI-SPH_Kernels` |
| ObjectDrop | `./bin/demo_FSI-SPH_ObjectDrop --no_vis --quiet --output false` |
| WaveTank | `./bin/demo_FSI-SPH_WaveTank --no_vis --quiet --output_particle_data false` |
| BaffleFlow | `./bin/demo_FSI-SPH_BaffleFlow --no_vis --quiet --output false --ps_freq 2` |
| AngleRepose | `./bin/demo_FSI-SPH_AngleRepose --no_vis --output false --ps_freq 2` |
| RassorDrum | `./bin/demo_FSI-SPH_RassorDrum --no_vis` |

Run from the **build directory** so `../data/` resolves.

Full catalog: Doxygen page `manual_fsi_sph_amd_instinct_tuning`.

## Tuning env snippets

```bash
source docs/fsi_sph_tuning/runtime_sdma_off.env.example
./bin/demo_FSI-SPH_Kernels
```

See `docs/fsi_sph_tuning/*.env.example`.
