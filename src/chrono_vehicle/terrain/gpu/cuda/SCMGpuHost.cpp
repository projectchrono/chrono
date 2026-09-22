// SCMGpuHost.cpp — CUDA host bridge: pinned staging, async copy/compute streams, body reduce.
// NOTE: this file is DUPLICATED per GPU backend. Its counterpart is
// terrain/gpu/hip/SCMGpuHost.cpp, and the two differ only in the runtime API names
// (about thirty symbols). Any change here must be made there too -- nothing enforces it,
// because only one of the two is compiled in a given build. Chrono has no compatibility
// layer for this and deliberately does not grow one; the kernels, which CAN be shared,
// are single-source .cu files instead.
//
// Not a name-for-name mapping: hipHostMalloc(p, n) is cudaHostAlloc(p, n, flags).

#include "chrono_vehicle/terrain/SCMGpu.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

using chrono::vehicle::scm::gpu::BodyForceAccum;
using chrono::vehicle::scm::gpu::HitInput;
using chrono::vehicle::scm::gpu::HitOutput;
using chrono::vehicle::scm::gpu::SoilParams;

extern "C" int scm_launch_compute_forces(const void* soil_host,
                                         const void* in_dev,
                                         void* out_dev,
                                         int n,
                                         cudaStream_t stream);

extern "C" int scm_launch_reduce_body_forces(const void* in_dev,
                                             const void* out_dev,
                                             void* body_forces_dev,
                                             int n,
                                             int n_bodies,
                                             cudaStream_t stream);

struct BufferSlot {
    HitInput* h_in = nullptr;
    HitOutput* h_out = nullptr;
    HitInput* d_in = nullptr;
    HitOutput* d_out = nullptr;
};

struct ScmGpuContextImpl {
    int device = 0;
    cudaStream_t stream_copy = nullptr;
    cudaStream_t stream_compute = nullptr;
    cudaEvent_t event_h2d_done = nullptr;
    cudaEvent_t event_compute_done = nullptr;

    BufferSlot slot;
    bool in_flight = false;

    BodyForceAccum* h_body = nullptr;
    double* d_body = nullptr;
    std::size_t hit_capacity = 0;
    std::size_t body_capacity = 0;
    bool warmed_up = false;
};

chrono::vehicle::scm_gpu::Config& MutableConfig() {
    static chrono::vehicle::scm_gpu::Config cfg;
    return cfg;
}

void die_cuda(const char* msg, cudaError_t err) {
    fprintf(stderr, "SCM GPU FATAL: %s — %s\n", msg, cudaGetErrorString(err));
    std::abort();
}

void ensure_device(int device) {
    int current = -1;
    cudaGetDevice(&current);
    if (current != device)
        cudaSetDevice(device);
}

void free_slot(BufferSlot& slot) {
    if (slot.d_in)
        (void)cudaFree(slot.d_in);
    if (slot.d_out)
        (void)cudaFree(slot.d_out);
    if (slot.h_in)
        (void)cudaFreeHost(slot.h_in);
    if (slot.h_out)
        (void)cudaFreeHost(slot.h_out);
    slot = {};
}

void ensure_hit_capacity(ScmGpuContextImpl* ctx, std::size_t n) {
    if (n <= ctx->hit_capacity)
        return;

    free_slot(ctx->slot);

    const std::size_t bytes_in = n * sizeof(HitInput);
    const std::size_t bytes_out = n * sizeof(HitOutput);

    cudaError_t e1 = cudaMalloc(&ctx->slot.d_in, bytes_in);
    if (e1 != cudaSuccess)
        die_cuda("cudaMalloc d_in", e1);
    cudaError_t e2 = cudaMalloc(&ctx->slot.d_out, bytes_out);
    if (e2 != cudaSuccess)
        die_cuda("cudaMalloc d_out", e2);
    cudaError_t e3 = cudaHostAlloc(&ctx->slot.h_in, bytes_in, cudaHostAllocDefault);
    if (e3 != cudaSuccess)
        die_cuda("cudaHostAlloc h_in", e3);
    cudaError_t e4 = cudaHostAlloc(&ctx->slot.h_out, bytes_out, cudaHostAllocDefault);
    if (e4 != cudaSuccess)
        die_cuda("cudaHostAlloc h_out", e4);

    ctx->hit_capacity = n;
}

void ensure_body_capacity(ScmGpuContextImpl* ctx, std::size_t n_bodies) {
    if (n_bodies <= ctx->body_capacity)
        return;

    if (ctx->d_body)
        (void)cudaFree(ctx->d_body);
    if (ctx->h_body)
        (void)cudaFreeHost(ctx->h_body);

    const std::size_t bytes = n_bodies * 6 * sizeof(double);
    cudaError_t e1 = cudaMalloc(&ctx->d_body, bytes);
    if (e1 != cudaSuccess)
        die_cuda("cudaMalloc d_body", e1);
    cudaError_t e2 = cudaHostAlloc(reinterpret_cast<void**>(&ctx->h_body), bytes, cudaHostAllocDefault);
    if (e2 != cudaSuccess)
        die_cuda("cudaHostAlloc h_body", e2);

    ctx->body_capacity = n_bodies;
}

BufferSlot& current_slot(ScmGpuContextImpl* impl) {
    return impl->slot;
}

void sync_impl(ScmGpuContextImpl* impl) {
    if (!impl->in_flight)
        return;
    cudaError_t e1 = cudaStreamSynchronize(impl->stream_copy);
    if (e1 != cudaSuccess)
        die_cuda("cudaStreamSynchronize copy", e1);
    cudaError_t e2 = cudaStreamSynchronize(impl->stream_compute);
    if (e2 != cudaSuccess)
        die_cuda("cudaStreamSynchronize compute", e2);
    impl->in_flight = false;
}

int launch_pipelined(ScmGpuContextImpl* impl,
                     const SoilParams& soil,
                     std::size_t n_hits,
                     std::size_t n_bodies) {
    BufferSlot& slot = current_slot(impl);
    const std::size_t bytes_in = n_hits * sizeof(HitInput);
    const std::size_t bytes_out = n_hits * sizeof(HitOutput);
    const bool reduce_bodies = n_bodies > 0;

    if (reduce_bodies)
        ensure_body_capacity(impl, n_bodies);

    sync_impl(impl);

    cudaError_t e_h2d =
        cudaMemcpyAsync(slot.d_in, slot.h_in, bytes_in, cudaMemcpyHostToDevice, impl->stream_copy);
    if (e_h2d != cudaSuccess)
        return static_cast<int>(e_h2d);

    cudaError_t e_rec_h2d = cudaEventRecord(impl->event_h2d_done, impl->stream_copy);
    if (e_rec_h2d != cudaSuccess)
        return static_cast<int>(e_rec_h2d);

    cudaError_t e_wait_h2d = cudaStreamWaitEvent(impl->stream_compute, impl->event_h2d_done, 0);
    if (e_wait_h2d != cudaSuccess)
        return static_cast<int>(e_wait_h2d);

    const int launch_err = scm_launch_compute_forces(&soil,
                                                     slot.d_in,
                                                     slot.d_out,
                                                     static_cast<int>(n_hits),
                                                     impl->stream_compute);
    if (launch_err != cudaSuccess)
        return launch_err;

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        cudaError_t e_zero =
            cudaMemsetAsync(impl->d_body, 0, body_bytes, impl->stream_compute);
        if (e_zero != cudaSuccess)
            return static_cast<int>(e_zero);

        const int reduce_err = scm_launch_reduce_body_forces(slot.d_in,
                                                             slot.d_out,
                                                             impl->d_body,
                                                             static_cast<int>(n_hits),
                                                             static_cast<int>(n_bodies),
                                                             impl->stream_compute);
        if (reduce_err != cudaSuccess)
            return reduce_err;
    }

    cudaError_t e_rec_compute = cudaEventRecord(impl->event_compute_done, impl->stream_compute);
    if (e_rec_compute != cudaSuccess)
        return static_cast<int>(e_rec_compute);

    cudaError_t e_wait_compute = cudaStreamWaitEvent(impl->stream_copy, impl->event_compute_done, 0);
    if (e_wait_compute != cudaSuccess)
        return static_cast<int>(e_wait_compute);

    cudaError_t e_d2h_out =
        cudaMemcpyAsync(slot.h_out, slot.d_out, bytes_out, cudaMemcpyDeviceToHost, impl->stream_copy);
    if (e_d2h_out != cudaSuccess)
        return static_cast<int>(e_d2h_out);

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        cudaError_t e_d2h_body = cudaMemcpyAsync(impl->h_body,
                                               impl->d_body,
                                               body_bytes,
                                               cudaMemcpyDeviceToHost,
                                               impl->stream_copy);
        if (e_d2h_body != cudaSuccess)
            return static_cast<int>(e_d2h_body);
    }

    impl->in_flight = true;
    sync_impl(impl);
    return static_cast<int>(cudaSuccess);
}

int launch_simple(ScmGpuContextImpl* impl, const SoilParams& soil, std::size_t n_hits, std::size_t n_bodies) {
    BufferSlot& slot = current_slot(impl);
    const std::size_t bytes_in = n_hits * sizeof(HitInput);
    const std::size_t bytes_out = n_hits * sizeof(HitOutput);
    const bool reduce_bodies = n_bodies > 0;

    if (reduce_bodies)
        ensure_body_capacity(impl, n_bodies);

    cudaError_t e1 = cudaMemcpy(slot.d_in, slot.h_in, bytes_in, cudaMemcpyHostToDevice);
    if (e1 != cudaSuccess)
        return static_cast<int>(e1);

    const int launch_err = scm_launch_compute_forces(&soil,
                                                     slot.d_in,
                                                     slot.d_out,
                                                     static_cast<int>(n_hits),
                                                     impl->stream_compute);
    if (launch_err != cudaSuccess)
        return launch_err;

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        cudaError_t e_zero = cudaMemset(impl->d_body, 0, body_bytes);
        if (e_zero != cudaSuccess)
            return static_cast<int>(e_zero);

        const int reduce_err = scm_launch_reduce_body_forces(slot.d_in,
                                                             slot.d_out,
                                                             impl->d_body,
                                                             static_cast<int>(n_hits),
                                                             static_cast<int>(n_bodies),
                                                             impl->stream_compute);
        if (reduce_err != cudaSuccess)
            return reduce_err;

        cudaError_t e_body = cudaMemcpy(impl->h_body, impl->d_body, body_bytes, cudaMemcpyDeviceToHost);
        if (e_body != cudaSuccess)
            return static_cast<int>(e_body);
    }

    cudaError_t e2 = cudaMemcpy(slot.h_out, slot.d_out, bytes_out, cudaMemcpyDeviceToHost);
    if (e2 != cudaSuccess)
        return static_cast<int>(e2);

    return static_cast<int>(cudaSuccess);
}

}  // namespace

namespace chrono {
namespace vehicle {
namespace scm_gpu {

void SetConfig(const Config& config) {
    MutableConfig() = config;
}

Config GetConfig() {
    return MutableConfig();
}

}  // namespace scm_gpu
}  // namespace vehicle
}  // namespace chrono

extern "C" std::size_t scm_gpu_min_hits(void) {
    return chrono::vehicle::scm_gpu::GetConfig().min_hits;
}

extern "C" std::size_t scm_gpu_reserve_hits(void) {
    return chrono::vehicle::scm_gpu::GetConfig().reserve_hits;
}

extern "C" int scm_gpu_async_enabled(void) {
    return chrono::vehicle::scm_gpu::GetConfig().async ? 1 : 0;
}

extern "C" ScmGpuContext* scm_gpu_create(int device_id) {
    auto* impl = new ScmGpuContextImpl();
    impl->device = device_id;
    ensure_device(device_id);

    cudaError_t e1 = cudaStreamCreateWithFlags(&impl->stream_copy, cudaStreamNonBlocking);
    if (e1 != cudaSuccess)
        die_cuda("cudaStreamCreate copy", e1);
    cudaError_t e2 = cudaStreamCreateWithFlags(&impl->stream_compute, cudaStreamNonBlocking);
    if (e2 != cudaSuccess)
        die_cuda("cudaStreamCreate compute", e2);
    cudaError_t e3 = cudaEventCreateWithFlags(&impl->event_h2d_done, cudaEventDisableTiming);
    if (e3 != cudaSuccess)
        die_cuda("cudaEventCreate h2d", e3);
    cudaError_t e4 = cudaEventCreateWithFlags(&impl->event_compute_done, cudaEventDisableTiming);
    if (e4 != cudaSuccess)
        die_cuda("cudaEventCreate compute", e4);

    return reinterpret_cast<ScmGpuContext*>(impl);
}

extern "C" void scm_gpu_destroy(ScmGpuContext* ctx) {
    if (!ctx)
        return;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    sync_impl(impl);
    free_slot(impl->slot);
    if (impl->d_body)
        cudaFree(impl->d_body);
    if (impl->h_body)
        cudaFreeHost(impl->h_body);
    if (impl->event_h2d_done)
        cudaEventDestroy(impl->event_h2d_done);
    if (impl->event_compute_done)
        cudaEventDestroy(impl->event_compute_done);
    if (impl->stream_copy)
        cudaStreamDestroy(impl->stream_copy);
    if (impl->stream_compute)
        cudaStreamDestroy(impl->stream_compute);
    delete impl;
}

extern "C" void scm_gpu_reserve(ScmGpuContext* ctx, std::size_t n_hits) {
    if (!ctx || n_hits == 0)
        return;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    ensure_hit_capacity(impl, n_hits);
}

extern "C" void scm_gpu_warmup(ScmGpuContext* ctx) {
    if (!ctx)
        return;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    if (impl->warmed_up)
        return;
    ensure_device(impl->device);
    const std::size_t reserve_n = std::max<std::size_t>(scm_gpu_reserve_hits(), 1);
    ensure_hit_capacity(impl, reserve_n);
    ensure_body_capacity(impl, 1);
    current_slot(impl).h_in[0] = {};
    current_slot(impl).h_in[0].active = 1;
    SoilParams soil{};
    soil.elastic_k = 1.0;
    soil.area = 1.0;
    soil.dt = 1e-3;
    impl->warmed_up = true;
    if (scm_gpu_async_enabled())
        (void)launch_pipelined(impl, soil, 1, 1);
    else
        (void)launch_simple(impl, soil, 1, 1);
}

extern "C" HitInput* scm_gpu_prepare_input(ScmGpuContext* ctx, std::size_t n_hits) {
    if (!ctx || n_hits == 0)
        return nullptr;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    ensure_hit_capacity(impl, n_hits);
    return current_slot(impl).h_in;
}

extern "C" HitOutput* scm_gpu_prepare_output(ScmGpuContext* ctx, std::size_t n_hits) {
    if (!ctx || n_hits == 0)
        return nullptr;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    ensure_hit_capacity(impl, n_hits);
    return current_slot(impl).h_out;
}

extern "C" BodyForceAccum* scm_gpu_prepare_body_forces(ScmGpuContext* ctx, std::size_t n_bodies) {
    if (!ctx || n_bodies == 0)
        return nullptr;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    ensure_body_capacity(impl, n_bodies);
    return impl->h_body;
}

extern "C" void scm_gpu_sync(ScmGpuContext* ctx) {
    if (!ctx)
        return;
    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    sync_impl(impl);
}

extern "C" int scm_gpu_compute_forces_staged(ScmGpuContext* ctx,
                                             const SoilParams& soil,
                                             std::size_t n_hits,
                                             std::size_t n_bodies) {
    if (!ctx)
        return -1;
    if (n_hits == 0)
        return 0;

    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    if (!impl->warmed_up)
        scm_gpu_warmup(ctx);
    ensure_hit_capacity(impl, n_hits);

    if (scm_gpu_async_enabled())
        return launch_pipelined(impl, soil, n_hits, n_bodies);
    return launch_simple(impl, soil, n_hits, n_bodies);
}

extern "C" int scm_gpu_compute_forces(ScmGpuContext* ctx,
                                      const SoilParams& soil,
                                      const HitInput* in,
                                      HitOutput* out_host,
                                      std::size_t n_hits,
                                      std::size_t n_bodies) {
    if (!ctx || !in || !out_host)
        return -1;
    if (n_hits == 0)
        return 0;

    auto* impl = reinterpret_cast<ScmGpuContextImpl*>(ctx);
    ensure_device(impl->device);
    if (!impl->warmed_up)
        scm_gpu_warmup(ctx);
    ensure_hit_capacity(impl, n_hits);

    BufferSlot& slot = current_slot(impl);
    const std::size_t bytes_in = n_hits * sizeof(HitInput);
    const std::size_t bytes_out = n_hits * sizeof(HitOutput);
    if (in != slot.h_in)
        std::memcpy(slot.h_in, in, bytes_in);

    const int err = scm_gpu_compute_forces_staged(ctx, soil, n_hits, n_bodies);
    if (err != cudaSuccess)
        return err;
    if (out_host != slot.h_out)
        std::memcpy(out_host, slot.h_out, bytes_out);
    return 0;
}
