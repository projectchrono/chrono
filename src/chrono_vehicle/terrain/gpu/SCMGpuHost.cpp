// SCMGpuHost.cpp — HIP host bridge: pinned staging, async copy/compute streams, body reduce.

#include "chrono_vehicle/terrain/SCMGpu.h"

#include <hip/hip_runtime.h>

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
                                         hipStream_t stream);

extern "C" int scm_launch_reduce_body_forces(const void* in_dev,
                                             const void* out_dev,
                                             void* body_forces_dev,
                                             int n,
                                             int n_bodies,
                                             hipStream_t stream);

struct BufferSlot {
    HitInput* h_in = nullptr;
    HitOutput* h_out = nullptr;
    HitInput* d_in = nullptr;
    HitOutput* d_out = nullptr;
};

struct ScmGpuContextImpl {
    int device = 0;
    hipStream_t stream_copy = nullptr;
    hipStream_t stream_compute = nullptr;
    hipEvent_t event_h2d_done = nullptr;
    hipEvent_t event_compute_done = nullptr;

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

void die_hip(const char* msg, hipError_t err) {
    fprintf(stderr, "SCM GPU FATAL: %s — %s\n", msg, hipGetErrorString(err));
    std::abort();
}

void ensure_device(int device) {
    int current = -1;
    hipGetDevice(&current);
    if (current != device)
        hipSetDevice(device);
}

void free_slot(BufferSlot& slot) {
    if (slot.d_in)
        (void)hipFree(slot.d_in);
    if (slot.d_out)
        (void)hipFree(slot.d_out);
    if (slot.h_in)
        (void)hipHostFree(slot.h_in);
    if (slot.h_out)
        (void)hipHostFree(slot.h_out);
    slot = {};
}

void ensure_hit_capacity(ScmGpuContextImpl* ctx, std::size_t n) {
    if (n <= ctx->hit_capacity)
        return;

    free_slot(ctx->slot);

    const std::size_t bytes_in = n * sizeof(HitInput);
    const std::size_t bytes_out = n * sizeof(HitOutput);

    hipError_t e1 = hipMalloc(&ctx->slot.d_in, bytes_in);
    if (e1 != hipSuccess)
        die_hip("hipMalloc d_in", e1);
    hipError_t e2 = hipMalloc(&ctx->slot.d_out, bytes_out);
    if (e2 != hipSuccess)
        die_hip("hipMalloc d_out", e2);
    hipError_t e3 = hipHostMalloc(&ctx->slot.h_in, bytes_in);
    if (e3 != hipSuccess)
        die_hip("hipHostMalloc h_in", e3);
    hipError_t e4 = hipHostMalloc(&ctx->slot.h_out, bytes_out);
    if (e4 != hipSuccess)
        die_hip("hipHostMalloc h_out", e4);

    ctx->hit_capacity = n;
}

void ensure_body_capacity(ScmGpuContextImpl* ctx, std::size_t n_bodies) {
    if (n_bodies <= ctx->body_capacity)
        return;

    if (ctx->d_body)
        (void)hipFree(ctx->d_body);
    if (ctx->h_body)
        (void)hipHostFree(ctx->h_body);

    const std::size_t bytes = n_bodies * 6 * sizeof(double);
    hipError_t e1 = hipMalloc(&ctx->d_body, bytes);
    if (e1 != hipSuccess)
        die_hip("hipMalloc d_body", e1);
    hipError_t e2 = hipHostMalloc(reinterpret_cast<void**>(&ctx->h_body), bytes);
    if (e2 != hipSuccess)
        die_hip("hipHostMalloc h_body", e2);

    ctx->body_capacity = n_bodies;
}

BufferSlot& current_slot(ScmGpuContextImpl* impl) {
    return impl->slot;
}

void sync_impl(ScmGpuContextImpl* impl) {
    if (!impl->in_flight)
        return;
    hipError_t e1 = hipStreamSynchronize(impl->stream_copy);
    if (e1 != hipSuccess)
        die_hip("hipStreamSynchronize copy", e1);
    hipError_t e2 = hipStreamSynchronize(impl->stream_compute);
    if (e2 != hipSuccess)
        die_hip("hipStreamSynchronize compute", e2);
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

    hipError_t e_h2d =
        hipMemcpyAsync(slot.d_in, slot.h_in, bytes_in, hipMemcpyHostToDevice, impl->stream_copy);
    if (e_h2d != hipSuccess)
        return static_cast<int>(e_h2d);

    hipError_t e_rec_h2d = hipEventRecord(impl->event_h2d_done, impl->stream_copy);
    if (e_rec_h2d != hipSuccess)
        return static_cast<int>(e_rec_h2d);

    hipError_t e_wait_h2d = hipStreamWaitEvent(impl->stream_compute, impl->event_h2d_done, 0);
    if (e_wait_h2d != hipSuccess)
        return static_cast<int>(e_wait_h2d);

    const int launch_err = scm_launch_compute_forces(&soil,
                                                     slot.d_in,
                                                     slot.d_out,
                                                     static_cast<int>(n_hits),
                                                     impl->stream_compute);
    if (launch_err != hipSuccess)
        return launch_err;

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        hipError_t e_zero =
            hipMemsetAsync(impl->d_body, 0, body_bytes, impl->stream_compute);
        if (e_zero != hipSuccess)
            return static_cast<int>(e_zero);

        const int reduce_err = scm_launch_reduce_body_forces(slot.d_in,
                                                             slot.d_out,
                                                             impl->d_body,
                                                             static_cast<int>(n_hits),
                                                             static_cast<int>(n_bodies),
                                                             impl->stream_compute);
        if (reduce_err != hipSuccess)
            return reduce_err;
    }

    hipError_t e_rec_compute = hipEventRecord(impl->event_compute_done, impl->stream_compute);
    if (e_rec_compute != hipSuccess)
        return static_cast<int>(e_rec_compute);

    hipError_t e_wait_compute = hipStreamWaitEvent(impl->stream_copy, impl->event_compute_done, 0);
    if (e_wait_compute != hipSuccess)
        return static_cast<int>(e_wait_compute);

    hipError_t e_d2h_out =
        hipMemcpyAsync(slot.h_out, slot.d_out, bytes_out, hipMemcpyDeviceToHost, impl->stream_copy);
    if (e_d2h_out != hipSuccess)
        return static_cast<int>(e_d2h_out);

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        hipError_t e_d2h_body = hipMemcpyAsync(impl->h_body,
                                               impl->d_body,
                                               body_bytes,
                                               hipMemcpyDeviceToHost,
                                               impl->stream_copy);
        if (e_d2h_body != hipSuccess)
            return static_cast<int>(e_d2h_body);
    }

    impl->in_flight = true;
    sync_impl(impl);
    return static_cast<int>(hipSuccess);
}

int launch_simple(ScmGpuContextImpl* impl, const SoilParams& soil, std::size_t n_hits, std::size_t n_bodies) {
    BufferSlot& slot = current_slot(impl);
    const std::size_t bytes_in = n_hits * sizeof(HitInput);
    const std::size_t bytes_out = n_hits * sizeof(HitOutput);
    const bool reduce_bodies = n_bodies > 0;

    if (reduce_bodies)
        ensure_body_capacity(impl, n_bodies);

    hipError_t e1 = hipMemcpy(slot.d_in, slot.h_in, bytes_in, hipMemcpyHostToDevice);
    if (e1 != hipSuccess)
        return static_cast<int>(e1);

    const int launch_err = scm_launch_compute_forces(&soil,
                                                     slot.d_in,
                                                     slot.d_out,
                                                     static_cast<int>(n_hits),
                                                     impl->stream_compute);
    if (launch_err != hipSuccess)
        return launch_err;

    if (reduce_bodies) {
        const std::size_t body_bytes = n_bodies * 6 * sizeof(double);
        hipError_t e_zero = hipMemset(impl->d_body, 0, body_bytes);
        if (e_zero != hipSuccess)
            return static_cast<int>(e_zero);

        const int reduce_err = scm_launch_reduce_body_forces(slot.d_in,
                                                             slot.d_out,
                                                             impl->d_body,
                                                             static_cast<int>(n_hits),
                                                             static_cast<int>(n_bodies),
                                                             impl->stream_compute);
        if (reduce_err != hipSuccess)
            return reduce_err;

        hipError_t e_body = hipMemcpy(impl->h_body, impl->d_body, body_bytes, hipMemcpyDeviceToHost);
        if (e_body != hipSuccess)
            return static_cast<int>(e_body);
    }

    hipError_t e2 = hipMemcpy(slot.h_out, slot.d_out, bytes_out, hipMemcpyDeviceToHost);
    if (e2 != hipSuccess)
        return static_cast<int>(e2);

    return static_cast<int>(hipSuccess);
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

    hipError_t e1 = hipStreamCreateWithFlags(&impl->stream_copy, hipStreamNonBlocking);
    if (e1 != hipSuccess)
        die_hip("hipStreamCreate copy", e1);
    hipError_t e2 = hipStreamCreateWithFlags(&impl->stream_compute, hipStreamNonBlocking);
    if (e2 != hipSuccess)
        die_hip("hipStreamCreate compute", e2);
    hipError_t e3 = hipEventCreateWithFlags(&impl->event_h2d_done, hipEventDisableTiming);
    if (e3 != hipSuccess)
        die_hip("hipEventCreate h2d", e3);
    hipError_t e4 = hipEventCreateWithFlags(&impl->event_compute_done, hipEventDisableTiming);
    if (e4 != hipSuccess)
        die_hip("hipEventCreate compute", e4);

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
        hipFree(impl->d_body);
    if (impl->h_body)
        hipHostFree(impl->h_body);
    if (impl->event_h2d_done)
        hipEventDestroy(impl->event_h2d_done);
    if (impl->event_compute_done)
        hipEventDestroy(impl->event_compute_done);
    if (impl->stream_copy)
        hipStreamDestroy(impl->stream_copy);
    if (impl->stream_compute)
        hipStreamDestroy(impl->stream_compute);
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
    if (err != hipSuccess)
        return err;
    if (out_host != slot.h_out)
        std::memcpy(out_host, slot.h_out, bytes_out);
    return 0;
}
