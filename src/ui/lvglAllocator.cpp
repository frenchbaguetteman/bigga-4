#include "ui/lvglRuntime.h"

#if defined(_PROS_INCLUDE_LIBLVGL_LLEMU_HPP)

#include "liblvgl/stdlib/lv_mem.h"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstring>

namespace {

struct AllocationHeader {
    std::size_t size;
};

std::atomic<std::size_t> g_liveBytes{0};
std::atomic<std::size_t> g_peakBytes{0};
std::atomic<std::size_t> g_liveAllocs{0};

std::size_t clampRequestedSize(std::size_t size) {
    return std::max<std::size_t>(size, 1U);
}

void updatePeak(std::size_t usedNow) {
    std::size_t observed = g_peakBytes.load(std::memory_order_relaxed);
    while (usedNow > observed &&
           !g_peakBytes.compare_exchange_weak(
               observed,
               usedNow,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }
}

void * allocateTracked(std::size_t requested, bool zeroFill) {
    const std::size_t size = clampRequestedSize(requested);
    auto * raw = static_cast<AllocationHeader *>(
        std::malloc(sizeof(AllocationHeader) + size));
    if (!raw) return nullptr;

    raw->size = size;
    void * payload = raw + 1;
    if (zeroFill) {
        std::memset(payload, 0, size);
    }

    const std::size_t usedNow =
        g_liveBytes.fetch_add(size, std::memory_order_relaxed) + size;
    g_liveAllocs.fetch_add(1, std::memory_order_relaxed);
    updatePeak(usedNow);
    return payload;
}

AllocationHeader * headerFromPayload(void * payload) {
    if (!payload) return nullptr;
    return static_cast<AllocationHeader *>(payload) - 1;
}

const AllocationHeader * headerFromPayload(const void * payload) {
    if (!payload) return nullptr;
    return static_cast<const AllocationHeader *>(payload) - 1;
}

void releaseTracked(void * payload) {
    auto * header = headerFromPayload(payload);
    if (!header) return;

    g_liveBytes.fetch_sub(header->size, std::memory_order_relaxed);
    g_liveAllocs.fetch_sub(1, std::memory_order_relaxed);
    std::free(header);
}

}  // namespace

extern "C" {

void lv_mem_init(void) {}

void lv_mem_deinit(void) {}

lv_mem_pool_t lv_mem_add_pool(void * /*mem*/, size_t /*bytes*/) {
    return nullptr;
}

void lv_mem_remove_pool(lv_mem_pool_t /*pool*/) {}

void * lv_malloc_core(size_t size) {
    return allocateTracked(size, false);
}

void lv_free_core(void * p) {
    releaseTracked(p);
}

void * lv_realloc_core(void * p, size_t new_size) {
    if (!p) return allocateTracked(new_size, false);
    if (new_size == 0) {
        releaseTracked(p);
        return nullptr;
    }

    AllocationHeader * oldHeader = headerFromPayload(p);
    const std::size_t oldSize = oldHeader->size;
    const std::size_t size = clampRequestedSize(new_size);

    auto * newHeader = static_cast<AllocationHeader *>(
        std::realloc(oldHeader, sizeof(AllocationHeader) + size));
    if (!newHeader) return nullptr;

    newHeader->size = size;
    if (size > oldSize) {
        const std::size_t usedNow =
            g_liveBytes.fetch_add(size - oldSize, std::memory_order_relaxed) +
            (size - oldSize);
        updatePeak(usedNow);
    } else if (size < oldSize) {
        g_liveBytes.fetch_sub(oldSize - size, std::memory_order_relaxed);
    }

    return newHeader + 1;
}

void lv_mem_monitor_core(lv_mem_monitor_t * mon_p) {
    if (!mon_p) return;

    std::memset(mon_p, 0, sizeof(*mon_p));
    const std::size_t used = g_liveBytes.load(std::memory_order_relaxed);
    const std::size_t peak = g_peakBytes.load(std::memory_order_relaxed);
    const std::size_t allocs = g_liveAllocs.load(std::memory_order_relaxed);

    mon_p->total_size = peak;
    mon_p->used_cnt = allocs;
    mon_p->max_used = peak;
    mon_p->used_pct = (peak == 0) ? 0 : static_cast<uint8_t>((used * 100U) / peak);
    mon_p->frag_pct = 0;
}

lv_result_t lv_mem_test_core(void) {
    void * probe = lv_malloc_core(32);
    if (!probe) return LV_RESULT_INVALID;
    lv_free_core(probe);
    return LV_RESULT_OK;
}

}  // extern "C"

#endif
