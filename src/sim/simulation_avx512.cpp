#include "sim/simulation.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "app/app_config.h"
#include "ui/ui.h"

#if defined(_MSC_VER)
    #include <intrin.h>
#elif defined(__GNUC__) || defined(__clang__)
    #include <cpuid.h>
#endif

#ifdef BOIDS_COMPILE_AVX512
    #include <immintrin.h>
#endif

namespace {

bool boids_cpu_supports_avx512_impl() {
#ifndef BOIDS_COMPILE_AVX512
    return false;
#else
    #if defined(_MSC_VER)
    int cpu_info[4] = {};
    __cpuid(cpu_info, 1);

    const bool has_osxsave = (cpu_info[2] & (1 << 27)) != 0;
    const bool has_avx = (cpu_info[2] & (1 << 28)) != 0;
    if (!has_osxsave || !has_avx) {
        return false;
    }

    const unsigned long long xcr0 = _xgetbv(0);
    const unsigned long long required_xcr0 = (1ull << 1) | (1ull << 2) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    if ((xcr0 & required_xcr0) != required_xcr0) {
        return false;
    }

    __cpuidex(cpu_info, 7, 0);
    return (cpu_info[1] & (1 << 16)) != 0;
    #elif defined(__GNUC__) || defined(__clang__)
    if (!__get_cpuid_max(0, nullptr)) {
        return false;
    }

    unsigned int eax = 0;
    unsigned int ebx = 0;
    unsigned int ecx = 0;
    unsigned int edx = 0;

    __cpuid(1, eax, ebx, ecx, edx);
    const bool has_osxsave = (ecx & bit_OSXSAVE) != 0;
    const bool has_avx = (ecx & bit_AVX) != 0;
    if (!has_osxsave || !has_avx) {
        return false;
    }

    unsigned int xcr0_lo = 0;
    unsigned int xcr0_hi = 0;
    __asm__ volatile("xgetbv" : "=a"(xcr0_lo), "=d"(xcr0_hi) : "c"(0));
    const unsigned long long xcr0 = (static_cast<unsigned long long>(xcr0_hi) << 32) | xcr0_lo;
    const unsigned long long required_xcr0 = (1ull << 1) | (1ull << 2) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    if ((xcr0 & required_xcr0) != required_xcr0) {
        return false;
    }

    __cpuid_count(7, 0, eax, ebx, ecx, edx);
    return (ebx & bit_AVX512F) != 0;
    #else
    return false;
    #endif
#endif
}

#ifdef BOIDS_COMPILE_AVX512

inline __mmask16 boids_tail_mask(const int count) {
    if (count >= 16) {
        return 0xFFFFu;
    }

    return static_cast<__mmask16>((1u << count) - 1u);
}

inline void update_cell2_impl(const int x, const int y, const Rules* rules, const BoidList* boid_list) {
    const auto world_height = boid_map->m_cell_size * boid_map->m_ysize;
    const auto world_width = boid_map->m_cell_size * boid_map->m_xsize;

    const Boid cell_begin = boid_map->m_boid_map[y * boid_map->m_xsize + x];
    if (cell_begin == -1) {
        return;
    }

    const BoidStore* src_store = boid_list->m_boid_store;
    BoidStore* dst_store = boid_list->m_backbuffer;

    const int32_t* depth = src_store->depth;
    const Boid cell_end = cell_begin + depth[cell_begin];

    const float* xs = src_store->xs;
    const float* ys = src_store->ys;
    const float* vxs = src_store->vxs;
    const float* vys = src_store->vys;
    const int32_t* homes = src_store->homes;

    const __m512 ads_vec = _mm512_set1_ps(rules->avoid_distance_squared);
    const __m512 srs_vec = _mm512_set1_ps(rules->sight_range_squared);
    const __m512 af_vec = _mm512_set1_ps(rules->avoid_factor);
    const __m512 alignment_factor = _mm512_set1_ps(rules->alignment_factor);
    const __m512 cohesion_factor = _mm512_set1_ps(rules->cohesion_factor);
    const __m512 rf = _mm512_set1_ps(rules->edge_factor);
    const __m512 ew = _mm512_set1_ps(static_cast<float>(rules->edge_width));
    const __m512 one = _mm512_set1_ps(1.0f);
    const __m512 two = _mm512_set1_ps(2.0f);
    const __m512 zero = _mm512_setzero_ps();
    const __m512 max_float = _mm512_set1_ps(std::numeric_limits<float>::max());
    const __m512 minspeed = _mm512_set1_ps(3.0f);
    const __m512 maxspeed = _mm512_set1_ps(4.0f);

    Boid near_row_begin[3] = { -1, -1, -1 };
    Boid near_row_end[3] = { -1, -1, -1 };

    for (int cy = -1; cy <= 1; cy++) {
        Boid row_begin = -1;
        Boid row_end = -1;

        for (int cx = -1; cx <= 1; cx++) {
            const Boid current = boid_map->get_coord(y + cy, x + cx);
            if (current != -1) {
                if (row_begin == -1) {
                    row_begin = current;
                }
                row_end = current + depth[current];
            }
        }

        near_row_begin[cy + 1] = row_begin;
        near_row_end[cy + 1] = row_end;
    }

    const __m512i rotate_idx = _mm512_set_epi32(0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
    const __m512i home_shift = _mm512_set1_epi32(4);
    const __m512i home_mask = _mm512_set1_epi32(15);
    const __m512i home_bias = _mm512_set1_epi32(1);

    for (Boid current_boid = cell_begin; current_boid < cell_end; current_boid += 16) {
        const int current_count = static_cast<int>(cell_end - current_boid);
        const bool full_current = current_count >= 16;
        const __mmask16 current_mask = boids_tail_mask(current_count);

        const __m512 current_xs_vec = full_current
            ? _mm512_loadu_ps(&xs[current_boid])
            : _mm512_maskz_loadu_ps(current_mask, &xs[current_boid]);
        const __m512 current_ys_vec = full_current
            ? _mm512_loadu_ps(&ys[current_boid])
            : _mm512_maskz_loadu_ps(current_mask, &ys[current_boid]);

        __m512 sep_x_vec = zero;
        __m512 sep_y_vec = zero;
        __m512 avg_x_vec = zero;
        __m512 avg_y_vec = zero;
        __m512 avg_vx_vec = zero;
        __m512 avg_vy_vec = zero;
        __m512i isc = _mm512_setzero_si512();

        for (int row_i = 0; row_i < 3; row_i++) {
            const Boid row_begin = near_row_begin[row_i];
            const Boid row_end = near_row_end[row_i];

            for (Boid nearby_boid = row_begin; nearby_boid < row_end; nearby_boid += 16) {
                const int nearby_count = static_cast<int>(row_end - nearby_boid);
                const bool full_nearby = nearby_count >= 16;
                const __mmask16 nearby_mask = boids_tail_mask(nearby_count);

                __m512 nearby_xs_vec = full_nearby
                    ? _mm512_loadu_ps(&xs[nearby_boid])
                    : _mm512_maskz_loadu_ps(nearby_mask, &xs[nearby_boid]);
                __m512 nearby_ys_vec = full_nearby
                    ? _mm512_loadu_ps(&ys[nearby_boid])
                    : _mm512_maskz_loadu_ps(nearby_mask, &ys[nearby_boid]);
                __m512 nearby_vxs_vec = full_nearby
                    ? _mm512_loadu_ps(&vxs[nearby_boid])
                    : _mm512_maskz_loadu_ps(nearby_mask, &vxs[nearby_boid]);
                __m512 nearby_vys_vec = full_nearby
                    ? _mm512_loadu_ps(&vys[nearby_boid])
                    : _mm512_maskz_loadu_ps(nearby_mask, &vys[nearby_boid]);

                if (full_current && full_nearby) {
                    for (int i = 0; i < 16; i++) {
                        const __m512 xs_delta = _mm512_sub_ps(current_xs_vec, nearby_xs_vec);
                        const __m512 ys_delta = _mm512_sub_ps(current_ys_vec, nearby_ys_vec);
                        const __m512 ds_vec = _mm512_fmadd_ps(xs_delta, xs_delta, _mm512_mul_ps(ys_delta, ys_delta));

                        const __mmask16 srs_mask = _mm512_cmp_ps_mask(ds_vec, srs_vec, _CMP_LT_OS);
                        const __mmask16 ads_mask = _mm512_cmp_ps_mask(ds_vec, ads_vec, _CMP_LT_OS);
                        const __mmask16 sna_mask = srs_mask & ~ads_mask;

                        const __m512 ads_take_ds = _mm512_sub_ps(ads_vec, ds_vec);
                        const __m512 ads_take_ds_sqr = _mm512_mul_ps(ads_take_ds, ads_take_ds);

                        sep_x_vec = _mm512_add_ps(sep_x_vec, _mm512_maskz_mul_ps(ads_mask, xs_delta, ads_take_ds_sqr));
                        sep_y_vec = _mm512_add_ps(sep_y_vec, _mm512_maskz_mul_ps(ads_mask, ys_delta, ads_take_ds_sqr));

                        avg_vx_vec = _mm512_add_ps(avg_vx_vec, _mm512_maskz_mov_ps(sna_mask, nearby_vxs_vec));
                        avg_vy_vec = _mm512_add_ps(avg_vy_vec, _mm512_maskz_mov_ps(sna_mask, nearby_vys_vec));
                        avg_x_vec = _mm512_add_ps(avg_x_vec, _mm512_maskz_mov_ps(sna_mask, nearby_xs_vec));
                        avg_y_vec = _mm512_add_ps(avg_y_vec, _mm512_maskz_mov_ps(sna_mask, nearby_ys_vec));
                        isc = _mm512_add_epi32(isc, _mm512_maskz_set1_epi32(sna_mask, 1));

                        nearby_xs_vec = _mm512_permutexvar_ps(rotate_idx, nearby_xs_vec);
                        nearby_ys_vec = _mm512_permutexvar_ps(rotate_idx, nearby_ys_vec);
                        nearby_vxs_vec = _mm512_permutexvar_ps(rotate_idx, nearby_vxs_vec);
                        nearby_vys_vec = _mm512_permutexvar_ps(rotate_idx, nearby_vys_vec);
                    }
                } else if (full_nearby) {
                    for (int i = 0; i < 16; i++) {
                        const __m512 xs_delta = _mm512_sub_ps(current_xs_vec, nearby_xs_vec);
                        const __m512 ys_delta = _mm512_sub_ps(current_ys_vec, nearby_ys_vec);
                        const __m512 ds_vec = _mm512_fmadd_ps(xs_delta, xs_delta, _mm512_mul_ps(ys_delta, ys_delta));

                        const __mmask16 srs_mask = current_mask & _mm512_cmp_ps_mask(ds_vec, srs_vec, _CMP_LT_OS);
                        const __mmask16 ads_mask = current_mask & _mm512_cmp_ps_mask(ds_vec, ads_vec, _CMP_LT_OS);
                        const __mmask16 sna_mask = srs_mask & ~ads_mask;

                        const __m512 ads_take_ds = _mm512_sub_ps(ads_vec, ds_vec);
                        const __m512 ads_take_ds_sqr = _mm512_mul_ps(ads_take_ds, ads_take_ds);

                        sep_x_vec = _mm512_mask_add_ps(sep_x_vec, ads_mask, sep_x_vec, _mm512_mul_ps(xs_delta, ads_take_ds_sqr));
                        sep_y_vec = _mm512_mask_add_ps(sep_y_vec, ads_mask, sep_y_vec, _mm512_mul_ps(ys_delta, ads_take_ds_sqr));

                        avg_vx_vec = _mm512_mask_add_ps(avg_vx_vec, sna_mask, avg_vx_vec, nearby_vxs_vec);
                        avg_vy_vec = _mm512_mask_add_ps(avg_vy_vec, sna_mask, avg_vy_vec, nearby_vys_vec);
                        avg_x_vec = _mm512_mask_add_ps(avg_x_vec, sna_mask, avg_x_vec, nearby_xs_vec);
                        avg_y_vec = _mm512_mask_add_ps(avg_y_vec, sna_mask, avg_y_vec, nearby_ys_vec);
                        isc = _mm512_add_epi32(isc, _mm512_maskz_set1_epi32(sna_mask, 1));

                        nearby_xs_vec = _mm512_permutexvar_ps(rotate_idx, nearby_xs_vec);
                        nearby_ys_vec = _mm512_permutexvar_ps(rotate_idx, nearby_ys_vec);
                        nearby_vxs_vec = _mm512_permutexvar_ps(rotate_idx, nearby_vxs_vec);
                        nearby_vys_vec = _mm512_permutexvar_ps(rotate_idx, nearby_vys_vec);
                    }
                } else {
                    alignas(64) float nearby_xs_buf[16];
                    alignas(64) float nearby_ys_buf[16];
                    alignas(64) float nearby_vxs_buf[16];
                    alignas(64) float nearby_vys_buf[16];

                    _mm512_store_ps(nearby_xs_buf, nearby_xs_vec);
                    _mm512_store_ps(nearby_ys_buf, nearby_ys_vec);
                    _mm512_store_ps(nearby_vxs_buf, nearby_vxs_vec);
                    _mm512_store_ps(nearby_vys_buf, nearby_vys_vec);

                    for (int i = 0; i < nearby_count; i++) {
                        const __m512 nearby_x = _mm512_set1_ps(nearby_xs_buf[i]);
                        const __m512 nearby_y = _mm512_set1_ps(nearby_ys_buf[i]);
                        const __m512 nearby_vx = _mm512_set1_ps(nearby_vxs_buf[i]);
                        const __m512 nearby_vy = _mm512_set1_ps(nearby_vys_buf[i]);

                        const __m512 xs_delta = _mm512_sub_ps(current_xs_vec, nearby_x);
                        const __m512 ys_delta = _mm512_sub_ps(current_ys_vec, nearby_y);
                        const __m512 ds_vec = _mm512_fmadd_ps(xs_delta, xs_delta, _mm512_mul_ps(ys_delta, ys_delta));

                        const __mmask16 srs_mask = current_mask & _mm512_cmp_ps_mask(ds_vec, srs_vec, _CMP_LT_OS);
                        const __mmask16 ads_mask = current_mask & _mm512_cmp_ps_mask(ds_vec, ads_vec, _CMP_LT_OS);
                        const __mmask16 sna_mask = srs_mask & ~ads_mask;

                        const __m512 ads_take_ds = _mm512_sub_ps(ads_vec, ds_vec);
                        const __m512 ads_take_ds_sqr = _mm512_mul_ps(ads_take_ds, ads_take_ds);

                        sep_x_vec = _mm512_mask_add_ps(sep_x_vec, ads_mask, sep_x_vec, _mm512_mul_ps(xs_delta, ads_take_ds_sqr));
                        sep_y_vec = _mm512_mask_add_ps(sep_y_vec, ads_mask, sep_y_vec, _mm512_mul_ps(ys_delta, ads_take_ds_sqr));

                        avg_vx_vec = _mm512_mask_add_ps(avg_vx_vec, sna_mask, avg_vx_vec, nearby_vx);
                        avg_vy_vec = _mm512_mask_add_ps(avg_vy_vec, sna_mask, avg_vy_vec, nearby_vy);
                        avg_x_vec = _mm512_mask_add_ps(avg_x_vec, sna_mask, avg_x_vec, nearby_x);
                        avg_y_vec = _mm512_mask_add_ps(avg_y_vec, sna_mask, avg_y_vec, nearby_y);
                        isc = _mm512_add_epi32(isc, _mm512_maskz_set1_epi32(sna_mask, 1));
                    }
                }
            }
        }

        const __mmask16 isc_mask = full_current
            ? _mm512_cmp_epi32_mask(isc, _mm512_setzero_si512(), _MM_CMPINT_GT)
            : current_mask & _mm512_cmp_epi32_mask(isc, _mm512_setzero_si512(), _MM_CMPINT_GT);

        __m512 vxs_out = full_current
            ? _mm512_loadu_ps(&vxs[current_boid])
            : _mm512_maskz_loadu_ps(current_mask, &vxs[current_boid]);
        __m512 vys_out = full_current
            ? _mm512_loadu_ps(&vys[current_boid])
            : _mm512_maskz_loadu_ps(current_mask, &vys[current_boid]);

        vxs_out = _mm512_fmadd_ps(sep_x_vec, af_vec, vxs_out);
        vys_out = _mm512_fmadd_ps(sep_y_vec, af_vec, vys_out);

        const __m512 cvt_isc = _mm512_cvtepi32_ps(isc);

#ifdef APPROXIMATE_NEIGHBOR_AVERAGES
        __m512 inv_isc = _mm512_rcp14_ps(cvt_isc);
        inv_isc = _mm512_mul_ps(inv_isc, _mm512_sub_ps(two, _mm512_mul_ps(cvt_isc, inv_isc)));
        inv_isc = full_current ? _mm512_maskz_mov_ps(isc_mask, inv_isc) : _mm512_mask_mov_ps(zero, isc_mask, inv_isc);

        avg_vx_vec = _mm512_mul_ps(avg_vx_vec, inv_isc);
        avg_vy_vec = _mm512_mul_ps(avg_vy_vec, inv_isc);
#else
        avg_vx_vec = _mm512_div_ps(avg_vx_vec, cvt_isc);
        avg_vy_vec = _mm512_div_ps(avg_vy_vec, cvt_isc);
#endif

        vxs_out = _mm512_fmadd_ps(alignment_factor, _mm512_maskz_sub_ps(isc_mask, avg_vx_vec, vxs_out), vxs_out);
        vys_out = _mm512_fmadd_ps(alignment_factor, _mm512_maskz_sub_ps(isc_mask, avg_vy_vec, vys_out), vys_out);

#ifdef APPROXIMATE_NEIGHBOR_AVERAGES
        avg_x_vec = _mm512_mul_ps(avg_x_vec, inv_isc);
        avg_y_vec = _mm512_mul_ps(avg_y_vec, inv_isc);
#else
        avg_x_vec = _mm512_div_ps(avg_x_vec, cvt_isc);
        avg_y_vec = _mm512_div_ps(avg_y_vec, cvt_isc);
#endif

        vxs_out = _mm512_fmadd_ps(cohesion_factor, _mm512_maskz_sub_ps(isc_mask, avg_x_vec, current_xs_vec), vxs_out);
        vys_out = _mm512_fmadd_ps(cohesion_factor, _mm512_maskz_sub_ps(isc_mask, avg_y_vec, current_ys_vec), vys_out);

        const __mmask16 cmpx_1 = full_current
            ? _mm512_cmp_ps_mask(current_xs_vec, ew, _CMP_LT_OS)
            : current_mask & _mm512_cmp_ps_mask(current_xs_vec, ew, _CMP_LT_OS);
        const __mmask16 cmpx_2 = full_current
            ? _mm512_cmp_ps_mask(current_xs_vec, _mm512_set1_ps(static_cast<float>(world_width - rules->edge_width)), _CMP_GT_OS)
            : current_mask & _mm512_cmp_ps_mask(current_xs_vec, _mm512_set1_ps(static_cast<float>(world_width - rules->edge_width)), _CMP_GT_OS);
        const __mmask16 cmpy_1 = full_current
            ? _mm512_cmp_ps_mask(current_ys_vec, ew, _CMP_LT_OS)
            : current_mask & _mm512_cmp_ps_mask(current_ys_vec, ew, _CMP_LT_OS);
        const __mmask16 cmpy_2 = full_current
            ? _mm512_cmp_ps_mask(current_ys_vec, _mm512_set1_ps(static_cast<float>(world_height - rules->edge_width)), _CMP_GT_OS)
            : current_mask & _mm512_cmp_ps_mask(current_ys_vec, _mm512_set1_ps(static_cast<float>(world_height - rules->edge_width)), _CMP_GT_OS);

        vxs_out = _mm512_add_ps(vxs_out, _mm512_sub_ps(_mm512_maskz_mov_ps(cmpx_1, rf), _mm512_maskz_mov_ps(cmpx_2, rf)));
        vys_out = _mm512_add_ps(vys_out, _mm512_sub_ps(_mm512_maskz_mov_ps(cmpy_1, rf), _mm512_maskz_mov_ps(cmpy_2, rf)));

        const __m512 rspeed_vec = _mm512_rsqrt14_ps(_mm512_fmadd_ps(vxs_out, vxs_out, _mm512_mul_ps(vys_out, vys_out)));
        const __m512 ispeed_vec = _mm512_mul_ps(rspeed_vec, maxspeed);
        const __m512 lspeed_vec = _mm512_mul_ps(rspeed_vec, minspeed);

        const __mmask16 too_slow = full_current
            ? _mm512_cmp_ps_mask(rspeed_vec, _mm512_set1_ps(1.0f / 3.0f), _CMP_GT_OS)
            : current_mask & _mm512_cmp_ps_mask(rspeed_vec, _mm512_set1_ps(1.0f / 3.0f), _CMP_GT_OS);
        const __mmask16 too_fast = full_current
            ? _mm512_cmp_ps_mask(rspeed_vec, _mm512_set1_ps(1.0f / 4.0f), _CMP_LT_OS)
            : current_mask & _mm512_cmp_ps_mask(rspeed_vec, _mm512_set1_ps(1.0f / 4.0f), _CMP_LT_OS);
        const __mmask16 too_inf = full_current
            ? _mm512_cmp_ps_mask(rspeed_vec, max_float, _CMP_GT_OS)
            : current_mask & _mm512_cmp_ps_mask(rspeed_vec, max_float, _CMP_GT_OS);

        __m512 multipliers = one;
        multipliers = _mm512_mask_mov_ps(multipliers, too_slow, lspeed_vec);
        multipliers = _mm512_mask_mov_ps(multipliers, too_fast, ispeed_vec);
        multipliers = _mm512_mask_mov_ps(multipliers, too_inf, one);

        vxs_out = _mm512_mul_ps(multipliers, vxs_out);
        vys_out = _mm512_mul_ps(multipliers, vys_out);

        const __m512i homes_vec = full_current
            ? _mm512_loadu_si512((const void*) &homes[current_boid])
            : _mm512_maskz_loadu_epi32(current_mask, &homes[current_boid]);

        const __m512 home_index_x_vec = _mm512_cvtepi32_ps(_mm512_add_epi32(_mm512_and_si512(homes_vec, home_mask), home_bias));
        const __m512 home_index_y_vec = _mm512_cvtepi32_ps(_mm512_add_epi32(_mm512_srlv_epi32(homes_vec, home_shift), home_bias));

        const __m512 home_loc_x_vec = _mm512_fmadd_ps(home_index_x_vec, _mm512_set1_ps(static_cast<float>((world_width - rules->edge_width * 2) / (16 + 1))), ew);
        const __m512 home_loc_y_vec = _mm512_fmadd_ps(home_index_y_vec, _mm512_set1_ps(static_cast<float>((world_height - rules->edge_width * 2) / (9 + 1))), ew);

        const __m512 dx_vec = _mm512_sub_ps(home_loc_x_vec, current_xs_vec);
        const __m512 dy_vec = _mm512_sub_ps(home_loc_y_vec, current_ys_vec);

        vxs_out = _mm512_fmadd_ps(dx_vec, _mm512_set1_ps(rules->homing), vxs_out);
        vys_out = _mm512_fmadd_ps(dy_vec, _mm512_set1_ps(rules->homing), vys_out);

        const __m512 xs_out = _mm512_add_ps(current_xs_vec, vxs_out);
        const __m512 ys_out = _mm512_add_ps(current_ys_vec, vys_out);

        if (full_current) {
            _mm512_storeu_ps(&dst_store->xs[current_boid], xs_out);
            _mm512_storeu_ps(&dst_store->ys[current_boid], ys_out);
            _mm512_storeu_ps(&dst_store->vxs[current_boid], vxs_out);
            _mm512_storeu_ps(&dst_store->vys[current_boid], vys_out);
            _mm512_storeu_si512((void*) &dst_store->homes[current_boid], homes_vec);
        } else {
            _mm512_mask_storeu_ps(&dst_store->xs[current_boid], current_mask, xs_out);
            _mm512_mask_storeu_ps(&dst_store->ys[current_boid], current_mask, ys_out);
            _mm512_mask_storeu_ps(&dst_store->vxs[current_boid], current_mask, vxs_out);
            _mm512_mask_storeu_ps(&dst_store->vys[current_boid], current_mask, vys_out);
            _mm512_mask_storeu_epi32(&dst_store->homes[current_boid], current_mask, homes_vec);
        }
    }
}

#endif

} // namespace

bool boids_cpu_supports_avx512() {
    static const bool supported = boids_cpu_supports_avx512_impl();
    return supported;
}

bool boids_avx512_compiled() {
#ifdef BOIDS_COMPILE_AVX512
    return true;
#else
    return false;
#endif
}

void update_cell2_avx512(const int x, const int y, const Rules* rules, const BoidList* boid_list) {
#ifdef BOIDS_COMPILE_AVX512
    update_cell2_impl(x, y, rules, boid_list);
#else
    (void) x;
    (void) y;
    (void) rules;
    (void) boid_list;
#endif
}
