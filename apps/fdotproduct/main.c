// Copyright 2022-2025 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#include <stdint.h>
#include <string.h>

#include "runtime.h"
#include "util.h"

#include "kernel/fdotproduct.h"

#ifndef SPIKE
#include "printf.h"
#else
#include <stdio.h>
#endif

// Threshold for FP comparisons
#define THRESHOLD_64b 0.0000000001
#define THRESHOLD_32b 0.0001
#define THRESHOLD_16b 1

// Run also the scalar benchmark
#define SCALAR 0

// Check the vector results against golden vectors
#define CHECK 1

// Macro to check similarity between two fp-values, wrt a threshold
#define fp_check(a, b, threshold) ((((a - b) < 0) ? b - a : a - b) < threshold)

// Vector size (Byte)
extern uint64_t vsize;
// Input vectors
extern double v64a[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern double v64b[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern float v32a[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern float v32b[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern _Float16 v16a[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern _Float16 v16b[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
// Golden outputs
extern double gold64;
extern float gold32;
extern _Float16 gold16;
// Output vectors
extern double res64_v, res64_s;
extern float res32_v, res32_s;
extern _Float16 res16_v, res16_s;

// Per-core partial results, allocated in the non-cacheable L2 window
// (0xBFFF_F848+, above CachedRegionLength = DRAMLength - 2048), so neither
// CVA6 D$ can hold a stale copy of another hart's partial.
// Only hart 0 allocates: baremetal_atomic_malloc()'s bump pointer is a plain
// non-atomic static, so calling it from every hart would hand out a different
// address to each.
volatile double *res64_part;

int main(int hart_id) {
  if (hart_id == 0)
    res64_part = baremetal_atomic_malloc(NR_CORES * sizeof(double));

  // Publish the pointer before any hart dereferences it.
  sync_barrier();

  if (hart_id == 0) {
    printf("\n");
    printf("===========\n");
    printf("=  FDOTP  =\n");
    printf("===========\n");
    printf("\n");
    printf("\n");
  }

  volatile double *res64_core = &res64_part[hart_id];

  uint64_t runtime_s, runtime_v;

  // `len` is the TOTAL length of this iteration; each hart owns len / NR_CORES.
  // Bound is `vsize` (an element count), not the original `vsize >> 3`, which
  // assumed vsize was a byte count and so never reached the full length.
  for (uint64_t len = 4096; len <= vsize; len *= 2) {
    // Re-slice the vectors for this iteration's length.
    uint64_t avl_i   = len / NR_CORES;
    uint64_t start_i = avl_i * hart_id;
    double *v64a_i   = v64a + start_i;
    double *v64b_i   = v64b + start_i;

    // Serialise the per-hart trace so the lines come out in hart order. This
    // must describe THIS iteration's slice: the partition changes with len.
    for (int h = 0; h < NR_CORES; ++h) {
      sync_barrier();
      if (hart_id == h)
        printf("From Hart:%d len:%lu a:%lx b:%lx res:%lx index:%lu-%lu\n",
               hart_id, len, (uint64_t)v64a_i, (uint64_t)v64b_i,
               (uint64_t)res64_core, start_i, start_i + avl_i - 1);
    }
    sync_barrier();

    if (hart_id == 0)
      printf("Calulating 64b dotp with vectors with length = %lu\n", len);

    // Align all harts on kernel entry: the printfs above must not skew it.
    sync_barrier();

    if (hart_id == 0)
      start_timer();

    *res64_core = fdotp_v64b(v64a_i, v64b_i, avl_i);

    // Every hart's kernel has retired and its partial has landed in L2.
    sync_barrier();

    if (hart_id == 0) {
      stop_timer();
      runtime_v = get_timer();
      printf("Vector runtime: %ld\n", runtime_v);

      float performance = 2.0 * len / runtime_v;
      float utilization = 100 * performance /
                          (2.0 * NR_LANES * NR_CLUSTERS * NR_CORES);
      printf("The performance is %f FLOP/cycle (%f%% utilization).\n",
           performance, utilization);

      // res64_part[] is non-cacheable, so the sync_barrier() above is all that
      // is needed to order every hart's partial store before these loads.
      res64_v = 0.0;
      for (int c = 0; c < NR_CORES; ++c)
        res64_v += res64_part[c];

      // Scalar pass: full length only, it costs a full scalar FP loop in
      // RTL simulation.
      if (SCALAR && len == vsize) {
        start_timer();
        res64_s = fdotp_s64b(v64a, v64b, len);
        stop_timer();
        runtime_s = get_timer();
        printf("Scalar runtime: %ld\n", runtime_s);
      }

      // Both references span the whole vector, so they are only valid on the
      // full-length iteration.
      if (CHECK && len == vsize) {
        if (SCALAR) {
          printf("Checking results: v = %f, s = %f\n", res64_v, res64_s);
          if (!similarity_check(res64_v, res64_s, THRESHOLD_64b)) {
            printf("Error: v = %f, s = %f\n", res64_v, res64_s);
            return -1;
          }
        } else {
          printf("Checking results: v = %f, golden = %f\n", res64_v, gold64);
          if (!similarity_check(res64_v, gold64, THRESHOLD_64b)) {
            printf("Error: v = %f, golden = %f\n", res64_v, gold64);
            return -1;
          }
        }
      }
    }
  }

  /*
  // for (uint64_t avl = 16; avl <= (vsize); avl *= 2) {
    uint64_t avl=vsize;
    printf("Calulating 32b dotp with vectors with length = %lu\n", avl);
    start_timer();
    res32_v = fdotp_v32b(v32a, v32b, avl);
    stop_timer();
    runtime_v = get_timer();
    printf("Vector runtime: %ld\n", runtime_v);

    if (SCALAR) {
      start_timer();
      res32_s = fdotp_s32b(v32a, v32b, avl);
      stop_timer();
      runtime_s = get_timer();
      printf("Scalar runtime: %ld\n", runtime_s);
    }

    if (CHECK) {
      if (SCALAR) {
        printf("Checking results: v = %f, s = %f\n", res32_v, res32_s);
        if (!similarity_check_32b(res32_v, res32_s, THRESHOLD_32b)) {
          printf("Error: v = %f, s = %f\n", res32_v, res32_s);
          return -1;
        }
      }
    }

    // Dotproduct Arithmetic intensity calculation
    // Ops = 2N FP32 ops 
    // Bytes = 2N * 4B = 8N Bytes , AI = 1/4 FP32 Op/B
    // BW = 32N bits/ cycle = 4N Bytes
    // Max Perf = N * 2 * 2 FP32op/cycle = 4N FP32 op/cycle
    // From roofline max perf at the arithmetic intensity = N FP32 op/cycle
    float performance = avl * 1.0 / runtime_v;
    float utilization = 100.0 * performance / (NR_LANES * NR_CLUSTERS);
    printf("The execution took %d cycles.\n", runtime_v);
    printf("The performance is %f FLOP/cycle (%f%% utilization).\n",
           performance, utilization);

  // }*/
  
  /*
  for (uint64_t avl = 8; avl <= (vsize >> 2); avl *= 8) {
    // Dotp
    printf("Calulating 16b dotp with vectors with length = %lu\n", avl);
    start_timer();
    res16_v = fdotp_v16b(v16a, v16b, avl);
    stop_timer();
    runtime_v = get_timer();
    printf("Vector runtime: %ld\n", runtime_v);

    if (SCALAR) {
      start_timer();
      res16_s = fdotp_s16b(v16a, v16b, avl);
      stop_timer();
      runtime_s = get_timer();
      printf("Scalar runtime: %ld\n", runtime_s);
    }

    if (CHECK) {
      if (SCALAR) {
        printf("Checking results: v = %x, s = %x\n", *((uint16_t *)&res16_v),
               *((uint16_t *)&res16_s));
        if (!similarity_check(res16_v, res16_s, THRESHOLD_16b)) {
          printf("Error: v = %x, s = %x\n", *((uint16_t *)&res16_v),
                 *((uint16_t *)&res16_s));
          return -1;
        }
      }
    }
  }*/

  if (hart_id == 0)
    printf("SUCCESS.\n");

  // ara_tb.sv:232 $finish-es on the FIRST write to eoc_address_reg, and
  // crt0.S:_eoc writes it as soon as main returns. Without this barrier
  // hart 1 would end the simulation while hart 0 is still verifying.
  sync_barrier();

  return 0;
}
