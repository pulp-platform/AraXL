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
#define THRESHOLD_32b 0.01
#define THRESHOLD_16b 1

// Run also the scalar benchmark
#define SCALAR 1

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

int main() {
  printf("\n");
  printf("===========\n");
  printf("=  FDOTP  =\n");
  printf("===========\n");
  printf("\n");
  printf("\n");

  uint64_t runtime_s, runtime_v;

  for (uint64_t avl = 8; avl <= (vsize); avl *= 8) {
    printf("Calulating 64b dotp with vectors with length = %lu\n", avl);
    start_timer();
    res64_v = fdotp_v64b(v64a, v64b, avl);
    stop_timer();
    runtime_v = get_timer();
    printf("Vector runtime: %ld\n", runtime_v);

    if (SCALAR) {
      start_timer();
      res64_s = fdotp_s64b(v64a, v64b, avl);
      stop_timer();
      runtime_s = get_timer();
      printf("Scalar runtime: %ld\n", runtime_s);
    }

    if (CHECK) {
      if (SCALAR) {
        printf("Checking results: v = %f, s = %f\n", res64_v, res64_s);
        if (!similarity_check(res64_v, res64_s, THRESHOLD_64b)) {
          printf("Error: v = %f, s = %f\n", res64_v, res64_s);
          return -1;
        }
      }
    }
  }


  // for (uint64_t avl = 112; avl <= (vsize); avl *= 2) {
  //   // start_timer();
  //   // res32_s = fdotp_s32b(v32a, v32b, avl);
  //   // stop_timer();
  //   // runtime_s = get_timer();
  //   // printf("Scalar runtime: %ld\n, result: %f\n", runtime_s, res32_s);

  //   printf("Calulating 32b dotp with vectors with length = %lu\n", avl);
  //   start_timer();
  //   res32_v = fdotp_v32b(v32a, v32b, avl);
  //   stop_timer();
  //   runtime_v = get_timer();
  //   printf("Vector runtime: %ld\n", runtime_v);

  //   if (SCALAR) {
  //     start_timer();
  //     res32_s = fdotp_s32b(v32a, v32b, avl);
  //     stop_timer();
  //     runtime_s = get_timer();
  //     printf("Scalar runtime: %ld\n, result: %f\n", runtime_s, res32_s);
  //   }

  //   if (CHECK) {
  //     if (SCALAR) {
  //       printf("Checking results: v = %f, s = %f\n", res32_v, res32_s);
  //       if (!similarity_check_32b(res32_v, res32_s, THRESHOLD_32b)) {
  //         printf("Error: v = %f, s = %f\n", res32_v, res32_s);
  //         return -1;
  //       }
  //     }
  //   }

  //   // // Dotproduct Arithmetic intensity calculation
  //   // // Ops = 2N FP32 ops 
  //   // // Bytes = 2N * 4B = 8N Bytes , AI = 1/4 FP32 Op/B
  //   // // BW = 32N bits/ cycle = 4N Bytes
  //   // // Max Perf = N * 2 * 2 FP32op/cycle = 4N FP32 op/cycle
  //   // // From roofline max perf at the arithmetic intensity = N FP32 op/cycle
  //   // float performance = avl * 1.0 / runtime_v;
  //   // float utilization = 100.0 * performance / (NR_LANES * NR_CLUSTERS);
  //   // printf("The execution took %d cycles.\n", runtime_v);
  //   // printf("The performance is %f FLOP/cycle (%f%% utilization).\n",
  //   //        performance, utilization);
  // }
  

  // for (uint64_t avl = 8; avl <= (vsize); avl *= 8) {
  //   // Dotp
  //   printf("Calulating 16b dotp with vectors with length = %lu\n", avl);
  //   start_timer();
  //   res16_v = fdotp_v16b(v16a, v16b, avl);
  //   stop_timer();
  //   runtime_v = get_timer();
  //   printf("Vector runtime: %ld\n", runtime_v);

  //   if (SCALAR) {
  //     start_timer();
  //     res16_s = fdotp_s16b(v16a, v16b, avl);
  //     stop_timer();
  //     runtime_s = get_timer();
  //     printf("Scalar runtime: %ld\n", runtime_s);
  //   }

  //   if (CHECK) {
  //     if (SCALAR) {
  //       printf("Checking results: v = %x, s = %x\n", *((uint16_t *)&res16_v),
  //              *((uint16_t *)&res16_s));
  //       if (!similarity_check(res16_v, res16_s, THRESHOLD_16b)) {
  //         printf("Error: v = %x, s = %x\n", *((uint16_t *)&res16_v),
  //                *((uint16_t *)&res16_s));
  //         return -1;
  //       }
  //     }
  //   }
  // }

  printf("SUCCESS.\n");

  return 0;
}
