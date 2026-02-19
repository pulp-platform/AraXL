// Copyright 2020-2025 ETH Zurich and University of Bologna.
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

// Author: Matheus Cavalcante, ETH Zurich
//         Samuel Riedel, ETH Zurich
//         Matteo Perotti, ETH Zurich

#include <stdio.h>
#include <string.h>

#include "kernel/fmatmul.h"
#include "kernel/fmatmul32.h"
#include "runtime.h"
#include "util.h"

#ifndef SPIKE
#include "printf.h"
#endif

// Define Matrix dimensions:
// C = AB with A=[MxN], B=[NxP], C=[MxP]
extern uint64_t M;
extern uint64_t N;
extern uint64_t P;

// #define FP32 1

#ifndef FP32
extern double a[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern double b[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern double c[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
// Gold results
extern double g[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

// Verify the matrix
int verify_matrix(double *result, double *gold, size_t R, size_t C,
                  double threshold) {
  for (uint64_t i = 0; i < R; ++i) {
    for (uint64_t j = 0; j < C; ++j) {
      uint64_t idx = i * C + j;
      if (!similarity_check(result[idx], gold[idx], threshold)) {
        return (i + j) == 0 ? -1 : idx;
      }
    }
  }
  return 0;
}

#else

extern float a[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern float b[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
extern float c[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
// Gold results
extern float g[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

// Verify the matrix
int verify_matrix(float *result, float *gold, size_t R, size_t C,
                  float threshold) {
  for (uint64_t i = 0; i < R; ++i) {
    for (uint64_t j = 0; j < C; ++j) {
      uint64_t idx = i * C + j;
      // printf("%d res:%f gold:%f\n",idx, result[idx], gold[idx]);
      if (!similarity_check_32b(result[idx], gold[idx], threshold)) {
        printf("%d res:%f gold:%f\n",idx, result[idx], gold[idx]);
        // return (i + j) == 0 ? -1 : idx;
      }
    }
  }
  return 0;
}

#endif

#define THRESHOLD64 0.000001
#define THRESHOLD32 0.001

int main(int hart_id) {
  if (hart_id == 0) {
    printf("\n");
    printf("=============\n");
    printf("=  FMATMUL  =\n");
    printf("=============\n");
    printf("\n");
    printf("\n");
  }

  uint64_t P_hart = P / NR_CORES;
  double *c_start = c + hart_id * P_hart;
  double *b_start = b + hart_id * P_hart;

  sync_barrier();

  if (hart_id == 0) {
    printf("From Hart:%d a:%x b:%x c:%x\n", hart_id, a, b_start, c_start);
  }

  sync_barrier();

  if (hart_id == 1) {
    printf("From Hart:%d a:%x b:%x c:%x\n", hart_id, a, b_start, c_start);
  }

  sync_barrier();

#ifdef VCD_DUMP
  // Measure only the full-size matmul
  for (uint64_t s = M; s <= M; s *= 2) {
#else
  for (uint64_t s = M; s <= M; s *= 2) {
#endif

    if (hart_id == 0) {
      printf("\n");
      printf("------------------------------------------------------------\n");
      printf("Calculating a (%d x %d) x (%d x %d) matrix multiplication...\n", s,
            N, N, P);
      printf("------------------------------------------------------------\n");
      printf("\n");
    }
    // Matrices are initialized --> Start calculating
#ifdef FP32
    if (hart_id == 0) {
      printf("Calculating fmatmul 32-bit...\n");
      start_timer();
    }
    
    fmatmul32(c_start, a, b_start, s, N, P_hart);
    if (hart_id == 0) {
      stop_timer();
    }
#else
    if (hart_id == 0) {
      printf("Calculating fmatmul 64-bit...\n");
      start_timer();
    }
    // Synchronize cores
    sync_barrier();
    fmatmul(c_start, a, b_start, s, N, P_hart);
    if (hart_id == 0) {
      stop_timer();
    }
#endif

    if (hart_id == 0) {
      // Metrics
      int64_t runtime = get_timer();

      float performance = 2.0 * s * N * P / runtime;
      float utilization;
      #ifdef FP32
      utilization = 100 * performance / (2.0 * NR_LANES * NR_CLUSTERS * NR_CORES * 2.0); // 2 FP32-ops/lane/cycle  
      #else
      utilization = 100 * performance / (2.0 * NR_LANES * NR_CLUSTERS * NR_CORES);
      #endif

      printf("The execution took %d cycles.\n", runtime);
      printf("The performance is %f FLOP/cycle (%f%% utilization).\n",
            performance, utilization);

      // Verify the result only for s == M (to keep it simple)
      if (s == M) {
        printf("Verifying result...\n");
        int error;
        
        #ifdef FP32
        error = verify_matrix(c, g, s, P, THRESHOLD32);
        #else
        error = verify_matrix(c, g, s, P, THRESHOLD64);
        #endif

        if (error != 0) {
          printf("Error code %d\n", error);
          printf("c[%d]=%d\n", error, c[error]);
          return error;
        } else {
          printf("Passed.\n");
        }
      }
    }
  }

  sync_barrier();

  return 0;
}
