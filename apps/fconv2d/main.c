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

// Author: Matteo Perotti

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fconv2d32.h"
#include "fconv2d.h"
#include "runtime.h"
#include "util.h"

#ifndef SPIKE
#include "printf.h"
#endif

// Uncomment the required data type
#define T double
// #define T float

// Comment to use FP64
// #define useFP32 1

// Define Matrix dimensions:
// o = i ° f, with i=[MxN], f=[FxF], o=[MxN]
// The filter is a square matrix, and F is odd

// Matrices defined in data.S
extern T i[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS)));        // [ (M+floor(F/2)) * (N+floor(F/2)) ]
extern T f[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS)));        // [ F*F ]
extern T o[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS)));        // [ M*N ]
extern T golden_o[] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS))); // [ M*N ]

// M, N, F defined in data.S
extern int64_t M;
extern int64_t N;
extern int64_t F;

// Verify the matrices
int verify_matrix(T *matrix, T *golden_matrix, int64_t R, int64_t C, T threshold) 
{
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c)
    #ifdef useFP32
      if (!similarity_check_32b(matrix[c + C * r], golden_matrix[c + C * r],
                            threshold)) {

    #else
      if (!similarity_check(matrix[c + C * r], golden_matrix[c + C * r],
                            threshold)) {
    #endif
        printf("Error: o[%d][%d] = %lf, instead of %lf\n", r, c,
               matrix[c + C * r], golden_matrix[c + C * r]);
        // return 1;
      }
  return 0;
}


void print_matrix(T const *matrix, uint64_t num_rows,
                  uint64_t num_columns) {
printf("0x%8X\n", (uint64_t)matrix);
  for (uint64_t i = 0; i < num_rows; ++i) {
    for (uint64_t j = 0; j < num_columns; ++j) {
      printf("%10f ", matrix[i * num_columns + j]);
    }
    printf("\n");
  }
}

int main(int hart_id) {
  if (hart_id == 0) {
    printf("\n");
    printf("=============\n");
    printf("=  FCONV2D  =\n");
    printf("=============\n");
    printf("\n");
    printf("\n");
  }

  int64_t row_start = (M * hart_id) / NR_CORES;
  int64_t row_end = (M * (hart_id + 1)) / NR_CORES;
  int64_t M_hart = row_end - row_start;
  T *i_start = i + row_start * (N + F - 1);
  T *o_start = o + row_start * N;

  sync_barrier();

  if (hart_id == 0) {
    printf("From Hart:%d i:%x f:%x o:%x rows:%d-%d\n", hart_id, i_start, f,
           o_start, row_start, row_end);
  }

  sync_barrier();

  if (hart_id == 1) {
    printf("From Hart:%d i:%x f:%x o:%x rows:%d-%d\n", hart_id, i_start, f,
           o_start, row_start, row_end);
  }

  sync_barrier();

  // Call the main kernel, and measure cycles
  if (hart_id == 0) {
    start_timer();
  }
  sync_barrier();
  if (F == 3)
    fconv2d_3x3(o_start, i_start, f, M_hart, N, F);
  else if (F == 7)
    #ifdef useFP32
    fconv2d32_7x7(o_start, i_start, f, M_hart, N, F);
    #else
    fconv2d_7x7(o_start, i_start, f, M_hart, N, F);
    #endif
  else if (hart_id == 0)
    printf("Error: the filter size is different from 3 or 7.\n");
  sync_barrier();
  if (hart_id == 0) {
    stop_timer();
  }

  // Performance metrics
  int error = 0;
  if (hart_id == 0) {
    int64_t runtime = get_timer();
    float performance = 2.0 * F * F * M * N / runtime;
#ifdef useFP32
    float utilization =
        100 * performance / (2.0 * NR_LANES * NR_CLUSTERS * NR_CORES * 2.0);
#else
    float utilization =
        100 * performance / (2.0 * NR_LANES * NR_CLUSTERS * NR_CORES);
#endif

    printf("The execution took %d cycles.\n", runtime);
    printf("The performance is %f DPFLOP/cycle (%f%% utilization).\n",
           performance, utilization);

    // Verify correctness
    printf("Verifying result...\n");
#ifdef useFP32
    error = verify_matrix(o, golden_o, M, N, THRESHOLD32);
#else
    error = verify_matrix(o, golden_o, M, N, THRESHOLD);
#endif

    if (error != 0) {
      printf("Fail.\n");
    } else {
      printf("Passed.\n");
    }
  }
  sync_barrier();

  return error;
}
