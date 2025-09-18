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
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#ifndef _PATHFINDER_H_
#define _PATHFINDER_H_

#include <stdint.h>

#include "riscv_vector.h"

#include "util.h"

#ifndef SPIKE
#include "printf.h"
#else
#include <stdio.h>
#endif

int *run(int *wall, int *result_s, int *src, uint32_t cols, uint32_t rows,
         uint32_t num_runs);
void run_vector(int *wall, int *result_v, uint32_t cols, uint32_t rows,
                uint32_t num_runs);
void run_vector_short_m4(int *wall, int *result_v, uint32_t cols, uint32_t rows,
                         uint32_t num_runs, int neutral_value);

#endif
