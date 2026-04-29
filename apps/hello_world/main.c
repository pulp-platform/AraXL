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

#include <stdint.h>
#include <string.h>
#include "runtime.h"

#ifndef SPIKE
#include "printf.h"
#else
#include "util.h"
#include <stdio.h>
#endif

int main(int hart_id) {
  
  static int k=0;

  if (hart_id == 0) {
    mutex_lock_acquire();
    k+=3;
    mutex_lock_release();
  }

  if (hart_id == 1) {
    mutex_lock_acquire();
    k+=2;
    mutex_lock_release();
  }
  sync_barrier();

  if (hart_id == 0) {
    printf_("Hello World from core %d k=%d!\n", hart_id, k);
  }
  sync_barrier();

  return 0;
}
