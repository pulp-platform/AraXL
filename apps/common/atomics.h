// Copyright 2026 ETH Zurich and University of Bologna.
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
// Author: Navaneeth Kunhi Purayil, ETH Zurich
//
// Utility functions for baremetal atomics

#ifndef _ATOMICS_H_
#define _ATOMICS_H_

// Use compiler __atomic builtins directly so the type works in both C and C++
// without requiring <stdatomic.h> or <atomic>.
typedef int atomic_int_t;

extern atomic_int_t sync_flag;
extern atomic_int_t sync_generation;

/////////////
// Barrier //
/////////////

// Barrier implementation using atomic operations for multi-core configurations
inline void sync_barrier() {
# if NR_CORES > 1
  // Snapshot generation before incrementing count so we know which
  // generation we are waiting for.
  int gen = __atomic_load_n(&sync_generation, __ATOMIC_SEQ_CST);
  if (__atomic_fetch_add(&sync_flag, 1, __ATOMIC_SEQ_CST) == NR_CORES - 1) {
    // Last core: reset count then bump generation to release all spinners.
    __atomic_store_n(&sync_flag, 0, __ATOMIC_SEQ_CST);
    __atomic_fetch_add(&sync_generation, 1, __ATOMIC_SEQ_CST);
  } else {
    // Spin on generation, not on sync_flag, so a fast core racing back
    // to the next barrier cannot cause a deadlock.
    while (__atomic_load_n(&sync_generation, __ATOMIC_SEQ_CST) == gen)
      ;
  }
#endif
}

/////////////////
// CAS atomics //
/////////////////

inline void acquire_lock(atomic_int_t *lock) {
  while (__atomic_exchange_n(lock, 1, __ATOMIC_ACQUIRE)) {
    while (__atomic_load_n(lock, __ATOMIC_RELAXED))
      ;
  }
}

inline void release_lock(atomic_int_t *lock) {
  __atomic_store_n(lock, 0, __ATOMIC_RELEASE);
}

///////////
// Fence //
///////////

// To invalidate CVA6 caches
// Set CVA6ConfigDcacheFlushOnFence = 1 and CVA6ConfigDcacheInvalidateOnFlush = 1 in cv64a6_imafdcv_sv39_config_pkg.sv
// Depending on the requied configuration, the fence instruction will flush and invalidate the D-cache

inline void fence()
{
  asm volatile ("fence" ::: "memory");
}

static inline void flush_sync_barrier()
{
#if NR_CORES > 1
  fence();
  sync_barrier();
  fence();
#endif
}

extern atomic_int_t mutex_lock;

static inline void mutex_lock_acquire()
{
# if NR_CORES > 1
  acquire_lock(&mutex_lock);
#endif
}

static inline void mutex_lock_release()
{
# if NR_CORES > 1
  fence();
  release_lock(&mutex_lock);
#endif
}

#endif // _ATOMICS_H_
