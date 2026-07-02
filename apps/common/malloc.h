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
// Utility functions for baremetal malloc

#ifndef _MALLOC_H_
#define _MALLOC_H_

#include <stdint.h>

// Allocate atomic variables in non-cacheable L2 heap
static inline void * baremetal_atomic_malloc(int incr)
{
    extern char l2_atomic_alloc_base;   /* Set by linker */
    static char *atomic_heap_end = 0;

    uintptr_t aligned;
    char *result;

    /* First call: initialize heap */
    if (!atomic_heap_end)
        atomic_heap_end = &l2_atomic_alloc_base;

    /* Align current heap pointer */
    aligned = (uintptr_t)atomic_heap_end;
    result = (char *)aligned;

    /* Move heap past allocated block */
    atomic_heap_end = result + incr;

    return (void *)result;
}

// Allocate variables in L2 heap
#define ALIGNMENT (NR_LANES * NR_CLUSTERS * 4)

#define ALIGN_UP(x, a)  (((x) + (a) - 1) & ~((a) - 1))

inline void * baremetal_malloc(int incr)
{
    extern char l2_alloc_base;   /* Set by linker */
    static char *heap_end = 0;

    uintptr_t aligned;
    char *result;

    /* First call: initialize heap */
    if (!heap_end)
        heap_end = &l2_alloc_base;

    /* Align current heap pointer */
    aligned = ALIGN_UP((uintptr_t)heap_end, ALIGNMENT);

    result = (char *)aligned;

    /* Move heap past allocated block */
    heap_end = result + incr;

    return (void *)result;
}

#endif // _MALLOC_H_
