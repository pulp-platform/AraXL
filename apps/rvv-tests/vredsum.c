// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#include "vector_macros.h"

// Naive test
void TEST_CASE1(void) {
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U8(1, v3, 73);

  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 1);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U16(2, v3, 73);

  VSET(16, e32, m1);
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U32(3, v3, 73);

  VSET(16, e64, m1);
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U64(4, v3, 73);
}

// Masked naive test
void TEST_CASE2(void) {
  uint8_t mask[2] = {0xAA, 0x55};
  VSET(16, e8, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1);
  VLOAD_8(v3, 1);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U8(5, v3, 37);

  VSET(16, e16, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 1);
  VLOAD_16(v3, 1);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U16(6, v3, 37);

  VSET(16, e32, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1);
  VLOAD_32(v3, 1);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U32(7, v3, 37);

  VSET(16, e64, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1);
  VLOAD_64(v3, 1);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U64(8, v3, 37);
}

// Are we respecting the undisturbed tail policy?
void TEST_CASE3(void) {
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U8(9, v3, 73, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U16(10, v3, 73, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(16, e32, m1);
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U32(11, v3, 73, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(16, e64, m1);
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U64(12, v3, 73, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
}

// Odd number of elements, undisturbed policy
void TEST_CASE4(void) {
  VSET(15, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U8(13, v3, 65, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(1, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U16(14, v3, 2, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(3, e32, m1);
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U32(15, v3, 7, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(7, e64, m1);
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U64(16, v3, 29, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(15, e64, m1);
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2");
  VCMP_U64(17, v3, 65, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
}

// Odd number of elements, undisturbed policy, and mask
void TEST_CASE5(void) {
  uint8_t mask[2] = {0x00, 0x40};
  VSET(15, e8, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 100, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U8(18, v3, 107, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  mask[0] = 0xaa;
  mask[1] = 0x55;
  VSET(1, e16, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U16(19, v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);

  VSET(3, e32, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v3, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vredsum.vs v3, v1, v2, v0.t");
  VCMP_U32(20, v3, 3, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
}

//**** VL regression SEW=8 *****//

uint8_t src_8[256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

void TEST_CASE6(void) {

  for (int i=0; i<256; i++) {
    src_8[i] = i+1;
  }
  uint8_t scalar = 1, result, expected;
  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    uint32_t vl = vl_list[i], avl;
    printf("Testing with vl = %u\n", vl);
    
    asm volatile("vsetvli %0, %1, e8, m4, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile("vmv.v.x v4, %0"::"r"(scalar));
    asm volatile("vle8.v v8, (%0)" :: "r"(src_8));
    asm volatile("vredsum.vs v0, v8, v4");
    asm volatile("vmv.x.s %0, v0" : "=r"(result));

    expected = vl * (vl + 1) / 2 + scalar; // sum of first n integers + scalar
    if (result != expected) {
      printf("Error: expected %hhu, got %hhu\n", expected, result);
      num_failed++;
      return;
    }
  }
  printf("PASSED!\n");
}

//**** VL regression SEW=16 *****//

uint16_t src_16[256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

void TEST_CASE7(void) {

  for (int i=0; i<256; i++) {
    src_16[i] = i+1;
  }
  uint16_t scalar = 1, result, expected;
  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    uint32_t vl = vl_list[i], avl;
    printf("Testing with vl = %u\n", vl);
    
    asm volatile("vsetvli %0, %1, e16, m4, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile("vmv.v.x v4, %0"::"r"(scalar));
    asm volatile("vle16.v v8, (%0)" :: "r"(src_16));
    asm volatile("vredsum.vs v0, v8, v4");
    asm volatile("vmv.x.s %0, v0" : "=r"(result));

    expected = vl * (vl + 1) / 2 + scalar; // sum of first n integers + scalar
    if (result != expected) {
      printf("Error: expected %hu, got %hu\n", expected, result);
      num_failed++;
      return;
    }
  }
  printf("PASSED!\n");
}

//**** VL regression SEW=32 *****//

uint32_t src_32[256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

void TEST_CASE8(void) {

  for (int i=0; i<256; i++) {
    src_32[i] = i+1;
  }
  uint32_t scalar = 1, result, expected;
  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    uint32_t vl = vl_list[i], avl;
    printf("Testing with vl = %u\n", vl);
    
    asm volatile("vsetvli %0, %1, e32, m4, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile("vmv.v.x v4, %0"::"r"(scalar));
    asm volatile("vle32.v v8, (%0)" :: "r"(src_32));
    asm volatile("vredsum.vs v0, v8, v4");
    asm volatile("vmv.x.s %0, v0" : "=r"(result));

    expected = vl * (vl + 1) / 2 + scalar; // sum of first n integers + scalar
    if (result != expected) {
      printf("Error: expected %u, got %u\n", expected, result);
      num_failed++;
      return;
    }
  }
  printf("PASSED!\n");
}

//**** VL regression SEW=64 *****//

uint64_t src[256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

void TEST_CASE9(void) {

  for (int i=0; i<256; i++) {
    src[i] = i+1;
  }
  uint64_t scalar = 1, result, expected;
  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    uint32_t vl = vl_list[i], avl;
    printf("Testing with vl = %u\n", vl);
    
    asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile("vmv.v.x v4, %0"::"r"(scalar));
    asm volatile("vle64.v v8, (%0)" :: "r"(src));
    asm volatile("vredsum.vs v0, v8, v4");
    asm volatile("vmv.x.s %0, v0" : "=r"(result));

    expected = vl * (vl + 1) / 2 + scalar; // sum of first n integers + scalar
    if (result != expected) {
      printf("Error: expected %lu, got %lu\n", expected, result);
      num_failed++;
      return;
    }
  }
  printf("PASSED!\n");
}

int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_CASE1();
  TEST_CASE2();
  TEST_CASE3();
  TEST_CASE4();
  TEST_CASE5();

  TEST_CASE6();
  TEST_CASE7();
  TEST_CASE8();
  TEST_CASE9();

  EXIT_CHECK();
}
