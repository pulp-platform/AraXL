// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//         Basile Bougenot <bbougenot@student.ethz.ch>
//         Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>

#include "vector_macros.h"
#include "long_array.h"

uint8_t mask_1[2] = {0xAA, 0xAA};
uint8_t mask_2[2] = {0x55, 0x55};

void TEST_CASE1() {
  uint64_t scalar = 99;

  VSET(32, e8, m1);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
          20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e8, m1);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile("vslide1down.vx v1, v2, %[A]" ::[A] "r"(scalar));
  VCMP_U8(1, v1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 99);

  VSET(32, e16, m1);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e16, m1);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile("vslide1down.vx v1, v2, %[A]" ::[A] "r"(scalar));
  VCMP_U16(2, v1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 99);

  VSET(32, e32, m1);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e32, m1);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_32(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile("vslide1down.vx v1, v2, %[A]" ::[A] "r"(scalar));
  VCMP_U32(3, v1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 99);

  VSET(32, e64, m1);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e64, m1);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_64(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile("vslide1down.vx v1, v2, %[A]" ::[A] "r"(scalar));
  VCMP_U64(4, v1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 99);
}

void TEST_CASE2() {
  uint64_t scalar = 99;

  VSET(32, e8, m1);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
          20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e8, m1);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile ("vlm.v v0, (%0)"::"r"(&mask_1));
  asm volatile("vslide1down.vx v1, v2, %[A], v0.t" ::[A] "r"(scalar));
  VCMP_U8(5, v1, -1, 3, -1, 5, -1, 7, -1, 9, -1, 11, -1, 13, -1, 15, -1, 99);

  VSET(32, e16, m1);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e16, m1);
  VLOAD_16(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile ("vlm.v v0, (%0)"::"r"(&mask_2));
  asm volatile("vslide1down.vx v1, v2, %[A], v0.t" ::[A] "r"(scalar));
  VCMP_U16(6, v1, 2, -1, 4, -1, 6, -1, 8, -1, 10, -1, 12, -1, 14, -1, 16, -1);

  VSET(32, e32, m1);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e32, m1);
  VLOAD_32(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_32(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile ("vlm.v v0, (%0)"::"r"(&mask_1));
  asm volatile("vslide1down.vx v1, v2, %[A], v0.t" ::[A] "r"(scalar));
  VCMP_U32(7, v1, -1, 3, -1, 5, -1, 7, -1, 9, -1, 11, -1, 13, -1, 15, -1, 99);

  VSET(32, e64, m1);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
           19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  VSET(16, e64, m1);
  VLOAD_64(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_64(v1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
  asm volatile ("vlm.v v0, (%0)"::"r"(&mask_2));
  asm volatile("vslide1down.vx v1, v2, %[A], v0.t" ::[A] "r"(scalar));
  VCMP_U64(8, v1, 2, -1, 4, -1, 6, -1, 8, -1, 10, -1, 12, -1, 14, -1, 16, -1);
}

//**** VL regression SEW=8 *****//

// Test cases to load irregular vector lengths and misaligned start address
uint8_t res_8[20][256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
void TEST_CASE3(void) {
  printf("Running vl regression tests for slide1down SEW=8\n");
  int avl, vl;

  uint8_t a = 0xDE;

  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    vl = vl_list[i];

    printf ("Testing with vl=%d\n", vl);

    asm volatile("vsetvli %0, %1, e8, m2, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile ("vle8.v v2, (%0)"::"r"(LONG_I8));
    asm volatile ("vslide1down.vx v8, v2, %0"::"r"(a));
    asm volatile ("vse8.v v8, (%0)"::"r"(res_8[i]));

    // check results
    for (int idx = 0; idx < vl; idx++) {
      if (idx == vl - 1) {
        if (res_8[i][idx] != a) {
          printf("Error at index %d: expected %x, got %x\n", idx, a, res_8[i][idx]);
          num_failed++;
          return;
        }
      } else {
        if (res_8[i][idx] != LONG_I8[idx+1]) {
          printf("Error at index %d: expected %x, got %x\n", idx, LONG_I8[idx+1], res_8[i][idx]);
          num_failed++;
          return;
        }
      }
    }
  }
}

//**** VL regression SEW=16 *****//

// Test cases to load irregular vector lengths and misaligned start address
uint16_t res_16[20][256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
void TEST_CASE4(void) {
  printf("Running vl regression tests for slide1down SEW=16\n");
  int avl, vl;

  uint16_t a = 0xDEAD;

  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    vl = vl_list[i];

    printf ("Testing with vl=%d\n", vl);

    asm volatile("vsetvli %0, %1, e16, m2, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile ("vle16.v v2, (%0)"::"r"(LONG_I16));
    asm volatile ("vslide1down.vx v8, v2, %0"::"r"(a));
    asm volatile ("vse16.v v8, (%0)"::"r"(res_16[i]));

    // check results
    for (int idx = 0; idx < vl; idx++) {
      if (idx == vl - 1) {
        if (res_16[i][idx] != a) {
          printf("Error at index %d: expected %x, got %x\n", idx, a, res_16[i][idx]);
          num_failed++;
          return;
        }
      } else {
        if (res_16[i][idx] != LONG_I16[idx+1]) {
          printf("Error at index %d: expected %x, got %x\n", idx, LONG_I16[idx+1], res_16[i][idx]);
          num_failed++;
          return;
        }
      }
    }
  }
}

//**** VL regression SEW=32 *****//

// Test cases to load irregular vector lengths and misaligned start address
uint32_t res_32[20][256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));
void TEST_CASE5(void) {
  printf("Running vl regression tests for slide1down SEW=32\n");
  int avl, vl;

  uint32_t a = 0xDEADBEEF;

  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    vl = vl_list[i];

    printf ("Testing with vl=%d\n", vl);

    asm volatile("vsetvli %0, %1, e32, m2, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile ("vle32.v v2, (%0)"::"r"(LONG_I32));
    asm volatile ("vslide1down.vx v8, v2, %0"::"r"(a));
    asm volatile ("vse32.v v8, (%0)"::"r"(res_32[i]));

    // check results
    for (int idx = 0; idx < vl; idx++) {
      if (idx == vl - 1) {
        if (res_32[i][idx] != a) {
          printf("Error at index %d: expected %x, got %x\n", idx, a, res_32[i][idx]);
          num_failed++;
          return;
        }
      } else {
        if (res_32[i][idx] != LONG_I32[idx+1]) {
          printf("Error at index %d: expected %x, got %x\n", idx, LONG_I32[idx+1], res_32[i][idx]);
          num_failed++;
          return;
        }
      }
    }
  }
}

//**** VL regression SEW=64 *****//

// Test cases to load irregular vector lengths and misaligned start address
uint64_t res[20][256] __attribute__((aligned(4 * NR_LANES * NR_CLUSTERS), section(".l2")));

void TEST_CASE6() {
  printf("Running vl regression tests for slide1down SEW=64\n");

  int avl, vl;
  uint64_t a = 0xDEADBEEFDEADBEEF;

  int vl_list[20] = {143, 27, 198, 64, 215, 9, 172, 54, 241, 88,
                      36, 159, 203, 71, 119, 6, 187, 95, 222, 41};
  
  for (int i=0; i<20; i++) {
    vl = vl_list[i];

    printf ("Testing with vl=%d\n", vl);

    asm volatile("vsetvli %0, %1, e64, m2, ta, ma" : "=r"(avl) : "r"(vl));
    asm volatile ("vle64.v v2, (%0)"::"r"(&LONG_I64[0]));
    asm volatile ("vslide1down.vx v8, v2, %0"::"r"(a));
    asm volatile ("vse64.v v8, (%0)"::"r"(res[i]));

    // check results
    for (int idx = 0; idx < vl; idx++) {
      if (idx == (vl - 1)) {
        if (res[i][idx] != a) {
          printf("Error at index %d: expected %lx, got %lx\n", idx, a, res[i][idx]);
          num_failed++;
          return;
        }
      } else {
        if (res[i][idx] != LONG_I64[idx+1]) {
          printf("Error at index %d: expected %lx, got %lx\n", idx, LONG_I64[idx+1], res[i][idx]);
          num_failed++;
          return;
        }
      }
    }
  }
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

  EXIT_CHECK();
}
