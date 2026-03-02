// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#include "vector_macros.h"

// Naive test
void TEST_CASE1(void) {
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 9, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U8(1, v3, 9);

  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 9);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U16(2, v3, 9);

  VSET(16, e32, m1);
  VLOAD_32(v1, 9, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U32(3, v3, 9);

  VSET(16, e64, m1);
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 9, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U64(4, v3, 9);
}
// Masked naive test
void TEST_CASE2(void) {
  uint8_t mask[2] = {0x03, 0x00};
  VSET(16, e8, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 9, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1);
  VLOAD_8(v3, 1);
  asm volatile("vredmaxu.vs v3, v1, v2, v0.t");
  VCMP_U8(5, v3, 2);

  mask[0] = 0x00;
  mask[1] = 0xC0;
  VSET(16, e16, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, 9);
  VLOAD_16(v3, 1);
  asm volatile("vredmaxu.vs v3, v1, v2, v0.t");
  VCMP_U16(6, v3, 9);

  VSET(16, e32, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_32(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1);
  VLOAD_32(v3, 1);
  asm volatile("vredmaxu.vs v3, v1, v2, v0.t");
  VCMP_U32(7, v3, 8);

  mask[0] = 0xAA;
  mask[1] = 0x55;
  VSET(16, e64, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_64(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, 4);
  VLOAD_64(v3, 1);
  asm volatile("vredmaxu.vs v3, v1, v2, v0.t");
  VCMP_U64(8, v3, 8);
}

// Naive test with negative values
void TEST_CASE3(void) {
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 9, -3, 4, 5, 6, 7, 8);
  VLOAD_8(v2, 1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U8(9, v3, -3);

  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_16(v2, -9);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U16(10, v3, -9);

  VSET(16, e32, m1);
  VLOAD_32(v1, 9, 2, 3, 4, -5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_32(v2, 1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U32(11, v3, -5);

  VSET(16, e64, m1);
  VLOAD_64(v1, 1, 2, 3, -4, 5, 6, 7, 9, 1, 2, 3, 4, 5, 6, 7, 8);
  VLOAD_64(v2, -1);
  asm volatile("vredmaxu.vs v3, v1, v2");
  VCMP_U64(12, v3, -1);
}

int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_CASE1();
  TEST_CASE2();
  TEST_CASE3();

  EXIT_CHECK();
}
