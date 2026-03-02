// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matteo Perotti <mperotti@iis.ee.ethz.ch>

#include "vector_macros.h"

// Naive test
void TEST_CASE1(void) {
  VSET(12, e8, m1);
  VLOAD_8(v1, 0xff, 0xf1, 0xf0, 0xff, 0xf1, 0xf0, 0xff, 0xf1, 0xf0, 0xff, 0xf1,
          0xf0);
  VLOAD_8(v2, 0xf0);
  asm volatile("vredand.vs v3, v1, v2");
  VCMP_U8(1, v3, 0xf0);

  VSET(12, e16, m1);
  VLOAD_16(v1, 0xffff, 0x0301, 0xf1f0, 0xffff, 0x0101, 0xf7f0, 0xffff, 0x0701,
           0xfff0, 0xffff, 0x0101, 0xf1f0);
  VLOAD_16(v2, 0xefff);
  asm volatile("vredand.vs v3, v1, v2");
  VCMP_U16(2, v3, 0x0100);

  VSET(12, e32, m1);
  VLOAD_32(v1, 0xffffffff, 0x100ff001, 0xf0f0f0f0, 0xffffffff, 0x100ff001,
           0xf0f0f0f0, 0xffffffff, 0x100ff001, 0xf0f0f0f0, 0xffffffff,
           0x100ff001, 0xf0f0f0f0);
  VLOAD_32(v2, 0x00f010f0);
  asm volatile("vredand.vs v3, v1, v2");
  VCMP_U32(3, v3, 0x00001000);

  VSET(12, e64, m1);
  VLOAD_64(v1, 0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0);
  VLOAD_64(v2, 0xfffffffffffffff7);
  asm volatile("vredand.vs v3, v1, v2");
  VCMP_U64(4, v3, 0x1000000000000000);
}

// Masked naive test
void TEST_CASE2(void) {
  uint8_t mask[2] = {0xf7, 0xff};
  VSET(12, e8, m1);
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_8(v1, 0xff, 0xf1, 0xff, 0x00, 0xf1, 0xf0, 0xff, 0xf1, 0xf0, 0xff, 0xf1,
          0xf0);
  VLOAD_8(v2, 0xf0);
  VLOAD_8(v3, 1);
  asm volatile("vredand.vs v3, v1, v2, v0.t");
  VCMP_U8(5, v3, 0xf0);

  VSET(12, e16, m1);
  mask[0] = 0x00;
  mask[1] = 0x08;
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_16(v1, 0xffff, 0x0301, 0xf1f0, 0xffff, 0x0101, 0xf7f0, 0xffff, 0x9701,
           0xfff0, 0xffff, 0x0101, 0xf1f0);
  VLOAD_16(v2, 0xefff);
  VLOAD_16(v3, 1);
  asm volatile("vredand.vs v3, v1, v2, v0.t");
  VCMP_U16(6, v3, 0xe1f0);

  VSET(12, e32, m1);
  mask[0] = 0xfe;
  mask[1] = 0xff;
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_32(v1, 0x00000000, 0x100ff001, 0xf0f0f0f0, 0xffffffff, 0x100ff001,
           0xf0f0f0f0, 0xffffffff, 0x100ff001, 0xf0f0f0f0, 0xffffffff,
           0x100ff001, 0xf0f0f0f0);
  VLOAD_32(v2, 0x00f010f0);
  VLOAD_32(v3, 1);
  asm volatile("vredand.vs v3, v1, v2, v0.t");
  VCMP_U32(7, v3, 0x00001000);

  VSET(12, e64, m1);
  mask[0] = 0xaa;
  mask[1] = 0x55;
  asm volatile ("vlm.v v0, (%0)"::"r"(mask));
  VLOAD_64(v1, 0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0,
           0xffffffffffffffff, 0x1000000000000001, 0xf0f0f0f0f0f0f0f0);
  VLOAD_64(v2, 0xfffffffffffffff7);
  VLOAD_64(v3, 1);
  asm volatile("vredand.vs v3, v1, v2, v0.t");
  VCMP_U64(8, v3, 0x1000000000000000);
}

int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_CASE1();
  TEST_CASE2();

  EXIT_CHECK();
}
