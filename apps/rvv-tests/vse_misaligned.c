// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Luca Caballero Cusin <lcaballero@ethz.ch>

#include "vector_macros.h"
#include "long_array.h"

#define AXI_DWIDTH 128

#define INIT 98


static volatile uint16_t B1[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint8_t B2[96]     __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 95] = INIT};
static volatile uint8_t B3[96]     __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 95] = INIT};
static volatile uint8_t B4[128]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 127] = INIT};
static volatile uint16_t B5[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint16_t B6[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint16_t B7[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint32_t B8[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint32_t B9[48]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint64_t B10[24]   __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 23] = INIT};
static volatile uint64_t B11[24]   __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 23] = INIT};
static volatile uint16_t B12[96]   __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 95] = INIT};
static volatile uint16_t B13[256]  __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 255] = INIT};
static volatile uint16_t B14[4096] __attribute__((aligned(4096))) = {[0 ... 4095] = INIT};
static volatile uint8_t B15[8192]  __attribute__((aligned(4096))) = {[0 ... 8191] = INIT};
static volatile uint8_t B16[4096]  __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 4095] = INIT};
static volatile uint8_t S16[2048]  __attribute__((aligned(AXI_DWIDTH)));
static volatile uint8_t F16 = 0;
static volatile uint16_t B17[48]   __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint8_t M17[8]     __attribute__((aligned(AXI_DWIDTH))) = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
static volatile uint8_t M19[8]     __attribute__((aligned(AXI_DWIDTH))) = {0x11, 0x22, 0x33, 0x44, 0x55, 0, 0, 0};
static volatile uint16_t B18[48]   __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 47] = INIT};
static volatile uint8_t B19[96]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 95] = INIT};
static volatile uint8_t B20[16384] __attribute__((aligned(4096))) = {[0 ... 16383] = INIT};
static volatile uint8_t S20[8192]  __attribute__((aligned(AXI_DWIDTH)));
static volatile uint8_t F20 = 0;
static volatile uint16_t B21[128] __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 127] = INIT};
static volatile uint16_t B22[128] __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 127] = INIT};
static volatile uint8_t F21 = 0;
static volatile uint8_t B23[8192]    __attribute__((aligned(4096))) = {[0 ... 8191] = INIT};
static volatile uint8_t MASK_SRC[16] __attribute__((aligned(AXI_DWIDTH))) = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static volatile uint16_t D21[16]     __attribute__((aligned(AXI_DWIDTH))) = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
static volatile uint16_t X21[16]     __attribute__((aligned(AXI_DWIDTH))) = {0,6,12,18,24,30,36,42,48,54,60,66,72,78,84,90};
static volatile uint16_t B24[256]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 255] = INIT};
static volatile uint16_t B25[256]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 255] = INIT};
static volatile uint16_t B26[256]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 255] = INIT};
static volatile uint16_t B27[256]    __attribute__((aligned(AXI_DWIDTH))) = {[0 ... 255] = INIT};


void reset_vec8(volatile uint8_t *vec, int rst_val, uint64_t len) {
  for (uint64_t i = 0; i < len; ++i)
    vec[i] = rst_val;
}


// e16, offset 0, 32 bytes
void TEST_CASE1(void) {
  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B1[16]));
  volatile uint16_t *r1 = &B1[15];
  VVCMP_U16(1, r1, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
}

// e8, offset 1, 32 bytes: one full beat plus 1 spilled byte
void TEST_CASE2(void) {
  VSET(32, e8, m1);
  VLOAD_8(v2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  asm volatile("vse8.v v2, (%0)" ::"r"(&B2[33]));
  volatile uint8_t *r2 = &B2[32];
  VVCMP_U8(2, r2, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, INIT);
}

// e8, offset 31, 1 byte: last lane of the beat, no spill
void TEST_CASE3(void) {
  VSET(1, e8, m1);
  VLOAD_8(v3, 7);
  asm volatile("vse8.v v3, (%0)" ::"r"(&B3[63]));
  volatile uint8_t *r3 = &B3[62];
  VVCMP_U8(3, r3, INIT, 7, INIT);
}

// e8 m2, offset 17, 40 bytes: 2 beats, partial last beat, no spill
void TEST_CASE4(void) {
  VSET(40, e8, m2);
  VLOAD_8(v24, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40);
  asm volatile("vse8.v v24, (%0)" ::"r"(&B4[49]));
  volatile uint8_t *r4 = &B4[48];
  VVCMP_U8(4, r4, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, INIT);
}

// e16, offset 2, 32 bytes: basic spill case
void TEST_CASE5(void) {
  VSET(16, e16, m1); 
  VLOAD_16(v5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v5, (%0)" ::"r"(&B5[17]));
  volatile uint16_t *r5 = &B5[16];
  VVCMP_U16(5, r5, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
}

// e16, offset 30, 2 elements: only the first element fits in the beat
void TEST_CASE6(void) {
  VSET(2, e16, m1);
  VLOAD_16(v6, 1, 2);
  asm volatile("vse16.v v6, (%0)" ::"r"(&B6[31]));
  volatile uint16_t *r6 = &B6[30];
  VVCMP_U16(6, r6, INIT, 1, 2, INIT);
}

// e16, offset 2, 30 bytes: ends exactly on the beat boundary
void TEST_CASE7(void) {
  VSET(15, e16, m1);
  VLOAD_16(v7, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  asm volatile("vse16.v v7, (%0)" ::"r"(&B7[17]));
  volatile uint16_t *r7 = &B7[16];
  VVCMP_U16(7, r7, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, INIT);
}

// e32, offset 4, 32 bytes
void TEST_CASE8(void) {
  VSET(8, e32, m1);
  VLOAD_32(v8, 1, 2, 3, 4, 5, 6, 7, 8);
  asm volatile("vse32.v v8, (%0)" ::"r"(&B8[9]));
  volatile uint32_t *r8 = &B8[8];
  VVCMP_U32(8, r8, INIT, 1, 2, 3, 4, 5, 6, 7, 8, INIT);
}

// e32, offset 28, 12 bytes: one element in the first beat, two spilled
void TEST_CASE9(void) {
  VSET(3, e32, m1);
  VLOAD_32(v9, 1, 2, 3);
  asm volatile("vse32.v v9, (%0)" ::"r"(&B9[15]));
  volatile uint32_t *r9 = &B9[14];
  VVCMP_U32(9, r9, INIT, 1, 2, 3, INIT);
}

// e64, offset 8, 32 bytes
void TEST_CASE10(void) {
  VSET(4, e64, m1);
  VLOAD_64(v10, 1, 2, 3, 4);
  asm volatile("vse64.v v10, (%0)" ::"r"(&B10[5]));
  volatile uint64_t *r10 = &B10[4];
  VVCMP_U64(10, r10, INIT, 1, 2, 3, 4, INIT);
}

// e64, offset 24, 1 element: ends exactly at the end of the beat
void TEST_CASE11(void) {
  VSET(1, e64, m1);
  VLOAD_64(v11, 1);
  asm volatile("vse64.v v11, (%0)" ::"r"(&B11[7]));
  volatile uint64_t *r11 = &B11[6];
  VVCMP_U64(11, r11, INIT, 1, INIT);
}

// e16 m2, offset 2, 64 bytes: multi-beat with a flush beat
void TEST_CASE12(void) {
  VSET(32, e16, m2);
  VLOAD_16(v12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  asm volatile("vse16.v v12, (%0)" ::"r"(&B12[33]));
  volatile uint16_t *r12 = &B12[32];
  VVCMP_U16(12, r12, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, INIT);
}

// Back to back stores with flush, no flush, aligned, flush at max offset
void TEST_CASE13(void) {
  VSET(16, e16, m1);
  VLOAD_16(v16, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v16, (%0)" ::"r"(&B13[17]));
  VSET(15, e16, m1);
  VLOAD_16(v17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  asm volatile("vse16.v v17, (%0)" ::"r"(&B13[65]));
  VSET(16, e16, m1);
  VLOAD_16(v18, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v18, (%0)" ::"r"(&B13[112]));
  VSET(2, e16, m1);
  VLOAD_16(v19, 1, 2);
  asm volatile("vse16.v v19, (%0)" ::"r"(&B13[175]));
  volatile uint16_t *r13a = &B13[16];
  volatile uint16_t *r13b = &B13[64];
  volatile uint16_t *r13c = &B13[111];
  volatile uint16_t *r13d = &B13[174];
  VVCMP_U16(13, r13a, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(14, r13b, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, INIT);
  VVCMP_U16(15, r13c, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(16, r13d, INIT, 1, 2, INIT);
}

// Page crossing: e16 m2, starts 34 bytes before the boundary (offset 30)
void TEST_CASE14(void) {
  VSET(32, e16, m2);
  VLOAD_16(v20, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);
  asm volatile("vse16.v v20, (%0)" ::"r"(&B14[2031]));
  volatile uint16_t *r14 = &B14[2030];
  VVCMP_U16(17, r14, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, INIT);
}

// Page crossing: e8 m2, starts 1 byte before the boundary (offset 31)
void TEST_CASE15(void) {
  VSET(40, e8, m2);
  VLOAD_8(v22, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40);
  asm volatile("vse8.v v22, (%0)" ::"r"(&B15[4095]));
  volatile uint8_t *r15 = &B15[4094];
  VVCMP_U8(18, r15, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, INIT);
}



// vs1r.v of a full register, offset 3 bytes. The size is read from vlenb at run time.
void TEST_CASE16(void) {
  uint64_t vlenb;
  asm volatile("csrr %0, vlenb" : "=r"(vlenb));
  for (uint64_t i = 0; i < vlenb; i++) S16[i] = (uint8_t)(i * 7 + 1);
  asm volatile("vsetvli zero, %0, e8, m1, ta, ma" ::"r"(vlenb));
  asm volatile("vle8.v v1, (%0)" ::"r"(&S16[0]) : "memory");
  asm volatile("vs1r.v v1, (%0)" ::"r"(&B16[64 + 3]) : "memory");
  F16 = 0;
  if (B16[66] != INIT || B16[67 + vlenb] != INIT) F16 = 1;
  for (uint64_t i = 0; i < vlenb; i++)
    if (B16[67 + i] != S16[i]) F16 = 1;
  volatile uint8_t *r16 = &F16;
  VVCMP_U8(19, r16, 0);
}

// e16, offset 2, 16 elements, mask 0x55 0x55: even elements stored, odd ones untouched.
// The last element (15) is masked, so the flush beat has zero strobes but must still be sent.
// e16, offset 2, 16 elements, mask 0x55: even-indexed elements stored, odd ones untouched
void TEST_CASE17(void) {
  VSET(16, e16, m1);
  asm volatile("vlm.v v0, (%0)" ::"r"(&M17[0]) : "memory");
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v1, (%0), v0.t" ::"r"(&B17[17]));
  volatile uint16_t *r17 = &B17[16];
  VVCMP_U16(20, r17, INIT, 1, INIT, 3, INIT, 5, INIT, 7, INIT, 9, INIT, 11, INIT,
            13, INIT, 15, INIT, INIT);
}

// vl = 0 at a misaligned address: nothing may be written and nothing may hang
void TEST_CASE18(void) {
  VSET(0, e16, m1);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B18[17]));
  volatile uint16_t *r18 = &B18[16];
  VVCMP_U16(21, r18, INIT, INIT, INIT, INIT);
}

// vsm.v with vl = 40 (5 bytes), offset 30: the 5 bytes spill into a flush beat
void TEST_CASE19(void) {
  VSET(40, e8, m1);
  asm volatile("vlm.v v1, (%0)" ::"r"(&M19[0]) : "memory");
  asm volatile("vsm.v v1, (%0)" ::"r"(&B19[62]) : "memory");
  volatile uint8_t *r19 = &B19[61];
  VVCMP_U8(22, r19, INIT, 0x11, 0x22, 0x33, 0x44, 0x55, INIT);
}

// e8 m8, maximum vl (up to 8192 bytes), offset 5: three or more AWs, spill carried across all of them
void TEST_CASE20(void) {
  uint64_t vl;
  asm volatile("vsetvli %0, %1, e8, m8, ta, ma" : "=r"(vl) : "r"(8192));
  for (uint64_t i = 0; i < 8192; i++) S20[i] = (uint8_t)(i * 7 + 1);
  asm volatile("vle8.v v8, (%0)" ::"r"(&S20[0]) : "memory");
  asm volatile("vse8.v v8, (%0)" ::"r"(&B20[4096 + 5]) : "memory");
  F20 = 0;
  if (B20[4100] != INIT || B20[4101 + vl] != INIT) F20 = 1;
  for (uint64_t i = 0; i < vl; i++)
    if (B20[4101 + i] != S20[i]) F20 = 1;
  volatile uint8_t *r20 = &F20;
  VVCMP_U8(23, r20, 0);
}

// TODO : Test currently fails. Possible issue with vsuxei
// Regression: indexed store (vsuxei16) only, element k at slot 32 + 3k
void TEST_CASE21(void) {
  VSET(16, e16, m1);
  VLOAD_16(v23, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v24, 0, 6, 12, 18, 24, 30, 36, 42, 48, 54, 60, 66, 72, 78, 84, 90);
  asm volatile("vsuxei16.v v23, (%0), v24" ::"r"(&B21[32]));
  F21 = 0;
  for (int i = 0; i < 128; i++) {
    int rel = i - 32;
    uint16_t exp = INIT;
    if (rel >= 0 && rel % 3 == 0 && rel / 3 < 16) exp = (uint16_t)(rel / 3 + 1);
    if (B21[i] != exp && F21 == 0) F21 = (uint16_t)(i + 1);   // first bad slot, +1
  }
  volatile uint16_t *r21 = &F21;
  VVCMP_U16(24, r21, 0);
}

void TEST_CASE22(void) {
  // e64 vl=2, starts 8 bytes before the boundary: element 0 in AW1, element 1 in the flush-only AW2
  reset_vec8(&B23[4096 - 64], INIT, 128);
  VSET(2, e64, m1);
  VLOAD_64(v22, 0x0807060504030201, 0x100f0e0d0c0b0a09);
  asm volatile("vse64.v v22, (%0)" ::"r"(&B23[4096 - 8]));
  volatile uint8_t *r22 = &B23[4096 - 9];
  VVCMP_U8(25, r22, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
}

// vl=100 mask bits -> 13 bytes, off+13>32, page crossing
void TEST_CASE23(void) {
  // k=1: off=31, 1 byte before the boundary, 12 after
  reset_vec8(&B23[4096 - 64], INIT, 128);
  VSET(100, e8, m1);
  asm volatile("vlm.v v1, (%0)" ::"r"(&MASK_SRC[0]));
  asm volatile("vsm.v v1, (%0)" ::"r"(&B23[4096 - 1]));
  volatile uint8_t *r23a = &B23[4096 - 2];
  VVCMP_U8(28, r23a, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, INIT);

  // k=5: 5 bytes before, 8 after
  reset_vec8(&B23[4096 - 64], INIT, 128);
  VSET(100, e8, m1);
  asm volatile("vlm.v v2, (%0)" ::"r"(&MASK_SRC[0]));
  asm volatile("vsm.v v2, (%0)" ::"r"(&B23[4096 - 5]));
  volatile uint8_t *r23b = &B23[4096 - 6];
  VVCMP_U8(29, r23b, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, INIT);

  // k=12: 12 bytes before, 1 after
  reset_vec8(&B23[4096 - 64], INIT, 128);
  VSET(100, e8, m1);
  asm volatile("vlm.v v3, (%0)" ::"r"(&MASK_SRC[0]));
  asm volatile("vsm.v v3, (%0)" ::"r"(&B23[4096 - 12]));
  volatile uint8_t *r23c = &B23[4096 - 13];
  VVCMP_U8(30, r23c, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, INIT);
}

// Regression: strided store (vsse16) only, element k at slot 32 + 3k
void TEST_CASE24(void) {
  VSET(16, e16, m1);
  VLOAD_16(v23, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  long stride = 6;
  asm volatile("vsse16.v v23, (%0), %1" ::"r"(&B22[32]), "r"(stride));
  F21 = 0;
  for (int i = 0; i < 128; i++) {
    int rel = i - 32;
    uint16_t exp = INIT;
    if (rel >= 0 && rel % 3 == 0 && rel / 3 < 16) exp = (uint16_t)(rel / 3 + 1);
    if (B22[i] != exp && F21 == 0) F21 = (uint8_t)(i + 1);   // first bad slot, +1
  }
  volatile uint8_t *r21 = &F21;
  VVCMP_U8(31, r21, 0);
}


 
// Back to back e16 stores, misaligned -> aligned: three pairs with vl 16, 14, 17 (one VSET per pair, a beat is 16 elements)
void TEST_CASE25(void) {
  // pair 1: A vl=16 @ elem 17 (off 2B), B vl=14 @ elem 48 (off 0B)
  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B24[17]));
  VSET(14, e16, m1);
  VLOAD_16(v2, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64);
  asm volatile("vse16.v v2, (%0)" ::"r"(&B24[48]));
  // pair 2: A vl=14 @ elem 87 (off 14B), B vl=17 @ elem 112 (off 0B)
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
  asm volatile("vse16.v v3, (%0)" ::"r"(&B24[87]));
  VSET(17, e16, m1);
  VLOAD_16(v4, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67);
  asm volatile("vse16.v v4, (%0)" ::"r"(&B24[112]));
  // pair 3: A vl=17 @ elem 157 (off 26B), B vl=16 @ elem 176 (off 0B)
  VSET(17, e16, m1);
  VLOAD_16(v5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
  asm volatile("vse16.v v5, (%0)" ::"r"(&B24[157]));
  VSET(16, e16, m1);
  VLOAD_16(v6, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66);
  asm volatile("vse16.v v6, (%0)" ::"r"(&B24[176]));

  volatile uint16_t *r25a1 = &B24[16];
  volatile uint16_t *r25b1 = &B24[47];
  volatile uint16_t *r25a2 = &B24[86];
  volatile uint16_t *r25b2 = &B24[111];
  volatile uint16_t *r25a3 = &B24[156];
  volatile uint16_t *r25b3 = &B24[175];
  VVCMP_U16(32, r25a1, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(33, r25b1, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, INIT);
  VVCMP_U16(34, r25a2, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, INIT);
  VVCMP_U16(35, r25b2, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, INIT);
  VVCMP_U16(36, r25a3, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, INIT);
  VVCMP_U16(37, r25b3, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, INIT);
}
 
// Back to back e16 stores, aligned -> aligned: three pairs with vl 16, 14, 17 (one VSET per pair, a beat is 16 elements)
void TEST_CASE26(void) {
  // pair 1: vl=16, A @ elem 16 (off 0B), B @ elem 48 (off 0B)
  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v2, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B25[16]));
  asm volatile("vse16.v v2, (%0)" ::"r"(&B25[48]));
  // pair 2: vl=14, A @ elem 80 (off 0B), B @ elem 112 (off 0B)
  VSET(14, e16, m1);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
  VLOAD_16(v4, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64);
  asm volatile("vse16.v v3, (%0)" ::"r"(&B25[80]));
  asm volatile("vse16.v v4, (%0)" ::"r"(&B25[112]));
  // pair 3: vl=17, A @ elem 144 (off 0B), B @ elem 176 (off 0B)
  VSET(17, e16, m1);
  VLOAD_16(v5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
  VLOAD_16(v6, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67);
  asm volatile("vse16.v v5, (%0)" ::"r"(&B25[144]));
  asm volatile("vse16.v v6, (%0)" ::"r"(&B25[176]));
  volatile uint16_t *r26a1 = &B25[15];
  volatile uint16_t *r26b1 = &B25[47];
  volatile uint16_t *r26a2 = &B25[79];
  volatile uint16_t *r26b2 = &B25[111];
  volatile uint16_t *r26a3 = &B25[143];
  volatile uint16_t *r26b3 = &B25[175];
  VVCMP_U16(38, r26a1, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(39, r26b1, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, INIT);
  VVCMP_U16(40, r26a2, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, INIT);
  VVCMP_U16(41, r26b2, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, INIT);
  VVCMP_U16(42, r26a3, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, INIT);
  VVCMP_U16(43, r26b3, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, INIT);
}
 
// Back to back e16 stores, aligned -> misaligned: three pairs with vl 16, 14, 17 (one VSET per pair, a beat is 16 elements)
void TEST_CASE27(void) {
  // pair 1: vl=16, A @ elem 16 (off 0B), B @ elem 49 (off 2B)
  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v2, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B26[16]));
  asm volatile("vse16.v v2, (%0)" ::"r"(&B26[49]));
  // pair 2: vl=14, A @ elem 80 (off 0B), B @ elem 121 (off 18B)
  VSET(14, e16, m1);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
  VLOAD_16(v4, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64);
  asm volatile("vse16.v v3, (%0)" ::"r"(&B26[80]));
  asm volatile("vse16.v v4, (%0)" ::"r"(&B26[121]));
  // pair 3: vl=17, A @ elem 144 (off 0B), B @ elem 181 (off 10B)
  VSET(17, e16, m1);
  VLOAD_16(v5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
  VLOAD_16(v6, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67);
  asm volatile("vse16.v v5, (%0)" ::"r"(&B26[144]));
  asm volatile("vse16.v v6, (%0)" ::"r"(&B26[181]));
  volatile uint16_t *r27a1 = &B26[15];
  volatile uint16_t *r27b1 = &B26[48];
  volatile uint16_t *r27a2 = &B26[79];
  volatile uint16_t *r27b2 = &B26[120];
  volatile uint16_t *r27a3 = &B26[143];
  volatile uint16_t *r27b3 = &B26[180];
  VVCMP_U16(44, r27a1, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(45, r27b1, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, INIT);
  VVCMP_U16(46, r27a2, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, INIT);
  VVCMP_U16(47, r27b2, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, INIT);
  VVCMP_U16(48, r27a3, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, INIT);
  VVCMP_U16(49, r27b3, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, INIT);
}
 
// Back to back e16 stores, misaligned -> misaligned: three pairs with vl 16, 14, 17 (one VSET per pair, a beat is 16 elements)
void TEST_CASE28(void) {
  // pair 1: vl=16, A @ elem 19 (off 6B), B @ elem 53 (off 10B)
  VSET(16, e16, m1);
  VLOAD_16(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_16(v2, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66);
  asm volatile("vse16.v v1, (%0)" ::"r"(&B27[19]));
  asm volatile("vse16.v v2, (%0)" ::"r"(&B27[53]));
  // pair 2: vl=14, A @ elem 91 (off 22B), B @ elem 113 (off 2B)
  VSET(14, e16, m1);
  VLOAD_16(v3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14);
  VLOAD_16(v4, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64);
  asm volatile("vse16.v v3, (%0)" ::"r"(&B27[91]));
  asm volatile("vse16.v v4, (%0)" ::"r"(&B27[113]));
  // pair 3: vl=17, A @ elem 158 (off 28B), B @ elem 179 (off 6B)
  VSET(17, e16, m1);
  VLOAD_16(v5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
  VLOAD_16(v6, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67);
  asm volatile("vse16.v v5, (%0)" ::"r"(&B27[158]));
  asm volatile("vse16.v v6, (%0)" ::"r"(&B27[179]));
  volatile uint16_t *r28a1 = &B27[18];
  volatile uint16_t *r28b1 = &B27[52];
  volatile uint16_t *r28a2 = &B27[90];
  volatile uint16_t *r28b2 = &B27[112];
  volatile uint16_t *r28a3 = &B27[157];
  volatile uint16_t *r28b3 = &B27[178];
  VVCMP_U16(50, r28a1, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, INIT);
  VVCMP_U16(51, r28b1, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, INIT);
  VVCMP_U16(52, r28a2, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, INIT);
  VVCMP_U16(53, r28b2, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, INIT);
  VVCMP_U16(54, r28a3, INIT, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, INIT);
  VVCMP_U16(55, r28b3, INIT, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, INIT);
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
  TEST_CASE10();
  TEST_CASE11();
  TEST_CASE12();
  TEST_CASE13();
  TEST_CASE14();
  TEST_CASE15();
  TEST_CASE16();
  TEST_CASE17();
  TEST_CASE18();
  TEST_CASE19();
  TEST_CASE20();
  // TEST_CASE21();
  TEST_CASE22();
  TEST_CASE23();
  TEST_CASE24();

  TEST_CASE25();
  TEST_CASE26();
  TEST_CASE27();
  TEST_CASE28();


  EXIT_CHECK();
}