// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//         Basile Bougenot <bbougenot@student.ethz.ch>

#include "../common/macros/vector/vector_macros.h"
#include "../common/macros/vector/long_array.h"
#include <stdio.h>


// EW8 to EW64 to EW8
void TEST_RESHUFFLE_1(void) {
 
  // Load with eew = 8
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  // Reshuffle to eew = 64
  VSET(2, e64, m1);
  asm volatile("vadd.vv v4, v1, v2");
  
  VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  // Reshuffle back to eew = 8
  VSET(16, e8, m1);
  asm volatile("vadd.vv v5, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);

}

// EW8 to EW16 to EW8
void TEST_RESHUFFLE_2(void) {
 
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(8, e16, m1);
  asm volatile("vadd.vv v3, v1, v2");
  VCMP_U16(7, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VCMP_U16(8, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  VCMP_U16(9, v3, 0x0302,0x0504 , 0x0706 ,0x0908 , 0x0B0A , 0x0D0C, 0x0F0E, 0x1110);

  VSET(16, e8, m1);
  asm volatile("vadd.vv v3, v1, v2");
  VCMP_U8(10 ,v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VCMP_U8(11 ,v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  VCMP_U8(12 ,v3, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);

}


// EW16 to EW64 to EW16
void TEST_RESHUFFLE_3(void) {
 
  VSET(8, e16, m1);
  VLOAD_16(v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VLOAD_16(v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  asm volatile("vadd.vv v3, v1, v2");
  VCMP_U64(13, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  VCMP_U64(14, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(15, v3, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(8, e16, m1);
  asm volatile("vadd.vv v3, v1, v2"); 
  VCMP_U16(16, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VCMP_U16(17, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  VCMP_U16(18, v3, 0x0302,0x0504 , 0x0706 ,0x0908 , 0x0B0A , 0x0D0C, 0x0F0E, 0x1110);
}

// EW16 to EW32 to EW16
void TEST_RESHUFFLE_4(void) {
 
  VSET(8, e16, m1);
  VLOAD_16(v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VLOAD_16(v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(4, e32, m1);
  asm volatile("vadd.vv v3, v1, v2");
  VCMP_U32(19, v1, 0x04030201, 0x08070605, 0x0C0B0A09, 0x100F0E0D);
  VCMP_U32(20, v2, 0x01010101, 0x01010101, 0x01010101, 0x01010101);
  VCMP_U32(21, v3, 0x05040302, 0x09080706, 0x0D0C0B0A, 0x11100F0E);

  VSET(8, e16, m1);
  asm volatile("vadd.vv v3, v1, v2"); 
  VCMP_U16(22, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VCMP_U16(23, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  VCMP_U16(24, v3, 0x0302,0x0504 , 0x0706 ,0x0908 , 0x0B0A , 0x0D0C, 0x0F0E, 0x1110);
}

// Length isn't multiple of 2
void TEST_RESHUFFLE_5(void) {
 
  VSET(30, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  asm volatile("vadd.vv v4, v1, v2");
  VCMP_U64(25, v1, 0x0807060504030201, 0x010F0E0D0C0B0A09);
  VCMP_U64(26, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(27, v4, 0x0908070605040302, 0x02100F0E0D0C0B0A);

  VSET(30, e8, m1);
  asm volatile("vadd.vv v4, v1, v2"); 
  VCMP_U8(28, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VCMP_U8(29, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  VCMP_U8(30, v4, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

}

void TEST_RESHUFFLE_6(void) {

  VSET(1024, e8, m2);
  
  uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_3[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_4[1024] __attribute__((aligned(128)));

  uint16_t LONG_I16_5[512] __attribute__((aligned(128)));
  uint16_t LONG_I16_6[512] __attribute__((aligned(128)));
  uint16_t LONG_I16_7[512] __attribute__((aligned(128)));

  int j=0;

  for (int i=0; i<1024; i++){
    if (j==64){
      j=0;
    }
    //LONG_I8[i]= j + 1;
    LONG_I8_2[i]= (uint8_t)0x02;
    LONG_I8_3[i]= (uint8_t)0x03;
    LONG_I8_4[i]= (uint8_t)0x05;

    if (i<512){
      LONG_I16_5[i]= 0x0202;
      LONG_I16_6[i]= 0x0303;
      LONG_I16_7[i]= 0x0505;
    }
    j ++;
  }
  __asm__ volatile("fence");
  
  asm volatile("vle8.v v0, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_3[0]));

  asm volatile("vadd.vv v4, v0, v2");
  LVCMP_U8(1, v4, LONG_I8_4);
  

  VSET(512, e16, m2);
  asm volatile("vadd.vv v6, v0, v2");
  LVCMP_U16(2, v0, LONG_I16_5);
  LVCMP_U16(2, v2, LONG_I16_6);
  LVCMP_U16(2, v6, LONG_I16_7);

  VSET(1024, e8, m2);
  asm volatile("vadd.vv v4, v0, v2");
  LVCMP_U8(2, v0, LONG_I8_2);
  LVCMP_U8(2, v2, LONG_I8_3);
  LVCMP_U8(2, v4, LONG_I8_4);

}

// Long, index encoded vectors
void TEST_RESHUFFLE_7(void) {

  uint16_t IDX_A[512]  __attribute__((aligned(128)));
  uint16_t IDX_B[512]  __attribute__((aligned(128)));
  uint16_t IDX_SUM[512] __attribute__((aligned(128)));
  uint16_t IDX_SUM_2[512] __attribute__((aligned(128)));
  uint16_t IDX_SUM_3[512] __attribute__((aligned(128)));

  uint8_t IDX_A8[1024]  __attribute__((aligned(128)));
  uint8_t IDX_SUM8[1024] __attribute__((aligned(128)));
  uint8_t IDX_SUM8_2[1024] __attribute__((aligned(128)));

  for (int i = 0; i < 512; i++) {
    IDX_A[i]   = (uint16_t)i;
    IDX_B[i]   = (uint16_t)(3*i +1);
    IDX_SUM[i] = (uint16_t)(IDX_A[i] + IDX_B[i]);
    
  }
  for (int i = 0; i < 512; i++) {
    IDX_A8[2 * i]     = (uint8_t)(IDX_A[i] & 0xFF);
    IDX_A8[2 * i + 1] = (uint8_t)(IDX_A[i] >> 8);
  }
  for (int i = 0; i < 512; i++) {
    IDX_SUM8[2 * i]     = (uint8_t)(IDX_SUM[i] & 0xFF);
    IDX_SUM8[2 * i + 1] = (uint8_t)(IDX_SUM[i] >> 8);
  }
    for (int k = 0; k < 1024; k++) {
    IDX_SUM8_2[k] = (uint8_t)(IDX_SUM8[k] + IDX_A8[k]);
  }

  for (int i = 0; i < 512; i++) {
    uint8_t lo = (uint8_t)(IDX_SUM8[2*i]   + IDX_A8[2*i]);
    uint8_t hi = (uint8_t)(IDX_SUM8[2*i+1] + IDX_A8[2*i+1]);
    IDX_SUM_2[i] = (uint16_t)lo | ((uint16_t)hi << 8);
    IDX_SUM_3[i] = (uint16_t)(IDX_A[i] + IDX_SUM_2[i]);
  }
  __asm__ volatile("fence");

  VSET(512, e16, m2);
  asm volatile("vle16.v v0, (%0)" ::"r"(&IDX_A[0]));
  asm volatile("vle16.v v2, (%0)" ::"r"(&IDX_B[0]));
  asm volatile("vadd.vv v4, v0, v2");

  VSET(1024, e8, m2);
  asm volatile("vadd.vv v2, v0, v4");
  LVCMP_U8(4, v0, IDX_A8);
  LVCMP_U8(5, v4, IDX_SUM8);
  LVCMP_U8(6, v2, IDX_SUM8_2);
  
  VSET(512, e16, m2);
  asm volatile("vadd.vv v4, v0, v2");
  LVCMP_U16(1, v0, IDX_A);
  LVCMP_U16(2, v2, IDX_SUM_2);
  LVCMP_U16(3, v4, IDX_SUM_3);

}


int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_RESHUFFLE_1();
  TEST_RESHUFFLE_2();
  TEST_RESHUFFLE_3();
  TEST_RESHUFFLE_4();
  TEST_RESHUFFLE_5();
  TEST_RESHUFFLE_6();
  TEST_RESHUFFLE_7();

  EXIT_CHECK();
}