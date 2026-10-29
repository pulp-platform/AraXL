// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//         Basile Bougenot <bbougenot@student.ethz.ch>

#include "/scratch/sem26h15/AraXL_sem_project/apps/common/macros/vector/vector_macros.h"
#include "/scratch/sem26h15/AraXL_sem_project/apps/common/macros/vector/long_array.h"
#include <stdio.h>
#include <printf.h>


int8_t mask[2] = {0xAA, 0xAA};

// }
// mem display -format hex /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[0]/i_ara_macro/i_ara/gen_lanes[0]/i_lane/i_vrf/gen_banks[0]/data_sram/sram 

//uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));

// void POPULATE_VECTOR(void) {

//   int j=0;
//   for (int i=0; i<1024; i++){
//     if (j==64){
//       j=0;
//     }
//     //LONG_I8[i]= j + 1;
//     LONG_I8_2[i]= 0;
//     j ++;
//   }
//   __asm__ volatile("fence");

// }

void TEST_RESHUFFLE(void) {
 
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  // 'Load baring banana'
  //VLOAD_8(v4, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(16, e8, m1);
  asm volatile("vadd.vv v5, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

  // check op correctness
  VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);

  // store result
  // uint64_t result[2];
  // asm volatile("vse64.v v1, (%0)" : : "r"(result));


  //VCMP_U64(1, v3, 0x201E1C1A18161412, 0x302E2C2A28262422);
  //VCMP_U8(1, v1, 0x080706504030201, 0x100F0E0D0C0B0A09);
  //VCMP_U64(1, v4, 0x080706504030201, 0x100F0E0D0C0B0A09);
}

void TEST_RESHUFFLE_TEST(void) {
 
  VSET(15, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  // VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  // VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  // VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(15, e8, m1);
  asm volatile("vadd.vv v5, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

  // check op correctness
  VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

  // store result
  // uint64_t result[2];
  // asm volatile("vse64.v v1, (%0)" : : "r"(result));


  //VCMP_U64(1, v3, 0x201E1C1A18161412, 0x302E2C2A28262422);
  //VCMP_U8(1, v1, 0x080706504030201, 0x100F0E0D0C0B0A09);
  //VCMP_U64(1, v4, 0x080706504030201, 0x100F0E0D0C0B0A09);
}

void TEST_RESHUFFLE_TEST_2(void) {
 
  VSET(30, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  // VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  // VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  // VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(30, e8, m1);
  asm volatile("vadd.vv v5, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

  // check op correctness
  //VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

  // store result
  // uint64_t result[2];
  // asm volatile("vse64.v v1, (%0)" : : "r"(result));


  //VCMP_U64(1, v3, 0x201E1C1A18161412, 0x302E2C2A28262422);
  //VCMP_U8(1, v1, 0x080706504030201, 0x100F0E0D0C0B0A09);
  //VCMP_U64(1, v4, 0x080706504030201, 0x100F0E0D0C0B0A09);
}

void TEST_RESHUFFLE_2(void) {
 
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(8, e16, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_16(v3, 0, 0, 0, 0, 0, 0, 0, 0);

  asm volatile("vadd.vv v3, v1, v2");
  
  VCMP_U16(1, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VCMP_U16(2, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);



}

void TEST_RESHUFFLE_3(void) {
 
  VSET(8, e16, m1);
  VLOAD_16(v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VLOAD_16(v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v3, v1, v2");
  VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);

  VSET(8, e16, m1);
  VLOAD_16(v3, 0, 0, 0, 0, 0, 0, 0, 0);
  asm volatile("vadd.vv v3, v1, v2"); 
  VCMP_U16(2, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  VCMP_U16(3, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);
  
  //VCMP_U16(1, v1, 0x0201,0x0403 , 0x0605 ,0x0807 , 0x0A09 , 0x0C0B, 0x0E0D, 0x100F);
  //VCMP_U16(2, v2, 0x0101,0x0101 , 0x0101 ,0x0101 , 0x0101 , 0x0101, 0x0101, 0x0101);

}

void TEST_RESHUFFLE_4(void) {

  VSET(1024, e8, m1);

  //uint8_t LONG_I8[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));

    int j=0;
    for (int i=0; i<1024; i++){
      if (j==64){
        j=0;
      }
      //LONG_I8[i]= j + 1;
      LONG_I8_2[i]= j;
      j ++;
    }
    __asm__ volatile("fence");
  

 
  
  asm volatile("vle8.v v1, (%0)" ::"r"(&LONG_I8[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_2[0]));
  

  //VSET(512, e16, m1);
  asm volatile("vadd.vv v4, v1, v2");

  // VSET(1024, e8, m1);
  // asm volatile("vle8.v v3, (%0)" ::"r"(&LONG_I8_2[0]));
  // asm volatile("vadd.vv v3, v1, v2");


  LVCMP_U8(15, v1, LONG_I8);



}

void TEST_RESHUFFLE_5(void) {
 
  VSET(30, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VCMP_U8(55, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  // VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  // VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  // VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(30, e8, m1);
  asm volatile("vadd.vv v4, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

  // check op correctness
  //VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

  // store result
  // uint64_t result[2];
  // asm volatile("vse64.v v1, (%0)" : : "r"(result));


  //VCMP_U64(1, v3, 0x201E1C1A18161412, 0x302E2C2A28262422);
  //VCMP_U8(1, v1, 0x080706504030201, 0x100F0E0D0C0B0A09);
  //VCMP_U64(1, v4, 0x080706504030201, 0x100F0E0D0C0B0A09);
}

void TEST_RESHUFFLE_6(void) {

  VSET(1024, e8, m1);

  //uint8_t LONG_I8[2048] __attribute__((aligned(128)));
  //uint8_t LONG_I8_2[2048] __attribute__((aligned(128)));
  // int j=0;
  // for (int i=0; i<2048; i++){
  //   LONG_I8_2[i]=3;
  //   if (i<1024){
  //     LONG_I8[i]=1;
  //   } else {
  //     LONG_I8[i]=2;
  //   }
  // }

 
  
  asm volatile("vle8.v v0, (%0)" ::"r"(&LONG_I8[0]));
  LVCMP_U8(55, v0, LONG_I8);
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8[0]));
  LVCMP_U8(55, v2, LONG_I8);

  asm volatile("vadd.vv v4, v0, v2");

  LVCMP_U8(1, v0, LONG_I8);
  

  VSET(512, e16, m1);
  asm volatile("vadd.vv v4, v0, v2");

  // Trigger error
  //LONG_I8[0]=5;

  VSET(1024, e8, m1);
  //asm volatile("vle8.v v4, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vadd.vv v4, v0, v2");


  LVCMP_U8(2, v0, LONG_I8);



}

void TEST_RESHUFFLE_7(void) {

  VSET(1024, e8, m1);

  //uint8_t LONG_I8[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_3[1024] __attribute__((aligned(128)));
  int j=0;
  for (int i=0; i<1024; i++){
    LONG_I8_2[i]=3;
    if (j==128){
      j = 0;
    }
    LONG_I8_3[i]=j;

    j++;
    
  }
  asm volatile("vle8.v v4, (%0)" ::"r"(&LONG_I8[0]));

  asm volatile("vle8.v v0, (%0)" ::"r"(&LONG_I8_3[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_2[0]));

  asm volatile("vadd.vv v4, v0, v2");

  LVCMP_U8(67, v0, LONG_I8_3);
  //asm volatile("fence");



}

void TEST_RESHUFFLE_8(void) {

  VSET(1024, e8, m2);

  
  uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));
    int j=0;
    for (int i=0; i<1024; i++){
      if (j==64){
        j=0;
      }
      //LONG_I8[i]= j + 1;
      LONG_I8_2[i]= 1;
      j ++;
    }
    __asm__ volatile("fence");
  
  asm volatile("vle8.v v0, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_2[0]));

  asm volatile("vadd.vv v4, v0, v2");

  LVCMP_U8(1, v0, LONG_I8_2);
  

  VSET(512, e16, m2);
  asm volatile("vadd.vv v4, v0, v2");

  // Trigger error
  //LONG_I8[0]=5;

  VSET(1024, e8, m2);
  //asm volatile("vle8.v v4, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vadd.vv v4, v0, v2");


  LVCMP_U8(2, v0, LONG_I8_2);



}

void TEST_RESHUFFLE_9(void) {

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
    LONG_I8_2[i]= 0x02;
    LONG_I8_3[i]= 0x03;
    LONG_I8_4[i]= 0x05;

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

  LVCMP_U8(2, v0, LONG_I8_2);
  LVCMP_U8(2, v2, LONG_I8_3);

  asm volatile("vadd.vv v4, v0, v2");

  LVCMP_U8(1, v4, LONG_I8_4);
  

  VSET(512, e16, m2);
  asm volatile("vadd.vv v6, v0, v2");

  LVCMP_U16(2, v0, LONG_I16_5);
  LVCMP_U16(2, v2, LONG_I16_6);
  LVCMP_U16(2, v6, LONG_I16_7);

  VSET(1024, e8, m2);
  //asm volatile("vle8.v v4, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vadd.vv v4, v0, v2");


  LVCMP_U8(2, v0, LONG_I8_2);
  LVCMP_U8(2, v2, LONG_I8_3);
  LVCMP_U8(2, v4, LONG_I8_4);

}

void TEST_RESHUFFLE_10(void) {

  
  uint16_t IDX_A[512]  __attribute__((aligned(128)));
  uint16_t IDX_B[512]  __attribute__((aligned(128)));
  uint16_t IDX_SUM[512] __attribute__((aligned(128)));

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
  __asm__ volatile("fence");

  // store result
  uint16_t rs_buffer[512];
  

  VSET(512, e16, m2);
  asm volatile("vle16.v v0, (%0)" ::"r"(&IDX_A[0]));
  asm volatile("vle16.v v2, (%0)" ::"r"(&IDX_B[0]));
  //asm volatile("vle16.v v4, (%0)" ::"r"(&IDX_B[0]));
  asm volatile("vadd.vv v4, v0, v2");

  

  //LVCMP_U16(1, v0, IDX_A);
  //LVCMP_U16(2, v2, IDX_B);
  //LVCMP_U16(3, v4, IDX_SUM);
  for (int i=0; i<10; i++) {
    printf("BUFFER index %d : %#08x \r\n",i,IDX_SUM8[i]);

  }
  asm volatile("vse16.v v4, (%0)" : : "r"(rs_buffer));

  // FENCE IS NEEDED
  //__asm__ volatile("fence");

  VSET(1024, e8, m2);
  
  //asm volatile("vle8.v v4, (%0)" : : "r"(rs_buffer));
  //asm volatile("vadd.vv v2, v0, v4");
  //LVCMP_U8(4, v0, IDX_A8);
  //LVCMP_U8(5, v4, IDX_SUM8);
  //LVCMP_U8(6, v2, IDX_SUM8_2);
  
  //VSET(512, e16, m2);
  //asm volatile("vadd.vv v6, v0, v2");
  //LVCMP_U16(7, v0, IDX_A);
}


void TEST_RESHUFFLE_11(void) {
 
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  //VLOAD_8(v4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(16, e8, m1);
  asm volatile("vadd.vv v5, v4, v2"); 
  VCMP_U8(4, v4, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);
  VSET(8, e16, m1);
  asm volatile("vadd.vv v5, v4, v2"); 
  VSET(4, e32, m1);
  asm volatile("vadd.vv v5, v4, v2");
  VSET(2, e64, m1);
  asm volatile("vadd.vv v5, v4, v2");
  VSET(16, e8, m1);
  asm volatile("vadd.vv v5, v4, v2"); 
  VCMP_U8(4, v4, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17);


}



int main(void) {
  INIT_CHECK();
  enable_vec();
  //POPULATE_VECTOR();

  TEST_RESHUFFLE();
  TEST_RESHUFFLE_TEST();
  TEST_RESHUFFLE_2();
  TEST_RESHUFFLE_3();

  
  TEST_RESHUFFLE_4();
  TEST_RESHUFFLE_5();
  TEST_RESHUFFLE_TEST();
  TEST_RESHUFFLE_6();
  TEST_RESHUFFLE_7();
  
  TEST_RESHUFFLE_8();
  TEST_RESHUFFLE_9();
  TEST_RESHUFFLE_10();
  TEST_RESHUFFLE_11();
  


  EXIT_CHECK();
}