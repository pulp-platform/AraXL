// Copyright 2021 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
//         Basile Bougenot <bbougenot@student.ethz.ch>

#include "/scratch/sem26h15/AraXL_sem_project/apps/common/macros/vector/vector_macros.h"
#include <stdio.h>


int8_t mask[2] = {0xAA, 0xAA};

// }
// mem display -format hex /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[0]/i_ara_macro/i_ara/gen_lanes[0]/i_lane/i_vrf/gen_banks[0]/data_sram/sram 
void TEST_RESHUFFLE(void) {
 
  VSET(16, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
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

  uint8_t LONG_I8[1024] __attribute__((aligned(128)));
  uint8_t LONG_I8_2[1024] __attribute__((aligned(128)));
  int j=0;
  for (int i=0; i<1024; i++){
    if (j==64){
      j=0;
    }
    LONG_I8[i]= j + 1;
    LONG_I8_2[i]= 0;
    j ++;
  }

 
  
  asm volatile("vle8.v v1, (%0)" ::"r"(&LONG_I8[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_2[0]));
  

  VSET(512, e16, m1);
  asm volatile("vadd.vv v3, v1, v2");

  VSET(1024, e8, m1);
  asm volatile("vle8.v v3, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vadd.vv v3, v1, v2");


  LVCMP_U8(15, v1, LONG_I8);



}

void TEST_RESHUFFLE_5(void) {
 
  VSET(23, e8, m1);
  VLOAD_8(v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,18,19,20,21,22,23);
  VLOAD_8(v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1,1);
  asm volatile("vadd.vv v3, v1, v2"); 

  VSET(2, e64, m1);
  //VLOAD_64(v3, 0x080706504030201, 0x100F0E0D0C0B0A09);
  VLOAD_64(v3, 0, 0);

  asm volatile("vadd.vv v4, v1, v2");
  
  VCMP_U64(1, v1, 0x0807060504030201, 0x100F0E0D0C0B0A09);
  VCMP_U64(2, v2, 0x0101010101010101, 0x0101010101010101);
  VCMP_U64(3, v4, 0x0908070605040302, 0x11100F0E0D0C0B0A);

  VSET(23, e8, m1);
  asm volatile("vadd.vv v5, v1, v2"); 

  // check we got back to base layout
  VCMP_U8(4, v1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,18,19,20,21,22,23);
  VCMP_U8(5, v2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1,1,1,1,1,1);

  // check op correctness
  VCMP_U8(6, v5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24);

  // store result
  // uint64_t result[2];
  // asm volatile("vse64.v v1, (%0)" : : "r"(result));


  //VCMP_U64(1, v3, 0x201E1C1A18161412, 0x302E2C2A28262422);
  //VCMP_U8(1, v1, 0x080706504030201, 0x100F0E0D0C0B0A09);
  //VCMP_U64(1, v4, 0x080706504030201, 0x100F0E0D0C0B0A09);
}

void TEST_RESHUFFLE_6(void) {

  VSET(2048, e8, m2);

  uint8_t LONG_I8[2048] __attribute__((aligned(128)));
  uint8_t LONG_I8_2[2048] __attribute__((aligned(128)));
  int j=0;
  for (int i=0; i<2048; i++){
    if (j==64){
      j=0;
    }
    LONG_I8[i]= j + 1;
    LONG_I8_2[i]= 0;
    j ++;
  }

 
  
  asm volatile("vle8.v v0, (%0)" ::"r"(&LONG_I8[0]));
  asm volatile("vle8.v v2, (%0)" ::"r"(&LONG_I8_2[0]));

  asm volatile("vadd.vv v4, v0, v2");

  LVCMP_U8(15, v0, LONG_I8);
  

  VSET(1024, e16, m2);
  asm volatile("vadd.vv v4, v0, v2");

  VSET(2048, e8, m2);
  asm volatile("vle8.v v4, (%0)" ::"r"(&LONG_I8_2[0]));
  asm volatile("vadd.vv v4, v0, v2");


  LVCMP_U8(16, v0, LONG_I8);



}


int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_RESHUFFLE();
  TEST_RESHUFFLE_2();
  TEST_RESHUFFLE_3();

  TEST_RESHUFFLE_4();

  TEST_RESHUFFLE_5();


  EXIT_CHECK();
}
