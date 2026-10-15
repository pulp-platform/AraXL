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


int main(void) {
  INIT_CHECK();
  enable_vec();

  TEST_RESHUFFLE();


  EXIT_CHECK();
}
