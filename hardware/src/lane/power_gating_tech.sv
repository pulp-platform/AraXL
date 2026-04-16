// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Authors: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
// Description:
// Technology specific mapping

module power_gating_tech #(
  parameter type T = logic,
  parameter int  NO_GLITCH = 0
) (
  input  T     in_i,
  input  logic en_i,
  output T     out_o
);

  for (genvar i = 0; i < $bits(in_i); i++) begin
    tc_clk_and2 i_power_gate (
      .clk0_i (en_i    ),
      .clk1_i (in_i[i] ),
      .clk_o  (out_o[i])
    );
  end

endmodule