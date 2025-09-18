// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Module to fork and cut the request interface.
// Helps pipeline the request interface from CVA6 to ARA Clusters

module req_fork_cut import ara_pkg::*; import rvv_pkg::*; #(
  parameter int unsigned NrCuts = 1
) (
  input logic clk_i, 
  input logic rst_ni,

  input accelerator_req_t   req_i,
  output accelerator_resp_t resp_o,

  output accelerator_req_t [1:0] req_o,
  input accelerator_resp_t [1:0] resp_i
);

  logic acc_req_valid_i, acc_req_ready_o;
  logic [1:0] acc_req_valid_o, acc_req_ready_i;

  accelerator_req_t  [1:0] req_cut_i, req_cut_o;
  accelerator_resp_t [1:0] resp_cut_i, resp_cut_o;
  
  // From CVA6
  assign acc_req_valid_i = req_i.req_valid;
  for (genvar i=0; i<2; i++) begin
    assign acc_req_ready_i[i] = resp_cut_o[i].req_ready;
  end

  stream_fork #(
    .N_OUP(2)
  ) i_request_fork (
    .clk_i  (clk_i            ),
    .rst_ni (rst_ni           ),
    
    // To CVA6
    .valid_i(acc_req_valid_i  ),
    .ready_o(acc_req_ready_o  ),
    
    // To Clusters
    .valid_o(acc_req_valid_o    ),
    .ready_i(acc_req_ready_i    )
  );

  for (genvar i=0; i<2; i++) begin
    if (NrCuts > 0) begin
      // Cut after a fork
      cva6_cut # (
        .NrCuts      (NrCuts           )
      ) i_cva6_macro_cut (
        .clk_i       (clk_i            ), 
        .rst_ni      (rst_ni           ), 

        .acc_req_i   (req_cut_i[i]     ),
        .acc_resp_o  (resp_cut_o[i]    ),

        .acc_req_o   (req_cut_o[i]     ),
        .acc_resp_i  (resp_cut_i[i]    )
      );
    end else begin 
      assign req_cut_o[i] = req_cut_i[i];
      assign resp_cut_o[i] = resp_cut_i[i];
    end

    assign req_o[i] = req_cut_o[i];
    assign resp_cut_i[i] = resp_i[i];
  end

  always_comb begin
    for (int i=0; i<2; i++) begin
      req_cut_i[i] = req_i;
      req_cut_i[i].req_valid = acc_req_valid_o[i];
    end

    resp_o = resp_cut_o[0];
    resp_o.req_ready = acc_req_ready_o;
  end

endmodule 
