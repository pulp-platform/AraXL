// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Module to cut the request interface from CVA6

module cva6_cut import ara_pkg::*; import rvv_pkg::*; #(
  parameter int unsigned NrCuts = 1
) (
    // Clock and Reset
    input  logic              clk_i,
    input  logic              rst_ni,

    // Interface with Ariane
    input  accelerator_req_t  acc_req_i,
    output accelerator_resp_t acc_resp_o,

    // Interface with ARA
    output  accelerator_req_t  acc_req_o,
    input accelerator_resp_t acc_resp_i
);
    `include "common_cells/registers.svh"

    typedef struct packed {
        accelerator_req_t  acc_req;
        accelerator_resp_t acc_resp;

        //// Request signals
        logic                                 req_valid;
        logic                                 resp_ready;
        logic                                 acc_cons_en;
        logic                                 store_pending_req;
        logic                                 inval_ready;
        //// Response signals
        logic                                 req_ready;
        logic                                 resp_valid;
        // Metadata
        logic                                 store_pending;
        logic                                 store_complete;
        logic                                 load_complete;
        logic [4:0]                           fflags;
        logic                                 fflags_valid;
        // Invalidation interface
        logic                                 inval_valid;
        logic [63:0]                          inval_addr;
    } cva6_cut_t;

    cva6_cut_t [NrCuts:0] cva6_cuts;

    for (genvar cut=0; cut < NrCuts; cut++) begin
        // Cut Request interface
        spill_register #(
            .T(accelerator_req_t)
        ) i_cva6_req_cut (
            .clk_i  (clk_i                         ),
            .rst_ni (rst_ni                        ),

            .valid_i(cva6_cuts[cut].req_valid      ),
            .ready_o(cva6_cuts[cut].req_ready      ),
            .data_i (cva6_cuts[cut].acc_req        ),

            .valid_o(cva6_cuts[cut+1].req_valid    ),
            .ready_i(cva6_cuts[cut+1].req_ready    ),
            .data_o (cva6_cuts[cut+1].acc_req      )
        );

        // Cut Response interface
        spill_register #(
            .T(accelerator_resp_t)
        ) i_cva6_resp_cut (
            .clk_i  (clk_i                ),
            .rst_ni (rst_ni               ),

            .valid_i(cva6_cuts[NrCuts-cut].resp_valid      ),
            .ready_o(cva6_cuts[NrCuts-cut].resp_ready      ),
            .data_i (cva6_cuts[NrCuts-cut].acc_resp        ),

            .valid_o(cva6_cuts[NrCuts-cut-1].resp_valid    ),
            .ready_i(cva6_cuts[NrCuts-cut-1].resp_ready    ),
            .data_o (cva6_cuts[NrCuts-cut-1].acc_resp      )
        );

        // Invalidation interface
        spill_register #(
            .T(logic [63:0])
        ) i_cva6_invalid_cut (
            .clk_i  (clk_i                                   ),
            .rst_ni (rst_ni                                  ),

            .valid_i(cva6_cuts[NrCuts-cut].inval_valid       ),
            .ready_o(cva6_cuts[NrCuts-cut].inval_ready       ),
            .data_i (cva6_cuts[NrCuts-cut].inval_addr        ),

            .valid_o(cva6_cuts[NrCuts-cut-1].inval_valid     ),
            .ready_i(cva6_cuts[NrCuts-cut-1].inval_ready     ),
            .data_o (cva6_cuts[NrCuts-cut-1].inval_addr      )
        );

        `FF(cva6_cuts[NrCuts-cut-1].load_complete  , cva6_cuts[NrCuts-cut].load_complete  , '0, clk_i, rst_ni);
        `FF(cva6_cuts[NrCuts-cut-1].store_complete , cva6_cuts[NrCuts-cut].store_complete , '0, clk_i, rst_ni);
        `FF(cva6_cuts[NrCuts-cut-1].store_pending  , cva6_cuts[NrCuts-cut].store_pending  , '0, clk_i, rst_ni);
        `FF(cva6_cuts[NrCuts-cut-1].fflags_valid   , cva6_cuts[NrCuts-cut].fflags_valid   , '0, clk_i, rst_ni);
        `FF(cva6_cuts[NrCuts-cut-1].fflags         , cva6_cuts[NrCuts-cut].fflags         , '0, clk_i, rst_ni);
        
        `FF(cva6_cuts[cut+1].acc_cons_en       , cva6_cuts[cut].acc_cons_en       , '0, clk_i, rst_ni);
        `FF(cva6_cuts[cut+1].store_pending_req , cva6_cuts[cut].store_pending_req , '0, clk_i, rst_ni);
    end

    always_comb begin
        //// Resp to CVA6
        acc_resp_o                = cva6_cuts[0].acc_resp;
        acc_resp_o.req_ready      = cva6_cuts[0].req_ready;
        acc_resp_o.resp_valid     = cva6_cuts[0].resp_valid;
        acc_resp_o.load_complete  = cva6_cuts[0].load_complete;
        acc_resp_o.store_complete = cva6_cuts[0].store_complete;
        acc_resp_o.store_pending  = cva6_cuts[0].store_pending;
        acc_resp_o.fflags_valid   = cva6_cuts[0].fflags_valid;
        acc_resp_o.fflags         = cva6_cuts[0].fflags;
        acc_resp_o.inval_valid    = cva6_cuts[0].inval_valid;
        acc_resp_o.inval_addr     = cva6_cuts[0].inval_addr;

        //// Resp from ARA
        cva6_cuts[NrCuts].acc_resp       = acc_resp_i;
        cva6_cuts[NrCuts].req_ready      = acc_resp_i.req_ready;
        cva6_cuts[NrCuts].resp_valid     = acc_resp_i.resp_valid;
        cva6_cuts[NrCuts].load_complete  = acc_resp_i.load_complete;
        cva6_cuts[NrCuts].store_complete = acc_resp_i.store_complete;
        cva6_cuts[NrCuts].store_pending  = acc_resp_i.store_pending;
        cva6_cuts[NrCuts].fflags_valid   = acc_resp_i.fflags_valid;
        cva6_cuts[NrCuts].fflags         = acc_resp_i.fflags;
        cva6_cuts[NrCuts].inval_valid    = acc_resp_i.inval_valid;
        cva6_cuts[NrCuts].inval_addr     = acc_resp_i.inval_addr;

        //// Request to ARA
        acc_req_o                 = cva6_cuts[NrCuts].acc_req;
        acc_req_o.req_valid       = cva6_cuts[NrCuts].req_valid;
        acc_req_o.resp_ready      = cva6_cuts[NrCuts].resp_ready;
        acc_req_o.inval_ready     = cva6_cuts[NrCuts].inval_ready;
        acc_req_o.store_pending   = cva6_cuts[NrCuts].store_pending_req;
        acc_req_o.acc_cons_en     = cva6_cuts[NrCuts].acc_cons_en;

        //// Request from CVA6
        cva6_cuts[0].acc_req            = acc_req_i;
        cva6_cuts[0].req_valid          = acc_req_i.req_valid;  
        cva6_cuts[0].resp_ready         = acc_req_i.resp_ready;
        cva6_cuts[0].inval_ready        = acc_req_i.inval_ready;
        cva6_cuts[0].store_pending_req  = acc_req_i.store_pending;
        cva6_cuts[0].acc_cons_en        = acc_req_i.acc_cons_en;
    end


endmodule