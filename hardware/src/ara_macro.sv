// Copyright 2021-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@student.ethz.ch>
// Description:
// A Repeatable ARA macro, containing ARA and ring router

module ara_macro import ara_pkg::*; import rvv_pkg::*; #(
    // RVV Parameters
    parameter  int           unsigned NrLanes      = 0,   // Number of parallel vector lanes per Ara instance

    // Support for floating-point data types
    parameter  fpu_support_e          FPUSupport   = FPUSupportHalfSingleDouble,
    // External support for vfrec7, vfrsqrt7
    parameter  fpext_support_e        FPExtSupport = FPExtSupportEnable,
    // Support for fixed-point data types
    parameter  fixpt_support_e        FixPtSupport = FixedPointEnable,
    // AXI Interface
    parameter  int           unsigned AxiDataWidth        = 0,
    parameter  int           unsigned AxiAddrWidth        = 0,
    parameter  int           unsigned ClusterAxiDataWidth = 0,

    parameter  type                   cluster_axi_ar_t     = logic,
    parameter  type                   cluster_axi_r_t      = logic,
    parameter  type                   cluster_axi_aw_t     = logic,
    parameter  type                   cluster_axi_w_t      = logic,
    parameter  type                   cluster_axi_b_t      = logic,
    parameter  type                   cluster_axi_req_t    = logic,
    parameter  type                   cluster_axi_resp_t   = logic,
  
    localparam int  unsigned DataWidth = $bits(elen_t),

    // Dependant parameters. DO NOT CHANGE!
    // Ara has NrLanes + 3 processing elements: each one of the lanes, the vector load unit, the
    // vector store unit, the slide unit, and the mask unit.
    localparam int           unsigned NrPEs        = NrLanes + 4
  ) (
    // Clock and Reset
    input  logic              clk_i,
    input  logic              rst_ni,

    // Scan chain
    input  logic              scan_enable_i,
    input  logic              scan_data_i,
    output logic              scan_data_o,

    // Id
    input  id_cluster_t       cluster_id_i,
    input  num_cluster_t      num_clusters_i,

    // Interface with Ariane
    input  accelerator_req_t  acc_req_i,
    output accelerator_resp_t acc_resp_o,

    // AXI interface
    output cluster_axi_req_t          axi_req_o,
    input  cluster_axi_resp_t         axi_resp_i,

    // Ring
    input remote_data_t  ring_data_r_i,
    input logic          ring_data_r_valid_i,
    output logic         ring_data_r_ready_o, 

    input remote_data_t  ring_data_l_i,
    input logic          ring_data_l_valid_i,
    output logic         ring_data_l_ready_o, 

    output remote_data_t ring_data_r_o,
    output logic         ring_data_r_valid_o,
    input logic          ring_data_r_ready_i, 

    output remote_data_t ring_data_l_o,
    output logic         ring_data_l_valid_o,
    input logic          ring_data_l_ready_i,

    output vew_e              vew_ar_o,
    output vew_e              vew_aw_o,
    output vlen_cluster_t     vl_ldst_o
  );

  `include "common_cells/registers.svh"

  // To System AXI
  cluster_axi_req_t     ara_axi_req;
  cluster_axi_resp_t    ara_axi_resp;

  // Sldu to Ariane
  remote_data_t sldu_i, sldu_o; 
  logic         sldu_valid_i, sldu_valid_o; 
  logic         sldu_ready_i, sldu_ready_o;

  vew_e vew_ar, vew_aw;
  vlen_cluster_t vl_ldst;
  id_cluster_t cluster_id;
  num_cluster_t num_clusters;

  accelerator_req_t acc_req, acc_req_o;
  accelerator_resp_t acc_resp, acc_resp_out;

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

  /////////////////////////
  // Cuts for the macro ///
  /////////////////////////

  `FF(vew_ar_o, vew_ar, vew_e'(1'b0), clk_i, rst_ni);
  `FF(vew_aw_o, vew_aw, vew_e'(1'b0), clk_i, rst_ni);
  `FF(vl_ldst_o, vl_ldst, '0, clk_i, rst_ni);
  `FF(cluster_id, cluster_id_i, '0, clk_i, rst_ni);
  `FF(num_clusters, num_clusters_i, '0, clk_i, rst_ni);
  
  // Cut Request interface
  spill_register #(
    .T(accelerator_req_t)
  ) i_cva6_req_cut (
    .clk_i  (clk_i                         ),
    .rst_ni (rst_ni                        ),
    
    .valid_i(acc_req_i.req_valid           ),
    .ready_o(req_ready                     ),
    .data_i (acc_req_i                     ),

    .valid_o(req_valid                     ),
    .ready_i(acc_resp.req_ready            ),
    .data_o (acc_req                       )
  );

  // Cut Response interface
  spill_register #(
    .T(accelerator_resp_t)
  ) i_cva6_resp_cut (
    .clk_i  (clk_i                        ),
    .rst_ni (rst_ni                       ),
    
    .valid_i(acc_resp.resp_valid         ),
    .ready_o(resp_ready                   ),
    .data_i (acc_resp                     ),

    .valid_o(resp_valid                   ),
    .ready_i(acc_req_i.resp_ready         ),
    .data_o (acc_resp_out                 )
  );

  // Invalidation interface
  spill_register #(
    .T(logic [63:0])
  ) i_cva6_invalid_cut (
    .clk_i  (clk_i                        ),
    .rst_ni (rst_ni                       ),
    
    .valid_i(acc_resp.inval_valid       ),
    .ready_o(inval_ready                  ),
    .data_i (acc_resp.inval_addr          ),

    .valid_o(inval_valid                ),
    .ready_i(acc_req_i.inval_ready        ),
    .data_o (inval_addr                 )
  );

   //`FF(trans_id, acc_resp.trans_id, '0, clk_i, rst_ni);
  `FF(load_complete, acc_resp.load_complete, '0, clk_i, rst_ni);
  `FF(store_complete, acc_resp.store_complete, '0, clk_i, rst_ni);
  `FF(store_pending, acc_resp.store_pending, '0, clk_i, rst_ni);
  `FF(fflags_valid, acc_resp.fflags_valid, '0, clk_i, rst_ni);
  `FF(fflags, acc_resp.fflags, '0, clk_i, rst_ni);
  `FF(acc_cons_en, acc_req_i.acc_cons_en, '0, clk_i, rst_ni);
  `FF(store_pending_req, acc_req_i.store_pending, '0, clk_i, rst_ni);

  always_comb begin
    //// Resp to CVA6
    acc_resp_o = acc_resp_out;
    acc_resp_o.req_ready = req_ready;
    acc_resp_o.resp_valid = resp_valid;
    
    // MetaData
    acc_resp_o.load_complete = load_complete;
    acc_resp_o.store_complete = store_complete;
    acc_resp_o.store_pending = store_pending;
    acc_resp_o.fflags_valid = fflags_valid;
    acc_resp_o.fflags = fflags;

    // Invalidation Interface
    acc_resp_o.inval_valid = inval_valid;
    acc_resp_o.inval_addr = inval_addr;

    //// Request from CVA6
    acc_req_o = acc_req;
    acc_req_o.req_valid = req_valid;
    acc_req_o.resp_ready = resp_ready;
    acc_req_o.inval_ready = inval_ready;
    acc_req_o.store_pending = store_pending_req;
    acc_req_o.acc_cons_en = acc_cons_en;

  end

  axi_cut #(
    .ar_chan_t   (cluster_axi_ar_t     ),
    .aw_chan_t   (cluster_axi_aw_t     ),
    .b_chan_t    (cluster_axi_b_t      ),
    .r_chan_t    (cluster_axi_r_t      ),
    .w_chan_t    (cluster_axi_w_t      ),
    .axi_req_t   (cluster_axi_req_t    ),
    .axi_resp_t  (cluster_axi_resp_t   )
  ) i_global_ldst_ara_axi_cut (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .slv_req_i   (ara_axi_req),
    .slv_resp_o  (ara_axi_resp),
    .mst_req_o   (axi_req_o),
    .mst_resp_i  (axi_resp_i)
  );

  ara #(
    .NrLanes     (NrLanes             ),
    .FPUSupport  (FPUSupport          ),
    .FPExtSupport(FPExtSupport        ),
    .FixPtSupport(FixPtSupport        ),
    .AxiDataWidth(ClusterAxiDataWidth ),
    .AxiAddrWidth(AxiAddrWidth        ),
    .axi_ar_t    (cluster_axi_ar_t    ),
    .axi_r_t     (cluster_axi_r_t     ),
    .axi_aw_t    (cluster_axi_aw_t    ),
    .axi_w_t     (cluster_axi_w_t     ),
    .axi_b_t     (cluster_axi_b_t     ),
    .axi_req_t   (cluster_axi_req_t   ),
    .axi_resp_t  (cluster_axi_resp_t  )
  ) i_ara (
    .clk_i           (clk_i            ),
    .rst_ni          (rst_ni           ),
    .scan_enable_i   (scan_enable_i    ),
    .scan_data_i     (1'b0             ),
    .scan_data_o     (/* Unused */     ),
    
    .cluster_id_i    (cluster_id       ),
    .num_clusters_i  (num_clusters     ),
    .acc_req_i       (acc_req_o        ),
    .acc_resp_o      (acc_resp         ),
    .axi_req_o       (ara_axi_req      ),
    .axi_resp_i      (ara_axi_resp     ),

    .vew_ar_o        (vew_ar           ),
    .vew_aw_o        (vew_aw           ),
    .vl_ldst_o       (vl_ldst          ),
    
    // To Ring Routers
    .ring_data_o         (sldu_o             ), 
    .ring_valid_o        (sldu_valid_o       ),
    .ring_ready_i        (sldu_ready_i       ),

    .ring_data_i         (sldu_i             ), 
    .ring_valid_i        (sldu_valid_i       ), 
    .ring_ready_o        (sldu_ready_o       )

  );

  ring_router i_ring_router (
    .clk_i             (clk_i),
    .rst_ni            (rst_ni),

    .cluster_id_i (cluster_id    ),
    .num_clusters_i  (num_clusters     ),
    
    // From SLDU in ARA
    .sldu_i       (sldu_o        ),
    .sldu_valid_i (sldu_valid_o  ),
    .sldu_ready_o (sldu_ready_i  ),  

    .sldu_o       (sldu_i        ),
    .sldu_valid_o (sldu_valid_i  ),
    .sldu_ready_i (sldu_ready_o  ),


    // From other ring routers
    .ring_right_i       (ring_data_r_i      ),
    .ring_right_valid_i (ring_data_r_valid_i),
    .ring_right_ready_o (ring_data_r_ready_o),

    .ring_left_i        (ring_data_l_i      ),
    .ring_left_valid_i  (ring_data_l_valid_i),
    .ring_left_ready_o  (ring_data_l_ready_o),

    .ring_right_o       (ring_data_r_o      ),
    .ring_right_valid_o (ring_data_r_valid_o),
    .ring_right_ready_i (ring_data_r_ready_i),

    .ring_left_o        (ring_data_l_o      ),
    .ring_left_valid_o  (ring_data_l_valid_o),
    .ring_left_ready_i  (ring_data_l_ready_i)
  );

endmodule