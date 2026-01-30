// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Ara's System, containing Ara instances and Global Units connecting them.

module ara_cluster import ara_pkg::*; import rvv_pkg::*;  #(
    // RVV Parameters
    parameter  int           unsigned NrLanes      = 0,   // Number of parallel vector lanes per Ara instance
    parameter  int           unsigned NrClusters   = 0,   // Number of Ara instances

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
    parameter  int           unsigned NrAxiCuts           = 2,

    parameter  type                   axi_ar_t     = logic,
    parameter  type                   axi_r_t      = logic,
    parameter  type                   axi_aw_t     = logic,
    parameter  type                   axi_w_t      = logic,
    parameter  type                   axi_b_t      = logic,
    parameter  type                   axi_req_t    = logic,
    parameter  type                   axi_resp_t   = logic,

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
    // Interface with Ariane
    input  accelerator_req_t  acc_req_i,
    output accelerator_resp_t acc_resp_o,
    // AXI interface
    output axi_req_t          axi_req_o,
    input  axi_resp_t         axi_resp_i

  );

  `include "common_cells/registers.svh" 

  // Number of Clusters configuration
  num_cluster_t numClusters;
  assign numClusters = $clog2(NrClusters);

  // Intermediate signals
  accelerator_req_t [NrClusters-1:0] acc_req, acc_req_cut;
  logic req_ready, resp_valid;

  accelerator_resp_t [NrClusters-1:0] acc_resp, acc_resp_cut;
  accelerator_resp_t acc_resp_d, acc_resp_q;

  cluster_axi_req_t      [NrClusters-1:0] ara_axi_req, ara_axi_req_cut, ldst_axi_req, ldst_axi_req_cut;
  cluster_axi_resp_t     [NrClusters-1:0] ara_axi_resp, ara_axi_resp_cut, ldst_axi_resp, ldst_axi_resp_cut;

  axi_req_t  axi_req_cut, axi_req_ldst, axi_req_align, axi_req_align_o;
  axi_resp_t axi_resp_cut, axi_resp_ldst, axi_resp_align, axi_resp_align_i;

  // Ring connections
  remote_data_t [NrClusters-1:0] ring_data_l, ring_data_r;
  logic [NrClusters-1:0] ring_data_l_ready, ring_data_l_valid, ring_data_r_ready, ring_data_r_valid;

  remote_data_t [NrClusters-1:0] ring_data_l_cut, ring_data_r_cut;
  logic [NrClusters-1:0] ring_data_l_ready_cut, ring_data_l_valid_cut, ring_data_r_ready_cut, ring_data_r_valid_cut;

  // Hierarchical stream fork
  localparam int nlevels = $clog2(NrClusters);
  typedef accelerator_req_t [NrClusters-1:0] fork_req_t;
  typedef accelerator_resp_t [NrClusters-1:0] fork_resp_t;

  fork_req_t  [nlevels : 0] acc_req_fork;
  fork_resp_t [nlevels : 0] acc_resp_fork;

  // Shuffle stage to ARA cuts
  localparam int ShuffleCuts = $clog2(NrClusters);
  typedef cluster_axi_req_t   [NrClusters-1:0] group_req_t;
  typedef cluster_axi_resp_t  [NrClusters-1:0] group_resp_t;

  group_req_t [ShuffleCuts:0] ara_axi_req_shuffle_cut;
  group_resp_t [ShuffleCuts:0] ara_axi_resp_shuffle_cut;

  vew_e [NrClusters-1:0]  vew_ar, vew_aw;
  vew_e                   vew_ar_cut, vew_aw_cut, vew_ar_cut_glsu, vew_aw_cut_glsu, 
                          vew_ar_glsu2align, vew_aw_glsu2align, vew_ar_align, vew_aw_align;
  
  vlen_cluster_t [NrClusters-1:0] vl_ldst;
  vlen_cluster_t  vl_ldst_cut, vl_ldst_cut_glsu, vl_ldst_rd_align, vl_ldst_wr_align;

  typedef vew_e [NrClusters-1:0] vew_group_t;
  typedef vlen_t [NrClusters-1:0] vlen_group_t;

  vew_group_t [ShuffleCuts:0] vew_ar_shuffle_cut, vew_aw_shuffle_cut;
  vlen_group_t [ShuffleCuts:0] vl_ldst_shuffle_cut;

  /////////
  // ARA //
  /////////

  // Some of the macros are almost abutted in the floorplan.
  // Thus, some of the ring interfaces will be extremely close together and will not need an AXI cut.
  // This optimization IS floorplan dependent!
  localparam bit modulate_ring_cuts = 1;

  // Function to check if an index is in a [NrCluster/2]-wide list
  function automatic bit is_index_in_list(int index, int list[8]);
    int found = 0;
    for (int i = 0; i < NrClusters/2; i++) begin
      if (list[i] == index) found = 1;
    end

    return found;
  endfunction

  localparam int idx_no_ring_cut_left[8] = '{1, 3, 5, 7, 9, 11, 13, 15};
  localparam int idx_no_ring_cut_right[8] = '{0, 2, 4, 6, 8, 10, 12, 14};

  for (genvar cluster=0; cluster < NrClusters; cluster++) begin : p_cluster
      ara_macro #(
        .NrLanes           (NrLanes             ),
        .FPUSupport        (FPUSupport          ),
        .FPExtSupport      (FPExtSupport        ),
        .FixPtSupport      (FixPtSupport        ),
        .AxiDataWidth      (AxiDataWidth        ),
        .AxiAddrWidth      (AxiAddrWidth        ),
        .ClusterAxiDataWidth (ClusterAxiDataWidth),
        .cluster_axi_ar_t  (cluster_axi_ar_t    ),
        .cluster_axi_r_t   (cluster_axi_r_t     ),
        .cluster_axi_aw_t  (cluster_axi_aw_t    ),
        .cluster_axi_w_t   (cluster_axi_w_t     ),
        .cluster_axi_b_t   (cluster_axi_b_t     ),
        .cluster_axi_req_t (cluster_axi_req_t   ),
        .cluster_axi_resp_t(cluster_axi_resp_t  )
      ) i_ara_macro (
        .clk_i             (clk_i               ),
        .rst_ni            (rst_ni              ),

        .scan_enable_i     (scan_enable_i       ),
        .scan_data_i       (scan_data_i         ),
        .scan_data_o       (/* Unused */        ),

        // Id
        .cluster_id_i      (id_cluster_t'(cluster) ),
        .num_clusters_i    (numClusters            ),

        // Interface with Ariane
        .acc_req_i         (acc_req_fork[nlevels][cluster]    ),
        .acc_resp_o        (acc_resp_fork[nlevels][cluster]   ),

        // AXI interface
        .axi_req_o         (ara_axi_req[cluster]   ),
        .axi_resp_i        (ara_axi_resp[cluster]  ),

        .vew_ar_o        (vew_ar[cluster]         ),
        .vew_aw_o        (vew_aw[cluster]         ),
        .vl_ldst_o       (vl_ldst[cluster]        ),

        // Ring
        .ring_data_r_i       (ring_data_l_cut        [cluster == NrClusters-1 ? 0 : cluster + 1]     ),
        .ring_data_r_valid_i (ring_data_l_valid_cut  [cluster == NrClusters-1 ? 0 : cluster + 1]     ),
        .ring_data_r_ready_o (ring_data_l_ready  [cluster]                                           ),

        .ring_data_l_i       (ring_data_r_cut        [cluster == 0 ? NrClusters-1 : cluster - 1]     ),
        .ring_data_l_valid_i (ring_data_r_valid_cut  [cluster == 0 ? NrClusters-1 : cluster - 1]     ),
        .ring_data_l_ready_o (ring_data_r_ready_cut  [cluster]                                       ),

        .ring_data_r_o       (ring_data_r        [cluster]                                       ),
        .ring_data_r_valid_o (ring_data_r_valid  [cluster]                                       ),
        .ring_data_r_ready_i (ring_data_r_ready  [cluster == NrClusters-1 ? 0 : cluster + 1]     ),

        .ring_data_l_o       (ring_data_l        [cluster]                                       ),
        .ring_data_l_valid_o (ring_data_l_valid  [cluster]                                       ),
        .ring_data_l_ready_i (ring_data_l_ready_cut  [cluster == 0 ? NrClusters-1 : cluster - 1]     )
      );

      // Cuts from ara macro to the shuffle stage
      assign ara_axi_req_shuffle_cut[0][cluster] = ara_axi_req[cluster];
      assign ara_axi_resp[cluster] = ara_axi_resp_shuffle_cut[0][cluster];

      assign vew_ar_shuffle_cut[0][cluster] = vew_ar[cluster];
      assign vew_aw_shuffle_cut[0][cluster] = vew_aw[cluster];
      assign vl_ldst_shuffle_cut[0][cluster] = vl_ldst[cluster];

      for (genvar s=0; s < ShuffleCuts; s++) begin
        axi_cut #(
          .ar_chan_t   (cluster_axi_ar_t     ),
          .aw_chan_t   (cluster_axi_aw_t     ),
          .b_chan_t    (cluster_axi_b_t      ),
          .r_chan_t    (cluster_axi_r_t      ),
          .w_chan_t    (cluster_axi_w_t      ),
          .axi_req_t   (cluster_axi_req_t    ),
          .axi_resp_t  (cluster_axi_resp_t   )
        ) i_macro_axi_cut (
          .clk_i       (clk_i),
          .rst_ni      (rst_ni),

          .slv_req_i   (ara_axi_req_shuffle_cut [s][cluster]),
          .slv_resp_o  (ara_axi_resp_shuffle_cut[s][cluster]),

          .mst_req_o   (ara_axi_req_shuffle_cut [s+1][cluster]),
          .mst_resp_i  (ara_axi_resp_shuffle_cut[s+1][cluster])
        );
        `FF(vew_ar_shuffle_cut[s+1][cluster], vew_ar_shuffle_cut[s][cluster], vew_e'('0), clk_i, rst_ni)
        `FF(vew_aw_shuffle_cut[s+1][cluster], vew_aw_shuffle_cut[s][cluster], vew_e'('0), clk_i, rst_ni)
        `FF(vl_ldst_shuffle_cut[s+1][cluster], vl_ldst_shuffle_cut[s][cluster], '0, clk_i, rst_ni)
      end
  end

  `ifdef ADD_RING_LATENCY

    typedef remote_data_t [`RING_LATENCY : 0] ring_cut_data_t;
    ring_cut_data_t [NrClusters-1 : 0] data_l_cut, data_r_cut;
    typedef logic [`RING_LATENCY:0] ring_bit_t;
    ring_bit_t [NrClusters-1:0] data_l_valid_cut, data_l_ready_cut, data_r_valid_cut, data_r_ready_cut;

    for (genvar cluster=0; cluster < NrClusters; cluster++) begin
      for (genvar i=0; i < `RING_LATENCY; i++) begin : p_ring_cut
          spill_register #(
            .T(remote_data_t)
          ) i_ring_latency_left (
            .clk_i  (clk_i                      ),
            .rst_ni (rst_ni                     ),

            .valid_i(data_l_valid_cut     [cluster][i]   ),
            .ready_o(data_l_ready_cut     [cluster][i]   ),
            .data_i (data_l_cut           [cluster][i]   ),

            .valid_o(data_l_valid_cut     [cluster][i+1] ),
            .ready_i(data_l_ready_cut     [cluster][i+1] ),
            .data_o (data_l_cut           [cluster][i+1] )
          );

          spill_register #(
            .T(remote_data_t)
          ) i_ring_latency_right (
            .clk_i  (clk_i                      ),
            .rst_ni (rst_ni                     ),

            .valid_i(data_r_valid_cut     [cluster][i]   ),
            .ready_o(data_r_ready_cut     [cluster][i]   ),
            .data_i (data_r_cut           [cluster][i]   ),

            .valid_o(data_r_valid_cut     [cluster][i+1] ),
            .ready_i(data_r_ready_cut     [cluster][i+1] ),
            .data_o (data_r_cut           [cluster][i+1] )
          );
      end

      assign data_l_valid_cut[cluster][0] = ring_data_l_valid[cluster];
      assign ring_data_l_valid_cut[cluster] = data_l_valid_cut[cluster][`RING_LATENCY];

      assign data_l_cut[cluster][0] = ring_data_l[cluster];
      assign ring_data_l_cut[cluster] = data_l_cut[cluster][`RING_LATENCY];

      assign ring_data_l_ready_cut[cluster == 0 ? NrClusters-1 : cluster - 1] = data_l_ready_cut[cluster][0];
      assign data_l_ready_cut[cluster][`RING_LATENCY] = ring_data_l_ready[cluster == 0 ? NrClusters-1 : cluster - 1];

      assign data_r_valid_cut[cluster][0] = ring_data_r_valid[cluster];
      assign ring_data_r_valid_cut[cluster] = data_r_valid_cut[cluster][`RING_LATENCY];

      assign data_r_cut[cluster][0] = ring_data_r[cluster];
      assign ring_data_r_cut[cluster] = data_r_cut[cluster][`RING_LATENCY];

      assign ring_data_r_ready[cluster == NrClusters-1 ? 0 : cluster + 1] = data_r_ready_cut[cluster][0];
      assign data_r_ready_cut[cluster][`RING_LATENCY] = ring_data_r_ready_cut[cluster == NrClusters-1 ? 0 : cluster + 1];
    end

  `else

      for (genvar cluster=0; cluster < NrClusters; cluster++) begin
        // Check if this cluster needs a cut on the left
        if (is_index_in_list(cluster, idx_no_ring_cut_left) && (NrClusters==8 || NrClusters==16)) begin
          // Pass through
          assign ring_data_l_valid_cut[cluster] = ring_data_l_valid[cluster];
          assign ring_data_l_ready_cut[cluster == 0 ? NrClusters-1 : cluster - 1] = ring_data_l_ready[cluster == 0 ? NrClusters-1 : cluster - 1];
          assign ring_data_l_cut[cluster] = ring_data_l[cluster];
        end else begin
          // Cuts on the ring interface
          // To meet timing at the top level
          spill_register #(
            .T(remote_data_t)
          ) i_ring_macro_spill_left (
            .clk_i  (clk_i                        ),
            .rst_ni (rst_ni                       ),

            .valid_i(ring_data_l_valid         [cluster]                                     ),
            .ready_o(ring_data_l_ready_cut     [cluster == 0 ? NrClusters-1 : cluster - 1]   ),
            .data_i (ring_data_l               [cluster]                                     ),

            .valid_o(ring_data_l_valid_cut     [cluster]   ),
            .ready_i(ring_data_l_ready         [cluster == 0 ? NrClusters-1 : cluster - 1]    ),
            .data_o (ring_data_l_cut           [cluster]   )
          );
        end

        // Check if this cluster needs a cut on the right
        if (is_index_in_list(cluster, idx_no_ring_cut_right) && (NrClusters==8 || NrClusters==16)) begin
          // Pass through
          assign ring_data_r_valid_cut[cluster] = ring_data_r_valid[cluster];
          assign ring_data_r_ready[cluster == NrClusters-1 ? 0 : cluster + 1] = ring_data_r_ready_cut[cluster == NrClusters-1 ? 0 : cluster + 1];
          assign ring_data_r_cut[cluster] = ring_data_r[cluster];
        end else begin
          spill_register #(
            .T(remote_data_t)
          ) i_ring_macro_spill_right (
            .clk_i  (clk_i                        ),
            .rst_ni (rst_ni                       ),

            .valid_i(ring_data_r_valid     [cluster]                                     ),
            .ready_o(ring_data_r_ready     [cluster == NrClusters-1 ? 0 : cluster + 1]   ),
            .data_i (ring_data_r           [cluster]                                     ),

            .valid_o(ring_data_r_valid_cut         [cluster]                                       ),
            .ready_i(ring_data_r_ready_cut         [cluster == NrClusters-1 ? 0 : cluster + 1]     ),
            .data_o (ring_data_r_cut               [cluster]                                       )
          );
        end
      end

  `endif

  //////////////////
  // GLOBAL LD-ST //
  //////////////////

  // Shuffle stage
  assign ara_axi_req_cut = ara_axi_req_shuffle_cut[ShuffleCuts];
  assign ara_axi_resp_shuffle_cut[ShuffleCuts] = ara_axi_resp_cut;

  assign vew_ar_cut = vew_ar_shuffle_cut[ShuffleCuts][0];
  assign vew_aw_cut = vew_aw_shuffle_cut[ShuffleCuts][0];
  assign vl_ldst_cut = vl_ldst_shuffle_cut[ShuffleCuts][0];

  shuffle_stage #(
    .NrLanes              (NrLanes              ),
    .NrClusters           (NrClusters           ),
      .ClusterAxiDataWidth(ClusterAxiDataWidth  ),
      .AxiAddrWidth       (AxiAddrWidth         ),
      .axi_r_t            (cluster_axi_r_t      ),
      .axi_w_t            (cluster_axi_w_t      ),
      .axi_req_t          (cluster_axi_req_t    ),
      .axi_resp_t         (cluster_axi_resp_t   )
    ) i_shuffle_stage (
      .clk_i              (clk_i                ),
      .rst_ni             (rst_ni               ),

      .vew_ar_i           (vew_ar_cut           ),
      .vew_aw_i           (vew_aw_cut           ),
      .vl_i               (vl_ldst_cut          ),

      .axi_req_i          (ara_axi_req_cut      ),
      .axi_resp_o         (ara_axi_resp_cut     ),

      .axi_req_o          (ldst_axi_req_cut     ),
      .axi_resp_i         (ldst_axi_resp_cut    )
  );

  for (genvar cluster=0; cluster < NrClusters; cluster++) begin : p_cluster_cut
    axi_cut #(
        .ar_chan_t   (cluster_axi_ar_t     ),
        .aw_chan_t   (cluster_axi_aw_t     ),
        .b_chan_t    (cluster_axi_b_t      ),
        .r_chan_t    (cluster_axi_r_t      ),
        .w_chan_t    (cluster_axi_w_t      ),
        .axi_req_t   (cluster_axi_req_t    ),
        .axi_resp_t  (cluster_axi_resp_t   )
    ) i_shuffle_axi_cut (
        .clk_i       (clk_i),
        .rst_ni      (rst_ni),

        .slv_req_i   (ldst_axi_req_cut[cluster]),
        .slv_resp_o  (ldst_axi_resp_cut[cluster]),

        .mst_req_o   (ldst_axi_req[cluster]),
        .mst_resp_i  (ldst_axi_resp[cluster])
      );
  end

  `FF(vew_ar_cut_glsu, vew_ar_cut, vew_e'('0), clk_i, rst_ni)
  `FF(vew_aw_cut_glsu, vew_aw_cut, vew_e'('0), clk_i, rst_ni)
  `FF(vl_ldst_cut_glsu, vl_ldst_cut, '0, clk_i, rst_ni)

  // Global Ld/St Unit
  global_ldst #(
    .NrLanes            (NrLanes            ),
    .NrClusters         (NrClusters         ),
    .AxiDataWidth       (AxiDataWidth       ),
    .ClusterAxiDataWidth(ClusterAxiDataWidth),
    .AxiAddrWidth       (AxiAddrWidth       ),
    .cluster_axi_req_t  (cluster_axi_req_t  ),
    .cluster_axi_resp_t (cluster_axi_resp_t ),
    .axi_req_t          (axi_req_t          ),
    .axi_resp_t         (axi_resp_t         )
  ) i_global_ldst (
    .clk_i              (clk_i              ),
    .rst_ni             (rst_ni             ),

    // To Ara
    .axi_req_i          (ldst_axi_req       ),
    .axi_resp_o         (ldst_axi_resp      ),

    .vew_ar_i           (vew_ar_cut_glsu      ),
    .vew_aw_i           (vew_aw_cut_glsu      ),
    .vl_ldst_i          (vl_ldst_cut_glsu     ),

    .vew_ar_o           (vew_ar_glsu2align    ),
    .vew_aw_o           (vew_aw_glsu2align    ),

    // .axi_req_i       (ara_axi_req_cut    ),
    // .axi_resp_o      (ara_axi_resp_cut   ),

    // To System
    .axi_resp_i         (axi_resp_cut       ),
    .axi_req_o          (axi_req_cut        )
  );

  axi_cut #(
    .ar_chan_t   (axi_ar_t     ),
    .aw_chan_t   (axi_aw_t     ),
    .b_chan_t    (axi_b_t      ),
    .r_chan_t    (axi_r_t      ),
    .w_chan_t    (axi_w_t      ),
    .axi_req_t   (axi_req_t    ),
    .axi_resp_t  (axi_resp_t   )
  ) i_global_axi_cut (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),

    .slv_req_i   (axi_req_cut),
    .slv_resp_o  (axi_resp_cut),

    .mst_req_o   (axi_req_ldst),
    .mst_resp_i  (axi_resp_ldst)
  );

  `FF(vew_ar_align, vew_ar_glsu2align, vew_e'('0), clk_i, rst_ni)
  `FF(vew_aw_align, vew_aw_glsu2align, vew_e'('0), clk_i, rst_ni)
  `FF(vl_ldst_rd_align, vl_ldst_cut_glsu, '0, clk_i, rst_ni)
  `FF(vl_ldst_wr_align, vl_ldst_cut_glsu, '0, clk_i, rst_ni)

  // Align stage
  align_stage #(
      .NrClusters       (NrClusters         ),
      .AxiDataWidth     (AxiDataWidth       ),
      .AxiAddrWidth     (AxiAddrWidth       ),
      .axi_ar_t         (axi_ar_t           ),
      .axi_aw_t         (axi_aw_t           ),
      .axi_b_t          (axi_b_t            ),
      .axi_r_t          (axi_r_t            ),
      .axi_w_t          (axi_w_t            ),
      .axi_req_t        (axi_req_t          ),
      .axi_resp_t       (axi_resp_t         )
    ) i_align_stage (
      .clk_i            (clk_i              ),
      .rst_ni           (rst_ni             ),

      .vew_ar_i         (vew_ar_align       ),
      .vew_aw_i         (vew_aw_align       ),
      .vl_ldst_rd_i     (vl_ldst_rd_align   ),
      .vl_ldst_wr_i     (vl_ldst_wr_align   ),

      // .axi_req_i        (axi_req_cut        ),
      // .axi_resp_o       (axi_resp_cut       ),

      .axi_req_i        (axi_req_ldst        ),
      .axi_resp_o       (axi_resp_ldst       ),

      .axi_req_o        (axi_req_align      ),
      .axi_resp_i       (axi_resp_align     )
  );

  axi_cut #(
    .ar_chan_t   (axi_ar_t     ),
    .aw_chan_t   (axi_aw_t     ),
    .b_chan_t    (axi_b_t      ),
    .r_chan_t    (axi_r_t      ),
    .w_chan_t    (axi_w_t      ),
    .axi_req_t   (axi_req_t    ),
    .axi_resp_t  (axi_resp_t   )
  ) i_align_axi_cut (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),

    .slv_req_i   (axi_req_align),
    .slv_resp_o  (axi_resp_align),

    // .slv_req_i   (axi_req_cut),
    // .slv_resp_o  (axi_resp_cut),

    .mst_req_o   (axi_req_align_o),
    .mst_resp_i  (axi_resp_align_i)
  );

  // For performance simulations to understand impact of memory access latency
  `ifdef ADD_MEM_LATENCY
    axi_req_t [`MEM_LATENCY:0] axi_req_latency;
    axi_resp_t [`MEM_LATENCY:0] axi_resp_latency;

    for (genvar i=0; i < `MEM_LATENCY; i++) begin

      axi_cut #(
        .ar_chan_t   (axi_ar_t     ),
        .aw_chan_t   (axi_aw_t     ),
        .b_chan_t    (axi_b_t      ),
        .r_chan_t    (axi_r_t      ),
        .w_chan_t    (axi_w_t      ),
        .axi_req_t   (axi_req_t    ),
        .axi_resp_t  (axi_resp_t   )
      ) i_mem_latency_axi_cut (
        .clk_i       (clk_i),
        .rst_ni      (rst_ni),

        .slv_req_i   (axi_req_latency[i]),
        .slv_resp_o  (axi_resp_latency[i]),

        .mst_req_o   (axi_req_latency[i+1]),
        .mst_resp_i  (axi_resp_latency[i+1])
      );
    end

    assign axi_req_latency[0] = axi_req_align_o;
    assign axi_resp_latency[`MEM_LATENCY] = axi_resp_i;

    assign axi_req_o = axi_req_latency[`MEM_LATENCY];
    assign axi_resp_align_i = axi_resp_latency[0];

  `else
    assign axi_req_o = axi_req_align_o;
    assign axi_resp_align_i = axi_resp_i;

  `endif

  //////////////////
  ////// CVA6 //////
  //////////////////

  // Synchronizing requests among clusters
  // Need to distribute a CVA6 request to all clusters
  // Should happen only when all clusters are ready to receive the request
  // This is implemented by using a stream fork module

  // For performance simulations to understand impact of cva6 latency
  `ifdef ADD_CVA6_LATENCY

    accelerator_req_t req_cut_i, req_cut_o;
    accelerator_resp_t resp_cut_i, resp_cut_o;

    cva6_cut # (
      .NrCuts      (`CVA6_LATENCY )
    ) i_cva6_latency_cut (
      .clk_i       (clk_i         ),
      .rst_ni      (rst_ni        ),

      .acc_req_i   (req_cut_i     ),
      .acc_resp_o  (resp_cut_o    ),

      .acc_req_o   (req_cut_o     ),
      .acc_resp_i  (resp_cut_i    )
    );

    assign req_cut_i = acc_req_i;
    assign acc_resp_o = resp_cut_o;

    assign acc_req_fork[0][0] = req_cut_o;
    assign resp_cut_i = acc_resp_fork[0][0];

  `else
    assign acc_req_fork[0][0] = acc_req_i;
    assign acc_resp_o = acc_resp_fork[0][0];

  `endif

  for (genvar l=0; l < nlevels; l++) begin : p_level
    for (genvar n=0; n < (1<<l); n++) begin : p_fork

      // Req fork
      req_fork_cut #(
        //.NrCuts ((l==0 || l==2) ? 1 : 0)
        .NrCuts (1)
      ) i_req_fork (
        .clk_i (clk_i                          ),
        .rst_ni(rst_ni                         ),

        .req_i (acc_req_fork[l][n]         ),
        .resp_o(acc_resp_fork[l][n]         ),

        .req_o (acc_req_fork[l+1][2*n +: 2]  ),
        .resp_i(acc_resp_fork[l+1][2*n +: 2]  )
      );
    end
  end

  if (NrClusters > MaxNrClusters)
    $error("Increase MaxNrClusters in ara_pkg size");

endmodule : ara_cluster