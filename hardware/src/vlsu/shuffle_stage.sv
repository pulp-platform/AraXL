// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// This module does the shuffling of data coming from Memory in stages to achieve the required element mapping.
// The shuffling should be such that the first N elements (where N is no of lanes) go to Cluster-0 for all data types, 
// Next N elements go to Cluster-1 and so on.

module shuffle_stage import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int           unsigned NrLanes             = 0,   // Number of Lanes in each ARA
  parameter  int           unsigned NrClusters          = 0,   // Number of Ara instances
  parameter  int           unsigned ClusterAxiDataWidth        = 0,   // Axi Data width of one cluster
  parameter  int           unsigned AxiAddrWidth        = 0,
  parameter  type                   axi_r_t             = logic,
  parameter  type                   axi_w_t             = logic,
  parameter  type                   axi_req_t           = logic,
  parameter  type                   axi_resp_t          = logic,
  
  parameter  type                   axi_addr_t          = logic [AxiAddrWidth-1:0],
  parameter  type                   axi_data_t          = logic [NrClusters*ClusterAxiDataWidth-1:0],
  // Dependant parameters. DO NOT CHANGE!
  // Shuffling starts from EEW1 to support mask loads
  localparam int           unsigned TotalNrLanes        = NrClusters * NrLanes,
  localparam int           unsigned NumStages           = $clog2(ClusterAxiDataWidth/NrLanes),
  localparam int           unsigned NumBuffers          = 2,
  localparam int           unsigned ClustersPerBuffer   = (NrClusters > NumBuffers) ? NrClusters / NumBuffers : 1
) (
  // Clock and Reset
  input  logic                        clk_i,
  input  logic                        rst_ni,

  input  cluster_metadata_t [NrClusters-1:0]  cluster_metadata_i,

  // Synchronization with cluster addrgen for indexed operations 
  output logic                                idx_completed_o,
  
  input   axi_req_t  [NrClusters-1:0] axi_req_i,
  output  axi_req_t  [NrClusters-1:0] axi_req_o,

  input   axi_resp_t [NrClusters-1:0] axi_resp_i,
  output  axi_resp_t [NrClusters-1:0] axi_resp_o
);

`include "common_cells/registers.svh"

// There are 2 dapaths in this unit
// 1) Shuffle - to shuffle the data coming from memory to the required cluster based on element width
// 2) Buffer - to buffer the data coming from memory if the element width is 64b and ClusterAxiDataWidth is 32N, since in this case, the data coming from memory needs to be stored and sent in 2 cycles to the clusters. 
// This is only needed for loads, for stores we can just buffer the write data until we have enough data to send to the clusters.
localparam int unsigned NUM_DATAPATHS = 2;
localparam int unsigned NumTrackers=16;

typedef enum logic { SHUFFLE, BUFFER } datapath_t;
typedef logic [$clog2(NumTrackers)-1:0] pnt_t; 
typedef logic [$clog2(NumTrackers):0] cnt_t;
typedef axi_w_t [NrClusters-1:0] stage_w_t; 

logic [NrClusters-1:0] buf_sel_d, buf_sel_q;
logic cluster_sel_d, cluster_sel_q;
logic cluster_buf_ready, cluster_buf_valid;

pnt_t [NumStages-1:0] wr_issue_pnt_d, wr_issue_pnt_q;

logic [NrClusters-1:0] wr_cluster_completed_d, wr_cluster_status_completed;

`FF(buf_sel_q,              buf_sel_d,              '0, clk_i, rst_ni)
`FF(cluster_sel_q,          cluster_sel_d,          '0, clk_i, rst_ni)

// This is the main tracking structure for the requests coming into the shuffle stage. 
// It keeps track of the status of each request and is used to configure the shuffle and buffer datapath.
typedef struct packed {
  axi_addr_t addr;
  vlen_t [NrClusters-1:0] len;
  elen_t stride;
  vew_e vew;
  logic is_load;
  logic is_burst;
  logic [NumStages-1:0] shuffle_en;
  
  // 1'b0 - shuffle - 1(mask)/8/16/32b data
  // 1'b1 - buffer - 64b data
  datapath_t datapath;
  logic second_buffer_unused;

  logic valid;
  vlen_cluster_t [NrClusters-1:0] vl;
  logic use_eew1;
  ara_op_e op;
} req_track_t;

req_track_t [NumTrackers-1:0] rd_tracker_d, rd_tracker_q;
pnt_t rd_accept_pnt_d, rd_accept_pnt_q;
pnt_t [NumStages-1:0] rd_issue_pnt_d, rd_issue_pnt_q;
cnt_t rd_cnt_d, rd_cnt_q;

req_track_t [NumTrackers-1:0] wr_tracker_d, wr_tracker_q;
pnt_t wr_accept_pnt_d, wr_accept_pnt_q;
cnt_t wr_cnt_d, wr_cnt_q;

typedef axi_r_t [NrClusters-1:0] stage_r_t;
stage_r_t [NumStages-1:0] r_data_in, r_data_out;

stage_w_t [NumStages-1:0] w_data_in, w_data_out;

logic [NumStages-1:0] r_valid, r_ready, w_valid, w_ready;
logic [NumStages-1:0] r_shuffle_en, w_shuffle_en;

logic [NrClusters-1:0] r_ready_i, r_valid_o;

logic rd_full, wr_full;
assign rd_full = (rd_cnt_q == NumTrackers);
assign wr_full = (wr_cnt_q == NumTrackers);

logic [$clog2(NrLanes):0] lane_ar_d, lane_ar_q;
logic [$clog2(NrLanes):0] lane_aw_d, lane_aw_q;
logic [$clog2(NrLanes):0] lane_w_d, lane_w_q;
logic [$clog2(NrClusters):0] cluster_ar_d, cluster_ar_q;
logic [$clog2(NrClusters):0] cluster_aw_d, cluster_aw_q;
logic [$clog2(NrClusters):0] cluster_w_d, cluster_w_q;

logic wr_idx_accepted_d, wr_idx_accepted_q;

// To handle cases where vlsu of each cluster is ready to 
// receive read resp or not.
stream_fork #(
  .N_OUP(NrClusters)
) i_cluster_stream_fork (
  .clk_i  (clk_i), 
  .rst_ni (rst_ni),
  .valid_o(r_valid_o            ),
  .valid_i(r_valid[NumStages-1] ), 
  .ready_i(r_ready_i            ),
  .ready_o(r_ready[NumStages-1] )
);

logic [NrClusters-1:0] axi_wr_buffer_valid, axi_wr_buffer_ready;
logic [NrClusters-1:0] axi_wr_shuffle_valid, axi_wr_shuffle_ready;
logic [NrClusters-1:0] axi_wr_shuffle_ready_inp, axi_wr_shuffle_valid_inp;

// To handle cases where write data does not come simultaneously 
// from all the clusters
stream_join #(
  .N_INP(NrClusters)
) i_cluster_stream_join (
  .inp_ready_o(axi_wr_shuffle_ready_inp),
  .inp_valid_i(axi_wr_shuffle_valid_inp),
  .oup_ready_i(w_ready[0]),
  .oup_valid_o(w_valid[0])
);

always_comb begin
  axi_wr_shuffle_ready = axi_wr_shuffle_ready_inp;
  axi_wr_shuffle_valid_inp = axi_wr_shuffle_valid;

  for (int c=0; c<NrClusters; c++) begin
    wr_cluster_status_completed[c] = (wr_cnt_q > 0) && (wr_tracker_q[wr_issue_pnt_q[0]].vl[c] == '0);
    if (wr_cluster_status_completed[c] & axi_req_i[0].w_valid) begin
      // Fake handshake for all completed clusters for 1 more cycle
      // say not ready to upstream, send a dummy packet for the completed cluster downstream
      axi_wr_shuffle_ready[c] = 1'b0;
      axi_wr_shuffle_valid_inp[c] = 1'b1;
    end
  end
end

for (genvar s=0; s<NumStages; s++) begin : p_stage

  // Shuffling read data
  shuffle #(
    .NrLanes             (NrLanes),
    .NrClusters          (NrClusters                  ),  
    .ClusterAxiDataWidth (ClusterAxiDataWidth         ),
    .T                   (stage_r_t                   ),
    .scale               (s                           ),
    .isRead              (1                           )
  ) i_shuffle_rd (
    .data_i       ( r_data_in  [s]  ),
    .data_o       ( r_data_out [s]  ),
    .enable_i     ( r_shuffle_en [s]  )
  );

  if (s >= 1) begin
    stream_register #(
      .T       (stage_r_t)
    ) i_shuffle_reg_r  (
      .clk_i      ( clk_i                     ),
      .rst_ni     ( rst_ni                    ),
      .clr_i      ( 1'b0                      ),
      .testmode_i ( 1'b0                      ),
      // Input
      .valid_i    ( r_valid    [s-1]          ),
      .ready_o    ( r_ready    [s-1]          ),
      .data_i     ( r_data_out [s-1]          ),
      // Output
      .valid_o    ( r_valid    [s]            ),
      .ready_i    ( r_ready    [s]            ),
      .data_o     ( r_data_in  [s]            )
    );
  end
 
  shuffle #(
    .NrLanes             (NrLanes),
    .NrClusters          (NrClusters                     ),  
    .ClusterAxiDataWidth (ClusterAxiDataWidth            ),
    .T                   (stage_w_t                      ),
    .scale               (NumStages - s -1               ),
    .isRead              (0                              ),
    .isMask              ((NumStages - s -1) > 0 ? 0 : 1 )
  ) i_shuffle_wr (
    .data_i       ( w_data_in  [s]    ),
    .data_o       ( w_data_out [s]    ),
    .enable_i     ( w_shuffle_en [s]  )
  );

  if (s >= 1) begin
    stream_register #(
      .T       (stage_w_t)
    ) i_shuffle_reg_w  (
      .clk_i      ( clk_i                     ),
      .rst_ni     ( rst_ni                    ),
      .clr_i      ( 1'b0                      ),
      .testmode_i ( 1'b0                      ),
      // Input
      .valid_i    ( w_valid    [s-1]          ),
      .ready_o    ( w_ready    [s-1]          ),
      .data_i     ( w_data_out [s-1]          ),
      // Output
      .valid_o    ( w_valid    [s]            ),
      .ready_i    ( w_ready    [s]            ),
      .data_o     ( w_data_in  [s]            )
    );
  end
end

// Set status of shuffle stage to the current pointers
for (genvar s=0; s<NumStages; s++) begin
  assign r_shuffle_en[s] = rd_tracker_q[rd_issue_pnt_q[s]].shuffle_en[s];
  assign w_shuffle_en[s] = wr_tracker_q[wr_issue_pnt_q[s]].shuffle_en[s];
end

///////////////
// Buffering //
///////////////

// Read Responses
typedef axi_r_t [NrClusters-1:0] axi_resp_ext_t;
axi_resp_ext_t [NumBuffers-1:0] buf_d, buf_q;
axi_resp_t [NrClusters-1:0]  axi_resp_buf_out;

logic rdbuf_pnt_q, rdbuf_pnt_d;
logic [NumBuffers-1:0] shift_d, shift_q;                           // For each buffer a single bit is needed. (For BW 32N only)
logic [NumBuffers-1:0] buf_valid_d, buf_valid_q;
logic r_ready_buf, r_ready_buf_q;

datapath_t rd_datapath;
ara_op_e rd_op;

logic stall_rd_resp;
logic is_datapath_switch;
logic is_op_switch;
req_track_t rd_tracker_first, rd_tracker_last, rd_tracker_second;

logic is_shuffle_ongoing, is_buffer_ongoing;

always_comb begin
  rd_tracker_first = rd_tracker_q[rd_issue_pnt_q[0]];
  rd_tracker_second = rd_tracker_q[rd_issue_pnt_q[1]];
  rd_tracker_last = rd_tracker_q[rd_issue_pnt_q[NumStages-1]];
  
  is_shuffle_ongoing = (rd_tracker_last.datapath == SHUFFLE) && (rd_tracker_first.datapath != SHUFFLE);
  is_buffer_ongoing = (rd_tracker_last.datapath == BUFFER) && !(rd_tracker_last.op inside {VLXE, VLSE}) && (rd_tracker_first.op inside {VLXE, VLSE} || rd_tracker_second.op inside {VLXE, VLSE});

  stall_rd_resp = (is_shuffle_ongoing || is_buffer_ongoing) && (rd_cnt_q > 1);

  rd_datapath = rd_tracker_last.datapath;
  rd_op = rd_tracker_last.op;
end

//////////////////////////////////
// Mux/Demux for read responses //
//////////////////////////////////

logic [NrClusters-1:0] axi_rd_unit_stride_ready, axi_rd_unit_stride_valid;
logic [NrClusters-1:0] axi_rd_buffer_ready, axi_rd_buffer_valid;
logic [NrClusters-1:0] axi_req_out_r_ready;
logic [NrClusters-1:0] axi_rd_shuffle_valid, axi_rd_shuffle_ready;
logic [NrClusters-1:0] axi_rd_indexed_valid, axi_rd_indexed_ready;

for (genvar c=0; c<NrClusters; c++) begin
  stream_demux #(
    .N_OUP(2)
  ) i_demux_indexed_rd (
    .inp_valid_i   (axi_resp_i[c].r_valid                                 ),
    .inp_ready_o   (axi_req_out_r_ready[c]                                ),
    .oup_sel_i     (rd_op inside {VLXE, VLSE} ? 1'b0 : 1'b1               ),
    .oup_valid_o   ({axi_rd_unit_stride_valid[c], axi_rd_indexed_valid[c]}),
    .oup_ready_i   ({axi_rd_unit_stride_ready[c], axi_rd_indexed_ready[c]})
  );

  stream_demux #(
    .N_OUP(2)
  ) i_demux_unit_stride_rd (
    .inp_valid_i   (axi_rd_unit_stride_valid[c]                      ),
    .inp_ready_o   (axi_rd_unit_stride_ready[c]                      ),
    .oup_sel_i     ((rd_datapath == BUFFER) ? 1'b0 : 1'b1            ),
    .oup_valid_o   ({axi_rd_shuffle_valid[c], axi_rd_buffer_valid[c]}),
    .oup_ready_i   ({axi_rd_shuffle_ready[c], axi_rd_buffer_ready[c]})
  );

  assign axi_req_o[c].r_ready = axi_req_out_r_ready[c] & ~stall_rd_resp;
  assign r_data_in[0][c] = axi_resp_i[c].r;
end
assign axi_rd_shuffle_ready = {NrClusters{r_ready[0]}};
assign r_valid[0] = axi_rd_shuffle_valid[0];

// Write packets
logic [NrClusters-1:0] [ClusterAxiDataWidth*2-1:0]  wrbuf_d, wrbuf_q;
logic [NrClusters-1:0] [(ClusterAxiDataWidth*2/8)-1:0]  wrbuf_be_d, wrbuf_be_q;
axi_req_t [NrClusters-1:0]  axi_req_buf_out;

logic [$clog2(NrClusters)-1:0] wrbuf_pnt_q, wrbuf_pnt_d;
logic [NrClusters-1:0] wr_shift_d, wr_shift_q;
logic [NrClusters-1:0] wrbuf_valid, wrbuf_valid_q;
logic [NrClusters-1:0] wrbuf_full, wrbuf_full_q;
logic wr_out_ready, wr_out_valid;

datapath_t wr_datapath;
ara_op_e wr_op;
assign wr_datapath = wr_tracker_q[wr_issue_pnt_q[0]].datapath;
assign wr_op = wr_tracker_q[wr_issue_pnt_q[0]].op;

logic [NrClusters-1:0] rd_cluster_completed_d, rd_cluster_completed_q;
logic [NumBuffers-1:0] rd_buffer_completed_d, rd_buffer_completed_q;

vlen_cluster_t vl_idx_cluster_d, vl_idx_cluster_q;

logic pending_resp;
logic buffer_ld_resp_accepted;

logic [NrClusters-1:0] buffer_wr_data_accepted;

///////////////////////////////
// Mux/Demux for write data  //
///////////////////////////////

logic [NrClusters-1:0] axi_req_unit_stride_ready, axi_req_unit_stride_valid;
logic [NrClusters-1:0] axi_wr_unit_stride_ready, axi_wr_unit_stride_valid;
logic [NrClusters-1:0] axi_wr_indexed_ready, axi_wr_indexed_valid;

for (genvar c=0; c<NrClusters; c++) begin
  stream_demux #(
    .N_OUP(2)
  ) i_demux_indexed_wr (
    .inp_valid_i   (axi_req_i[c].w_valid                                  ),
    .inp_ready_o   (axi_resp_o[c].w_ready                                 ),
    .oup_sel_i     (wr_op inside {VSXE, VSSE} ? 1'b0 : 1'b1               ),
    .oup_valid_o   ({axi_wr_unit_stride_valid[c], axi_wr_indexed_valid[c]}),
    .oup_ready_i   ({axi_wr_unit_stride_ready[c], axi_wr_indexed_ready[c]})
  );

  stream_demux #(
    .N_OUP(NUM_DATAPATHS)
  ) i_demux_unit_stride_wr (
    .inp_valid_i   (axi_wr_unit_stride_valid[c]                      ),
    .inp_ready_o   (axi_wr_unit_stride_ready[c]                      ),
    .oup_sel_i     (wr_datapath                                      ),
    .oup_valid_o   ({axi_wr_buffer_valid[c], axi_wr_shuffle_valid[c]}),
    .oup_ready_i   ({axi_wr_buffer_ready[c], axi_wr_shuffle_ready[c]})
  );
end

logic   [NumBuffers-1:0] [NrClusters-1:0] wr_buf_valid, wr_buf_ready;
logic   [NumBuffers-1:0] [NrClusters-1:0] wr_buf_valid_in, wr_buf_ready_in;

stage_w_t wr_arb_data_in;
stage_w_t [NumBuffers-1:0] wr_arb_data;

for (genvar c=0; c<NrClusters; c++) begin
  stream_demux #(
    .N_OUP(2)
  ) i_demux_wr_spill (
    .inp_valid_i   (axi_wr_buffer_valid[c]                        ),
    .inp_ready_o   (axi_wr_buffer_ready[c]                        ),
    .oup_sel_i     (buf_sel_q[c]                                  ),
    .oup_valid_o   ({wr_buf_valid_in[1][c], wr_buf_valid_in[0][c]}),
    .oup_ready_i   ({wr_buf_ready_in[1][c], wr_buf_ready_in[0][c]})
  );

  for (genvar b=0; b<NumBuffers; b++) begin
    spill_register #(
      .T(axi_w_t)
    ) i_wr_spill_reg (
      .clk_i      ( clk_i                ),
      .rst_ni     ( rst_ni               ),
      .valid_i    ( wr_buf_valid_in[b][c]),
      .ready_o    ( wr_buf_ready_in[b][c]),
      .data_i     ( wr_arb_data_in[c]    ),
      .valid_o    ( wr_buf_valid[b][c]   ),
      .ready_i    ( wr_buf_ready[b][c]   ),
      .data_o     ( wr_arb_data[b][c]    )
    );
  end
end

logic [NrClusters-1:0] w_ready_shuffle;

always_comb begin
  // Reset req fields
  for (int c=0; c < NrClusters; c++) begin
    wr_arb_data_in[c] = axi_req_i[c].w;
    wr_arb_data_in[c].last = 1'b0;
  end
end

stage_w_t  wr_buf_data_o;
stage_w_t  axi_req_unit_stride_o;
logic [NrClusters-1:0] wr_buf_ready_o, wr_buf_data_valid;

for (genvar c=0; c<NrClusters; c++) begin
  stream_mux #(
    .DATA_T(axi_w_t),
    .N_INP(NUM_DATAPATHS)
  ) i_mux_unit_stride (
    .inp_data_i    ({wr_buf_data_o[c], w_data_out[NumStages-1][c]}),
    .inp_ready_o   ({wr_buf_ready_o[c], w_ready_shuffle[c]}       ),
    .inp_valid_i   ({wr_buf_data_valid[c], w_valid[NumStages-1]}  ),
    .inp_sel_i     (wr_datapath                                   ),
    .oup_ready_i   (axi_req_unit_stride_ready[c]                  ),
    .oup_valid_o   (axi_req_unit_stride_valid[c]                  ),
    .oup_data_o    (axi_req_unit_stride_o[c]                      )
  );

  stream_mux #(
    .DATA_T(axi_w_t),
    .N_INP(2)
  ) i_mux_indexed (
    .inp_data_i    ({axi_req_unit_stride_o[c], axi_req_i[c].w}             ),
    .inp_ready_o   ({axi_req_unit_stride_ready[c], axi_wr_indexed_ready[c]}),
    .inp_valid_i   ({axi_req_unit_stride_valid[c], axi_wr_indexed_valid[c]}),
    .inp_sel_i     (wr_op inside {VSXE, VSSE} ? 1'b0 : 1'b1                ),
    .oup_ready_i   (axi_resp_i[c].w_ready                                  ),
    .oup_valid_o   (axi_req_o[c].w_valid                                   ),
    .oup_data_o    (axi_req_o[c].w                                         )
  );
end

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    // R
    buf_q              <= '0;
    buf_valid_q        <= '0;
    rdbuf_pnt_q        <= '0;
    shift_q            <= '0;
    r_ready_buf_q      <= 1'b1;
    // W
    wrbuf_q            <= '0;
    wrbuf_pnt_q        <= '0; 
    wr_shift_q         <= '0;
    wrbuf_valid_q      <= '0;
    wrbuf_full_q       <= '0;
    wrbuf_be_q         <= '0;
  end else begin
    // R
    buf_q              <= buf_d;
    buf_valid_q        <= buf_valid_d;
    rdbuf_pnt_q        <= rdbuf_pnt_d;
    shift_q            <= shift_d;
    r_ready_buf_q      <= r_ready_buf;
    // W
    wrbuf_q            <= wrbuf_d;
    wrbuf_pnt_q        <= wrbuf_pnt_d; 
    wr_shift_q         <= wr_shift_d;
    wrbuf_valid_q      <= wrbuf_valid;
    wrbuf_full_q       <= wrbuf_full;
    wrbuf_be_q         <= wrbuf_be_d;
  end
end

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    rd_tracker_q    <= '0;      
    wr_tracker_q    <= '0;
    rd_accept_pnt_q <= '0;
    rd_issue_pnt_q  <= '0;
    rd_cnt_q        <= '0;
    wr_accept_pnt_q <= '0;
    wr_issue_pnt_q  <= '0;
    wr_cnt_q        <= '0;
    rd_cluster_completed_q <= '0;
    rd_buffer_completed_q <= '0;
    cluster_ar_q <= '0;
    lane_ar_q <= '0;
    cluster_aw_q <= '0;
    lane_aw_q <= '0;
    cluster_w_q <= '0;
    lane_w_q <= '0;
    vl_idx_cluster_q <= '0;
    wr_idx_accepted_q <= '0;
  end else begin
    rd_tracker_q    <= rd_tracker_d;
    wr_tracker_q    <= wr_tracker_d;
    rd_accept_pnt_q <= rd_accept_pnt_d;
    rd_issue_pnt_q  <= rd_issue_pnt_d;
    rd_cnt_q        <= rd_cnt_d;
    wr_accept_pnt_q <= wr_accept_pnt_d;
    wr_issue_pnt_q  <= wr_issue_pnt_d;
    wr_cnt_q        <= wr_cnt_d;
    rd_cluster_completed_q <= rd_cluster_completed_d;
    rd_buffer_completed_q <= rd_buffer_completed_d;
    cluster_ar_q <= cluster_ar_d;
    lane_ar_q <= lane_ar_d;
    cluster_aw_q <= cluster_aw_d;
    lane_aw_q <= lane_aw_d;
    cluster_w_q <= cluster_w_d;
    lane_w_q <= lane_w_d;
    vl_idx_cluster_q <= vl_idx_cluster_d;
    wr_idx_accepted_q <= wr_idx_accepted_d;
  end
end

always_comb begin

  rd_tracker_d = rd_tracker_q;
  rd_accept_pnt_d = rd_accept_pnt_q;
  rd_issue_pnt_d = rd_issue_pnt_q;
  rd_cnt_d = rd_cnt_q;

  wr_tracker_d = wr_tracker_q;
  wr_accept_pnt_d = wr_accept_pnt_q;
  wr_issue_pnt_d = wr_issue_pnt_q;
  wr_cnt_d = wr_cnt_q;

  axi_resp_buf_out = '0;
  axi_req_buf_out = '0;

  //////////////
  // Requests //
  //////////////

  // If a request arrives, add to tracker.
  // Request taken from cluster 0
  lane_ar_d = lane_ar_q;
  cluster_ar_d = cluster_ar_q;
  vl_idx_cluster_d = vl_idx_cluster_q;
  idx_completed_o = 1'b0;
  
  if (axi_req_i[cluster_ar_q].ar_valid & axi_resp_o[cluster_ar_q].ar_ready) begin
    automatic cluster_metadata_t cluster_metadata = cluster_metadata_i[cluster_ar_q];
    // Store element width
    rd_tracker_d[rd_accept_pnt_q].vew = cluster_metadata.vew;
    rd_tracker_d[rd_accept_pnt_q].use_eew1 = cluster_metadata.use_eew1;
    // Track number of beats and vl
    for (int c=0; c<NrClusters; c++) begin
      automatic int unsigned vl_tot = cluster_metadata.use_eew1 ? cluster_metadata.vl << 3 : cluster_metadata.vl;
      automatic int unsigned vl_rem = vl_tot & (TotalNrLanes - 1);
      automatic int unsigned vl_base = vl_tot >> $clog2(TotalNrLanes);
      automatic int unsigned vl_rem_diff = vl_rem - (c * NrLanes);
      automatic int unsigned vl = (vl_base << $clog2(NrLanes)) + ((vl_rem >= (c + 1) * NrLanes) ? NrLanes : (vl_rem >= (c * NrLanes)) ? vl_rem_diff : '0);

      rd_tracker_d[rd_accept_pnt_q].len[c] = axi_req_i[cluster_ar_q].ar.len+1;
      rd_tracker_d[rd_accept_pnt_q].vl[c] = cluster_metadata.use_eew1 ? (vl < 8) ? 1 : vl >> 3 : vl;
    end
    // Update pnt to accept next request
    rd_accept_pnt_d = (rd_accept_pnt_q == NumTrackers-1) ? '0 : rd_accept_pnt_q + 1;
    rd_cnt_d += 1;
    // To enable certain shuffle stages based on element width
    for (int s=0; s<NumStages; s++) begin
      rd_tracker_d[rd_accept_pnt_q].shuffle_en[s] = cluster_metadata.use_eew1 ? 1'b1 : (s >= (3 + cluster_metadata.vew)) ? 1'b1 : 1'b0;
    end
    // To enable buffer for 64b element widths
    rd_tracker_d[rd_accept_pnt_q].datapath = NumStages < (3 + cluster_metadata.vew) ? BUFFER : SHUFFLE;
    rd_tracker_d[rd_accept_pnt_q].second_buffer_unused = cluster_metadata.vl <= (NrLanes * NrClusters / 2) && rd_tracker_d[rd_accept_pnt_q].datapath;
    rd_tracker_d[rd_accept_pnt_q].op = cluster_metadata.op;

    // If it is a VLXE/VLSE request, take from the desired cluster
    // and switch clusters for every NrLanes requests
    if (cluster_metadata.op inside {VLXE, VLSE}) begin
      lane_ar_d += 1;
      if (lane_ar_q == NrLanes - 1) begin
        cluster_ar_d += 1;
        if (cluster_ar_q == NrClusters - 1) begin
          cluster_ar_d = '0;
        end
        lane_ar_d = '0;
      end
      
      // If a valid request is sent, track it for synchronization
      if (axi_req_o[cluster_ar_q].ar_valid & axi_resp_o[cluster_ar_q].ar_ready) begin
        vl_idx_cluster_d = vl_idx_cluster_q + 1;
        if (vl_idx_cluster_q == (cluster_metadata.vl - 1)) begin
          vl_idx_cluster_d = '0;
          idx_completed_o = 1'b1;
          lane_ar_d = '0;
          cluster_ar_d = '0;
        end
      end
    end else begin
      lane_ar_d = '0;
      cluster_ar_d = '0;
    end
  end

  lane_aw_d = lane_aw_q;
  cluster_aw_d = cluster_aw_q;
  lane_w_d = lane_w_q;
  cluster_w_d = cluster_w_q;
  wr_idx_accepted_d = wr_idx_accepted_q;
  
  if (axi_req_i[cluster_aw_q].aw_valid & axi_resp_o[cluster_aw_q].aw_ready) begin
    automatic cluster_metadata_t cluster_metadata = cluster_metadata_i[cluster_aw_q];

    if (!wr_idx_accepted_q) begin
      // Store element width
      wr_tracker_d[wr_accept_pnt_q].vew = cluster_metadata.vew;
      wr_tracker_d[wr_accept_pnt_q].use_eew1 = cluster_metadata.use_eew1;
      // Track number of beats and vl
      for (int c=0; c<NrClusters; c++) begin
        automatic int unsigned vl_tot = cluster_metadata.vl;
        automatic int unsigned vl_rem = vl_tot & (TotalNrLanes - 1);
        automatic int unsigned vl_base = vl_tot >> $clog2(TotalNrLanes);
        automatic int unsigned vl_rem_diff = vl_rem - (c * NrLanes);
        automatic int unsigned vl = (vl_base << $clog2(NrLanes)) + ((vl_rem >= (c + 1) * NrLanes) ? NrLanes : (vl_rem >= (c * NrLanes)) ? vl_rem_diff : '0);      

        wr_tracker_d[wr_accept_pnt_q].vl[c] = vl;
        wr_tracker_d[wr_accept_pnt_q].len[c] = axi_req_i[cluster_aw_q].aw.len+1;
      end
      // Update pnt to accept next request
      wr_accept_pnt_d = (wr_accept_pnt_q == NumTrackers-1) ? '0 : wr_accept_pnt_q + 1; 
      wr_cnt_d += 1;

      // If indexed/strided request, write to tracker only once
      wr_idx_accepted_d = (cluster_metadata.op inside {VSXE, VSSE});

      // To enable certain shuffle stages based on element width
      for (int s=0; s<NumStages; s++) begin
        wr_tracker_d[wr_accept_pnt_q].shuffle_en[s] = cluster_metadata.use_eew1 ? 1'b1 : (((NumStages -s -1) >= (3 + cluster_metadata.vew)) ? 1'b1 : 1'b0);
      end
      // To enable buffer for 64b element widths
      wr_tracker_d[wr_accept_pnt_q].datapath = NumStages < (3 + cluster_metadata.vew) ? BUFFER : SHUFFLE;
      wr_tracker_d[wr_accept_pnt_q].second_buffer_unused = cluster_metadata.vl <= (NrLanes * NrClusters / 2) && wr_tracker_d[wr_accept_pnt_q].datapath;
      wr_tracker_d[wr_accept_pnt_q].op = cluster_metadata.op;
    end

    // If it is a VSXE/VSSE request, take from the desired cluster
    // and switch clusters for every NrLanes requests
    if (cluster_metadata.op inside {VSXE, VSSE}) begin
      lane_aw_d += 1;
      if (lane_aw_q == NrLanes - 1) begin
        cluster_aw_d += 1;
        if (cluster_aw_q == NrClusters - 1) begin
          cluster_aw_d = '0;
        end
        lane_aw_d = '0;
      end

      // If a valid request is sent, track it for synchronization
      if (axi_req_o[cluster_aw_q].aw_valid & axi_resp_i[cluster_aw_q].aw_ready) begin
        vl_idx_cluster_d = vl_idx_cluster_q + 1;
        if (vl_idx_cluster_q == (cluster_metadata.vl - 1)) begin
          vl_idx_cluster_d = '0;
          idx_completed_o = 1'b1;
          lane_aw_d = '0;
          cluster_aw_d = '0;
          wr_idx_accepted_d = 1'b0;
        end
      end
    end else begin
      lane_aw_d = '0;
      cluster_aw_d = '0;
      wr_idx_accepted_d = 1'b0;
    end
  end

  // Update counters for shuffle stage
  // Update issue pointer of each stage
  // Once last packet is received by each stage, point to the next tracker.
  for (int s=0; s < NumStages; s++) begin
    
    // Reset shuffle config for the read shuffle stages
    // If the last stage sends the last packet, we need to go to the vew of the next request
    if (r_data_out[s][0].last & r_valid[s] & r_ready[s]) begin
      rd_issue_pnt_d[s] = (rd_issue_pnt_q[s] == NumTrackers-1) ? '0 : rd_issue_pnt_q[s] + 1;
      // In the last stage, reset the shift enable for the tracker instance
      if (s==NumStages-1) begin
        rd_tracker_d[rd_issue_pnt_q[s]].shuffle_en = '0;
        rd_tracker_d[rd_issue_pnt_q[s]].datapath = SHUFFLE;
        rd_cnt_d -= 1'b1;
      end
    end
    
    // Reset shuffle config for the write shuffle stages
    if (w_data_out[s][0].last & w_valid[s] & w_ready[s]) begin
      wr_issue_pnt_d[s] = (wr_issue_pnt_q[s] == NumTrackers-1) ? '0 : wr_issue_pnt_q[s] + 1;
      // In the last stage, reset the shift enable for the tracker instance
      if (s==NumStages-1) begin
        wr_tracker_d[wr_issue_pnt_q[s]].shuffle_en = '0;
        wr_tracker_d[wr_issue_pnt_q[s]].datapath = SHUFFLE;
        wr_cnt_d -= 1'b1;
      end
    end
  end

  // Update vl
  if (r_valid[NumStages-1] & r_ready[NumStages-1]) begin
    automatic logic [$clog2(ClusterAxiDataWidth/8):0] nelem = (ClusterAxiDataWidth/8) >> rd_tracker_q[rd_issue_pnt_q[NumStages-1]].vew;
    for (int c=0; c<NrClusters; c++) begin
      if (rd_tracker_q[rd_issue_pnt_q[NumStages-1]].vl[c] <= nelem) begin
        rd_tracker_d[rd_issue_pnt_q[NumStages-1]].vl[c] = '0;
      end else begin
        rd_tracker_d[rd_issue_pnt_q[NumStages-1]].vl[c] -= nelem;
      end
    end
  end

  ///////////////
  // Buffering //
  ///////////////
  
  // Handling buffering of read responses

  // Handling cases where input data maps only to a single cluster e.g. ClusterAxiDataWidth=32N and EW=64
  // In this case, need to buffer the current data to be used in the following cycles.
  // NOTE : This buffering logic implemented only works for the default BW config.
  // ClusterAxiDataWidth = 32N and AxiDataWidth=32NC

  buf_d = buf_q;
  buf_valid_d = buf_valid_q;
  rdbuf_pnt_d = rdbuf_pnt_q;
  shift_d = shift_q;
  r_ready_buf = r_ready_buf_q;
  buffer_ld_resp_accepted = 1'b0;

  axi_rd_buffer_ready = '0;
  axi_rd_indexed_ready = '0;

  rd_cluster_completed_d = rd_cluster_completed_q;
  rd_buffer_completed_d = rd_buffer_completed_q;

  if ((rd_datapath == BUFFER) && (rd_op == VLE)) begin
    ///// UNIT STRIDE LOADS /////
    ///// 64b precision     /////

    // If have a valid handshake on response add to the buffer
    // If have a valid response from L2 after aligning buffer it first pointed by rdbuf_pnt_q
    // Set we have a valid data
    if (axi_rd_buffer_valid[0] & r_ready_buf_q & (&r_ready_i)) begin
      for (int c=0; c<NrClusters; c++) begin
        buf_d[rdbuf_pnt_q][c] = axi_resp_i[c].r;
      end
      buf_valid_d[rdbuf_pnt_q] = 1'b1;
      rdbuf_pnt_d = (rdbuf_pnt_q == 1'b1) ? 1'b0 : 1'b1;
      axi_rd_buffer_ready = '1;
    end

    // Assign data in buffer to the output
    for (int b=0; b < NumBuffers; b++) begin
      // Assign data from buffers to the desired clusters
      // The offset from each buffer is defined by shift
      if (buf_valid_d[b]) begin
        automatic logic cluster_ready = 1'b1;
        for (int c=0; c < (NrClusters / NumBuffers); c++) begin
          automatic int cl = b ? (NrClusters / NumBuffers) + c : c;
          
          // First Half of the the clusters take data from buf[0] rest half from buf[1]
          axi_resp_buf_out[cl].r.data = buf_d[b][c*2 + shift_d[b]].data;  // 2 works for default 32N configuration to support EW=64
          
          // Is the set of cluster corresponding to this buffer ready to receive data
          cluster_ready &= axi_req_i[cl].r_ready;
        end
        if (cluster_ready) begin
          // Only if we have a valid data and clusters ready to receive
          for (int c=0; c < (NrClusters / NumBuffers); c++) begin
            automatic logic [$clog2(ClusterAxiDataWidth/8):0] nelem = (ClusterAxiDataWidth/8) >> rd_tracker_q[rd_issue_pnt_q[b]].vew;
            automatic int cl = b ? (NrClusters / NumBuffers) + c : c;
            
            // Set valid to the response
            axi_resp_buf_out[cl].r_valid = (rd_tracker_q[rd_issue_pnt_q[b]].vl[cl] > 0) ? 1'b1 : 1'b0;
            rd_tracker_d[rd_issue_pnt_q[b]].len[cl] -= 1;

            // If the response is the last response, set last
            if (rd_tracker_q[rd_issue_pnt_q[b]].vl[cl] <= nelem) begin 
              // set last packet
              axi_resp_buf_out[cl].r.last = 1'b1;

              // reduce vl
              rd_tracker_d[rd_issue_pnt_q[b]].vl[cl] = '0;
              
              // set the status of cluster to completed
              rd_cluster_completed_d[cl] = 1'b1;
              
            end else begin
              // If not the last packet, update vl
              rd_tracker_d[rd_issue_pnt_q[b]].vl[cl] -= nelem;

              // Update the shift to point to the offset of the buffer
              shift_d[b] = (shift_q[b] == 1'b1) ? 1'b0 : 1'b1;
              if (shift_q[b] == 1'b1) begin
                buf_valid_d[b] = 1'b0;
              end
            end
          end
          
          // If the clusters corresponding to a buffer completed,
          // Clear buffer valid and go to the next instruction
          if (&(rd_cluster_completed_d[b*ClustersPerBuffer +: ClustersPerBuffer])) begin
            // Change to next instruction for the particular buffer
            // Since each buffer can complete at different times, we maintain a different instruction pointer
            rd_issue_pnt_d[b] = (rd_issue_pnt_q[b] == NumTrackers-1) ? '0 : rd_issue_pnt_q[b] + 1;
            
            // clear buffer for the next instruction
            buf_valid_d[b] = 1'b0;
            shift_d[b] = 1'b0;

            // Set the clusters corresponding to the buffer as completed for the instruction
            rd_cluster_completed_d[b*ClustersPerBuffer +: ClustersPerBuffer] = '0;

            // Set the buffer as completed
            rd_buffer_completed_d[b] = 1'b1;
          end

          // If instruction completed, i.e. both buffers have been utilized and read from
          if ( &rd_buffer_completed_d | (rd_buffer_completed_d == 2'b01 && rd_tracker_q[rd_issue_pnt_q[b]].second_buffer_unused == 1'b1)) begin
            // Update counters
            rd_cnt_d -= 1;

            // If the first buffer has the last response and if there is also a valid packet in the current cycle, do no reset the pointer
            // If it is the second buffer that finished last, or they complete together, can reset the pointer the required buffer
            // since in the next cycle we proceed with the next instruction and we want to start loading data always from buffer 0
            if ((rd_buffer_completed_q == 2'b01 && ~buf_valid_d[0]) || rd_buffer_completed_q == 2'b10 || rd_buffer_completed_q == 2'b00)
              rdbuf_pnt_d = '0;

            // Reset cluster completed signal
            rd_buffer_completed_d = '0;

            // Once a 64b load request is completed, reset pointers for all stages
            // to handle cases where the next request uses shuffle datapath
            for (int i=0; i <NumStages; i++) begin
              rd_issue_pnt_d[i] = rd_issue_pnt_d[b];
            end
          end
        end
      end
    end
    
    // The next buffer has to be available only then ready to receive
    r_ready_buf = (buf_valid_d[rdbuf_pnt_d] == 1'b0);
  end else if (rd_op inside {VLXE, VLSE}) begin

    ///// INDEXED/STRIDED LOADS /////
    ///// All bit precisions ///// 

    // If indexed and strided operation, just forward the data coming from GLSU without shuffling or buffering
    automatic logic single_cluster_handshake = 1'b0;
    for (int c=0; c < NrClusters; c++) begin
      axi_resp_buf_out[c].r_valid = axi_rd_indexed_valid[c];
      axi_resp_buf_out[c].r = axi_resp_i[c].r;
      single_cluster_handshake |= (axi_rd_indexed_valid[c] & axi_req_i[c].r_ready);
    end

    // Do this if we have any valid response
    // Forward the data directly to axi_resp_o if we have a ready
    if (single_cluster_handshake) begin
      rd_cnt_d -= 1;
      for (int i=0; i <NumStages; i++) begin
        rd_issue_pnt_d[i] = rd_issue_pnt_q[i] + 1;
      end
      axi_rd_indexed_ready = '1;
    end
  end

  // Handling buffering of write packets
  wrbuf_d       = wrbuf_q;
  wrbuf_pnt_d   = wrbuf_pnt_q; 
  wr_shift_d    = wr_shift_q;
  wrbuf_valid   = wrbuf_valid_q;
  wrbuf_full    = wrbuf_full_q;
  wrbuf_be_d    = wrbuf_be_q;

  // If a buff is full write it to the output
  wr_out_valid = 1'b1;
  wr_out_ready = 1'b1;
  buffer_wr_data_accepted = '0;

  cluster_buf_ready = 1'b1;
  cluster_buf_valid = 1'b1;

  buf_sel_d = buf_sel_q;
  cluster_sel_d = cluster_sel_q;

  wr_buf_ready = '0;
  wr_buf_data_valid = '0;
  wr_buf_data_o = '0;

  wr_cluster_completed_d = wr_cluster_status_completed;

  if (wr_op == VSE) begin
    
    ///// UNIT STRIDE STORES /////

    if (wr_datapath == BUFFER) begin
  
      ///// 64b precision     /////

      // switch spill buffer on valid handshake
      for (int c=0; c<NrClusters; c++) begin
        if (axi_wr_buffer_valid[c] & axi_wr_buffer_ready[c]) begin
          buf_sel_d[c] = ~buf_sel_q[c];
          if (axi_req_i[c].w.last) begin
            buf_sel_d[c] = '0;
          end
        end
      end

      // Check valid handshake
      for (int c=0; c<NrClusters/2; c++) begin
        for (int b=0; b<NumBuffers; b++) begin
          automatic int unsigned cluster_in = cluster_sel_q * (NrClusters/2) + c;
          automatic int unsigned cluster_out = c * 2 + b;
          cluster_buf_ready &= wr_buf_ready_o[cluster_out];
          cluster_buf_valid &= (wr_buf_valid[b][cluster_in]) || 
                               (wr_buf_valid[0][cluster_in] && (wr_tracker_q[wr_issue_pnt_q[0]].vl[cluster_in] <= 2)) || 
                               (wr_cluster_status_completed[cluster_in]);
          wr_buf_data_o[cluster_out] = wr_arb_data[b][cluster_in];
        end
      end

      // Edge cases where vector length is not same across clusters
      if (cluster_buf_ready & cluster_buf_valid) begin
        cluster_sel_d = ~cluster_sel_q;
        for (int c=0; c<NrClusters/2; c++) begin
          for (int b=0; b<NumBuffers; b++) begin
            automatic int unsigned cluster_in = cluster_sel_q * (NrClusters/2) + c;
            automatic int unsigned other_cluster_in = (~cluster_sel_q) * (NrClusters/2) + c;
            automatic int unsigned cluster_out = c * 2 + b;
            automatic int vl = wr_tracker_q[wr_issue_pnt_q[0]].vl[cluster_in];
            // If vl<=2 ack only one buffer of a cluster
            if (vl <= 2) begin
              wr_buf_ready[0][cluster_in] = 1'b1;
            end else begin
              wr_buf_ready[b][cluster_in] = 1'b1;
            end
            wr_buf_data_valid[cluster_out] = 1'b1;
          end
        end
      end

      // Update tracker and reset if instruction completed
      if (cluster_buf_valid & cluster_buf_ready) begin
        for (int c=0; c < (NrClusters/NumBuffers); c++) begin
          automatic int unsigned cluster_in = cluster_sel_q * (NrClusters/2) + c;
          if (wr_tracker_q[wr_issue_pnt_q[0]].vl[cluster_in] > NrLanes) begin
            wr_tracker_d[wr_issue_pnt_q[0]].vl[cluster_in] -= NrLanes;
          end else begin
            wr_tracker_d[wr_issue_pnt_q[0]].vl[cluster_in] = '0;
            wr_cluster_completed_d[cluster_in] = 1'b1;
            
            if (wr_cluster_completed_d == '1) begin
              wr_cluster_completed_d = '0;
              wr_buf_data_o[0].last = 1'b1;
              
              // start again from cluster-0
              cluster_sel_d = '0;

              for (int s=0; s < NumStages ; s++) begin
                wr_issue_pnt_d[s] = (wr_issue_pnt_q[s] == NumTrackers-1) ? '0 : wr_issue_pnt_q[s] + 1;
                wr_tracker_d[wr_issue_pnt_q[s]].shuffle_en = '0;
                wr_tracker_d[wr_issue_pnt_q[s]].datapath = SHUFFLE;
              end
              wr_cnt_d -= 1'b1;
            end
          end
        end
      end
    end else begin

      ///// UNIT STRIDE STORES /////
      ///// 8/16/32 bit precisions /////

      // Update vl tracked for every write packet received from clusters
      // All clusters synchronized, use only cluster 0 for handshaking
      for (int c=0; c <NrClusters; c++) begin
        if (axi_resp_o[c].w_ready & axi_req_i[c].w_valid) begin
          automatic logic [$clog2(NrClusters*ClusterAxiDataWidth/8):0] nelem = (ClusterAxiDataWidth/8) >> wr_tracker_q[wr_issue_pnt_q[0]].vew;
          wr_tracker_d[wr_issue_pnt_q[0]].len[c] -= 1;
          if (wr_tracker_q[wr_issue_pnt_q[0]].vl[c] <= nelem) begin
            wr_tracker_d[wr_issue_pnt_q[0]].vl[c] = 0;
          end else begin
            wr_tracker_d[wr_issue_pnt_q[0]].vl[c] -= nelem;
          end
        end
      end
    end
    // For non-{VSXE, VSSE} operations, always use cluster 0
    cluster_w_d = '0;
    lane_w_d = '0;
  end else if (wr_op inside {VSXE, VSSE}) begin

    // Update cluster and lane pointers for write data when data is received for VSXE/VSSE operations
    if (axi_req_i[cluster_w_q].w_valid && axi_resp_o[cluster_w_q].w_ready) begin
      automatic logic [NrClusters-1:0] cluster_completed;
      
      // If we have processed NrLanes data beats and have more data to process, move to next cluster
      lane_w_d = lane_w_q + 1;
      if (lane_w_q == NrLanes - 1) begin
        lane_w_d = '0;
        if (cluster_w_q == NrClusters - 1) begin
          cluster_w_d = '0;
        end else begin
          cluster_w_d = cluster_w_q + 1;
        end
      end

      // Upate the wr tracker for all the stages
      // reduced the wr tracker counter
      wr_tracker_d[wr_issue_pnt_q[0]].vl[cluster_w_q] -= 1;
      for (int c=0; c<NrClusters; c++) begin
        cluster_completed[c] = (wr_tracker_d[wr_issue_pnt_q[0]].vl[c] == 0);
      end
      if (&cluster_completed) begin
        wr_cnt_d -= 1'b1;
        for (int i=0; i<NumStages; i++) begin
          wr_issue_pnt_d[i] = (wr_issue_pnt_q[i] == NumTrackers-1) ? '0 : wr_issue_pnt_q[i] + 1;
        end
        cluster_w_d = '0;
        lane_w_d = '0;
      end
    end
  end
end

/// Output input interface assignments
// Handle Response path
for (genvar c=0; c < NrClusters; c++) begin  
  // Bypass the registers for signals other than R channel
  assign axi_resp_o[c].aw_ready = ((cluster_metadata_i[c].op inside {VSXE, VSSE}) ? (c==cluster_aw_q) ? 1'b1 : 1'b0 : 1'b1) && axi_resp_i[c].aw_ready && !wr_full;

  // If indexed/strided load send ready only to one of the clusters
  assign axi_resp_o[c].ar_ready = ((cluster_metadata_i[c].op inside {VLXE, VLSE}) ? (c==cluster_ar_q) ? 1'b1 : 1'b0 : 1'b1) && axi_resp_i[c].ar_ready && !rd_full;
  
  assign axi_resp_o[c].b_valid = axi_resp_i[c].b_valid;
  assign axi_resp_o[c].b = axi_resp_i[c].b;

  // Take resp from the shuffle or the buffer datapath as necessary, currently prioritize shuffle path
  // Usually responses from both shuffle and buffer paths do not exist simutaneously
  assign axi_resp_o[c].r = r_valid_o[c] ? r_data_out[NumStages-1][c] : axi_resp_buf_out[c].r;  // Copy output resp from last stage
  assign axi_resp_o[c].r_valid = r_valid_o[c] ? ((rd_tracker_q[rd_issue_pnt_q[NumStages-1]].vl[c] == 0) ? 1'b0 : 1'b1) : axi_resp_buf_out[c].r_valid;

end

// Handle Request path
for (genvar c=0; c < NrClusters; c++) begin
  assign axi_req_o[c].aw = axi_req_i[c].aw;
  assign axi_req_o[c].aw_valid = ((cluster_metadata_i[c].op inside {VSXE, VSSE}) ? ((c==cluster_aw_q) ? 1'b1 : 1'b0) : 1'b1) && axi_req_i[c].aw_valid && !wr_full;
  assign axi_req_o[c].ar = axi_req_i[c].ar;
  assign axi_req_o[c].ar_valid = ((cluster_metadata_i[c].op inside {VLXE, VLSE}) ? ((c==cluster_ar_q) ? 1'b1 : 1'b0) : 1'b1) && axi_req_i[c].ar_valid && !rd_full;
  assign axi_req_o[c].b_ready = axi_req_i[c].b_ready;
  
  // Reads
  assign r_ready_i[c] = axi_req_i[c].r_ready;           // From input request, get ready inputs to stream fork

  // Writes
  assign w_data_in[0][c] =  axi_req_i[c].w_valid ? axi_req_i[c].w : '0;

end
assign w_ready[NumStages-1] = &w_ready_shuffle; // The Global Ld-St is ready to receive write packets together. Hence using only cluster-0 's w_ready.

endmodule

module shuffle import rvv_pkg::*; #(
  parameter  int           unsigned NrLanes             = 0,
  parameter  int           unsigned NrClusters          = 0,
  parameter  int           unsigned ClusterAxiDataWidth = 0,
  parameter  type                   T                   = logic,
  parameter  int           unsigned scale               = 0, // In bytes
  parameter  int           unsigned isRead              = 1,
  parameter  int           unsigned isMask              = 0,
  localparam int           unsigned TotalDataWidth      = ClusterAxiDataWidth * NrClusters,
  localparam int           unsigned TotalLanes          = NrClusters * NrLanes,
  localparam int           unsigned BlockSize           = NrLanes << scale,
  localparam int           unsigned NumGatherBlocks     = TotalDataWidth / (BlockSize * NrClusters * 2)         

) (
  input  T  data_i,
  output T  data_o,

  input logic enable_i
);

  logic [TotalDataWidth-1:0] data_in, data_out;
  logic [TotalDataWidth/8-1:0] be_in, be_out;
  
  if (!isRead & !isMask) begin
    // Write shuffle stage for 8/16b elements
    // Also shuffle the byte enable masks
    always_comb begin
      data_o = data_i;

      for (int c=0; c<NrClusters; c++) begin 
        be_in[c*ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8] = data_i[c].strb; 
        data_in[c*ClusterAxiDataWidth +: ClusterAxiDataWidth] = data_i[c].data;
      end

      if (enable_i) begin

        for (int k=0; k<NumGatherBlocks; k++) begin
          for (int i=0; i<NrClusters; i++) begin
            for (int j=0; j<2; j++) begin
              be_out[(k * NrClusters * 2 + j * NrClusters + i)*(BlockSize/8) +: BlockSize/8] = be_in[(k * NrClusters * 2 + 2 * i + j)*(BlockSize/8)  +: BlockSize/8];
              data_out[(k * NrClusters * 2 + j * NrClusters + i)*BlockSize +: BlockSize] = data_in[(k * NrClusters * 2 + 2 * i + j)*BlockSize  +: BlockSize];
            end
          end
        end

        for (int c=0; c<NrClusters; c++) begin
          data_o[c].strb = be_out[c*ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8];
          data_o[c].data = data_out[c*ClusterAxiDataWidth +: ClusterAxiDataWidth];
        end
      end

    end
  end else if (!isRead & isMask) begin
      // Write shuffle stage used for mask writes to memory
      // byte enable gathering is ignored since operating at less than 8-bit blocks here
      always_comb begin
      data_o = data_i;

      for (int c=0; c<NrClusters; c++) begin 
        data_in[c*ClusterAxiDataWidth +: ClusterAxiDataWidth] = data_i[c].data;
      end

      if (enable_i) begin

        for (int k=0; k<NumGatherBlocks; k++) begin
          for (int i=0; i<NrClusters; i++) begin
            for (int j=0; j<2; j++) begin
              data_out[(k * NrClusters * 2 + j * NrClusters + i)*BlockSize +: BlockSize] = data_in[(k * NrClusters * 2 + 2 * i + j)*BlockSize  +: BlockSize];
            end
          end
        end

        for (int c=0; c<NrClusters; c++) begin
          data_o[c].data = data_out[c*ClusterAxiDataWidth +: ClusterAxiDataWidth];
        end
      end
    end
  end else begin
    // Read shuffle for 1/8/16b elements
    always_comb begin
      data_o = data_i;

      if (enable_i) begin
        for (int c=0; c<NrClusters; c++) begin 
          data_in[c*ClusterAxiDataWidth +: ClusterAxiDataWidth] = data_i[c].data;
        end

        for (int k=0; k<NumGatherBlocks; k++) begin
          for (int i=0; i<NrClusters; i++) begin
            for (int j=0; j<2; j++) begin
              data_out[(k * NrClusters * 2 + 2 * i + j)*BlockSize +: BlockSize] = data_in[(k * NrClusters * 2 + j * NrClusters + i)*BlockSize  +: BlockSize];
            end
          end
        end

        for (int c=0; c<NrClusters; c++) begin
          data_o[c].data = data_out[c*ClusterAxiDataWidth +: ClusterAxiDataWidth];
        end
      end
    end 
  end

  if (ClusterAxiDataWidth > 64*NrLanes)
    $error("Cluster BW should not be large than datapath width");

endmodule
