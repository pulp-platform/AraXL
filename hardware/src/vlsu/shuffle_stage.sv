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
  localparam int           unsigned NumBuffers          = 2
) (
  // Clock and Reset
  input  logic                        clk_i,
  input  logic                        rst_ni,

  input cluster_metadata_t            cluster_metadata_i,
  
  input   axi_req_t  [NrClusters-1:0] axi_req_i,
  output  axi_req_t  [NrClusters-1:0] axi_req_o,

  input   axi_resp_t [NrClusters-1:0] axi_resp_i,
  output  axi_resp_t [NrClusters-1:0] axi_resp_o
);

// There are 2 dapaths in this unit
// 1) Shuffle - to shuffle the data coming from memory to the required cluster based on element width
// 2) Buffer - to buffer the data coming from memory if the element width is 64b and ClusterAxiDataWidth is 32N, since in this case, the data coming from memory needs to be stored and sent in 2 cycles to the clusters. 
// This is only needed for loads, for stores we can just buffer the write data until we have enough data to send to the clusters.

typedef enum  { SHUFFLE, BUFFER } datapath_t;

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

localparam int unsigned NumTrackers=8;
typedef logic [$clog2(NumTrackers)-1:0] pnt_t; 
typedef logic [$clog2(NumTrackers):0] cnt_t; 

req_track_t [NumTrackers-1:0] rd_tracker_d, rd_tracker_q;
pnt_t rd_accept_pnt_d, rd_accept_pnt_q;
pnt_t [NumStages-1:0] rd_issue_pnt_d, rd_issue_pnt_q;
cnt_t rd_cnt_d, rd_cnt_q;

req_track_t [NumTrackers-1:0] wr_tracker_d, wr_tracker_q;
pnt_t wr_accept_pnt_d, wr_accept_pnt_q;
pnt_t [NumStages-1:0] wr_issue_pnt_d, wr_issue_pnt_q;
cnt_t wr_cnt_d, wr_cnt_q;

typedef axi_r_t [NrClusters-1:0] stage_r_t;
stage_r_t [NumStages-1:0] r_data_in, r_data_out;

typedef axi_w_t [NrClusters-1:0] stage_w_t; 
stage_w_t [NumStages-1:0] w_data_in, w_data_out;

logic [NumStages-1:0] r_valid, r_ready, w_valid, w_ready;
logic [NumStages-1:0] r_shuffle_en, w_shuffle_en;

logic [NrClusters-1:0] r_ready_i, r_valid_o;
logic [NrClusters-1:0] w_valid_i, w_ready_o;

logic rd_full, wr_full;
assign rd_full = (rd_cnt_q == NumTrackers);
assign wr_full = (wr_cnt_q == NumTrackers);

logic [$clog2(NrLanes):0] lane_ar_d, lane_ar_q;
logic [$clog2(NrClusters):0] cluster_ar_d, cluster_ar_q;

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

// To handle cases where write data does not come simultaneously 
// from all the clusters
stream_join #(
  .N_INP(NrClusters)
) i_cluster_stream_join (
  .inp_ready_o(w_ready_o),
  .inp_valid_i(w_valid_i),
  .oup_ready_i(w_ready[0]),
  .oup_valid_o(w_valid[0])
);

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

  // Shuffling write data
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
assign rd_datapath = rd_tracker_q[rd_issue_pnt_q[0]].datapath;
ara_op_e rd_op;
assign rd_op = rd_tracker_q[rd_issue_pnt_q[0]].op;

// Write packets
logic [NrClusters-1:0] [ClusterAxiDataWidth*2-1:0]  wrbuf_d, wrbuf_q;
logic [NrClusters-1:0] [(ClusterAxiDataWidth*2/8)-1:0]  wrbuf_be_d, wrbuf_be_q;
axi_req_t [NrClusters-1:0]  axi_req_buf_out;

logic [$clog2(NrClusters)-1:0] wrbuf_pnt_q, wrbuf_pnt_d;
logic [NrClusters-1:0] wr_shift_d, wr_shift_q;
logic [NrClusters-1:0] wrbuf_valid, wrbuf_valid_q;
logic [NrClusters-1:0] wrbuf_full, wrbuf_full_q;
logic wr_out_ready, wr_out_valid;

datapath_t wr_in_state;
assign wr_in_state = wr_tracker_q[wr_issue_pnt_q[0]].datapath;

logic [NrClusters-1:0] wr_cluster_completed, rd_cluster_completed_d, rd_cluster_completed_q;
logic [NumBuffers-1:0] rd_buffer_completed_d, rd_buffer_completed_q;
logic [NumBuffers-1:0] wr_buffer_completed_d, wr_buffer_completed_q; 

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
    wr_buffer_completed_q <= '0; 
    rd_buffer_completed_q <= '0;
    cluster_ar_q <= '0;
    lane_ar_q <= '0;
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
    wr_buffer_completed_q <= wr_buffer_completed_d; 
    rd_buffer_completed_q <= rd_buffer_completed_d;
    cluster_ar_q <= cluster_ar_d;
    lane_ar_q <= lane_ar_d;
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
  
  if (axi_req_i[cluster_ar_q].ar_valid & axi_resp_o[cluster_ar_q].ar_ready) begin
    // Store element width
    rd_tracker_d[rd_accept_pnt_q].vew = cluster_metadata_i.vew;
    rd_tracker_d[rd_accept_pnt_q].use_eew1 = cluster_metadata_i.use_eew1;
    // Track number of beats and vl
    for (int c=0; c<NrClusters; c++) begin
      automatic int unsigned vl_tot = cluster_metadata_i.use_eew1 ? cluster_metadata_i.vl << 3 : cluster_metadata_i.vl;
      automatic int unsigned vl_rem = vl_tot & (TotalNrLanes - 1);
      automatic int unsigned vl_base = vl_tot >> $clog2(TotalNrLanes);
      automatic int unsigned vl_rem_diff = vl_rem - (c * NrLanes);
      automatic int unsigned vl = (vl_base << $clog2(NrLanes)) + ((vl_rem >= (c + 1) * NrLanes) ? NrLanes : (vl_rem >= (c * NrLanes)) ? vl_rem_diff : '0);

      rd_tracker_d[rd_accept_pnt_q].len[c] = axi_req_i[cluster_ar_q].ar.len+1;
      rd_tracker_d[rd_accept_pnt_q].vl[c] = cluster_metadata_i.use_eew1 ? (vl < 8) ? 1 : vl >> 3 : vl;
    end
    // Update pnt to accept next request
    rd_accept_pnt_d = (rd_accept_pnt_q == NumTrackers-1) ? '0 : rd_accept_pnt_q + 1;
    rd_cnt_d += 1;
    // To enable certain shuffle stages based on element width
    for (int s=0; s<NumStages; s++) begin
      rd_tracker_d[rd_accept_pnt_q].shuffle_en[s] = cluster_metadata_i.use_eew1 ? 1'b1 : (s >= (3 + cluster_metadata_i.vew)) ? 1'b1 : 1'b0;
    end
    // To enable buffer for 64b element widths
    rd_tracker_d[rd_accept_pnt_q].datapath = NumStages < (3 + cluster_metadata_i.vew) ? BUFFER : SHUFFLE;
    rd_tracker_d[rd_accept_pnt_q].second_buffer_unused = cluster_metadata_i.vl <= (NrLanes * NrClusters / 2) && rd_tracker_d[rd_accept_pnt_q].datapath;
    rd_tracker_d[rd_accept_pnt_q].op = cluster_metadata_i.op;

    // If it is a VLXE request, take from the desired cluster
    // and switch clusters for every NrLanes requests
    if (cluster_metadata_i.op == VLXE) begin
      lane_ar_d += 1;
      if (lane_ar_q == NrLanes - 1) begin
        cluster_ar_d += 1;
        if (cluster_ar_q == NrClusters - 1) begin
          cluster_ar_d = '0;
        end
        lane_ar_d = '0;
      end
    end else begin
      lane_ar_d = '0;
      cluster_ar_d = '0;
    end
  end

  if (axi_req_i[0].aw_valid & axi_resp_o[0].aw_ready) begin
    // Store element width
    wr_tracker_d[wr_accept_pnt_q].vew = cluster_metadata_i.vew;
    wr_tracker_d[wr_accept_pnt_q].use_eew1 = cluster_metadata_i.use_eew1;
    // Track number of beats and vl
    for (int c=0; c<NrClusters; c++) begin
      automatic int unsigned vl_tot = cluster_metadata_i.vl;
      automatic int unsigned vl_rem = vl_tot & (TotalNrLanes - 1);
      automatic int unsigned vl_base = vl_tot >> $clog2(TotalNrLanes);
      automatic int unsigned vl_rem_diff = vl_rem - (c * NrLanes);
      automatic int unsigned vl = (vl_base << $clog2(NrLanes)) + ((vl_rem >= (c + 1) * NrLanes) ? NrLanes : (vl_rem >= (c * NrLanes)) ? vl_rem_diff : '0);      

      wr_tracker_d[wr_accept_pnt_q].vl[c] = vl;
      wr_tracker_d[wr_accept_pnt_q].len[c] = axi_req_i[0].aw.len+1;
    end
    // Update pnt to accept next request
    wr_accept_pnt_d = (wr_accept_pnt_q == NumTrackers-1) ? '0 : wr_accept_pnt_q + 1; 
    wr_cnt_d += 1;
    // To enable certain shuffle stages based on element width
    for (int s=0; s<NumStages; s++) begin
      wr_tracker_d[wr_accept_pnt_q].shuffle_en[s] = cluster_metadata_i.use_eew1 ? 1'b1 : (((NumStages -s -1) >= (3 + cluster_metadata_i.vew)) ? 1'b1 : 1'b0);
    end
    // To enable buffer for 64b element widths
    wr_tracker_d[wr_accept_pnt_q].datapath = NumStages < (3 + cluster_metadata_i.vew) ? BUFFER : SHUFFLE;
    wr_tracker_d[wr_accept_pnt_q].second_buffer_unused = cluster_metadata_i.vl <= (NrLanes * NrClusters / 2) && wr_tracker_d[wr_accept_pnt_q].datapath;
    wr_tracker_d[wr_accept_pnt_q].op = cluster_metadata_i.op;
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

  rd_cluster_completed_d = rd_cluster_completed_q;
  rd_buffer_completed_d = rd_buffer_completed_q;

  if (((rd_datapath == BUFFER) || (|buf_valid_q)) && (rd_op != VLXE)) begin

    // If have a valid handshake on response add to the buffer
    // If have a valid response from L2 after aligning buffer it first pointed by rdbuf_pnt_q
    // Set we have a valid data
    if (axi_resp_i[0].r_valid && r_ready_buf_q) begin
      for (int c=0; c<NrClusters; c++) begin
        buf_d[rdbuf_pnt_q][c] = axi_resp_i[c].r;
      end
      buf_valid_d[rdbuf_pnt_q] = 1'b1;
      rdbuf_pnt_d = (rdbuf_pnt_q == 1'b1) ? 1'b0 : 1'b1;
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
            axi_resp_buf_out[cl].r_valid = 1'b1;
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
          if (&(rd_cluster_completed_d[b*(NrClusters/NumBuffers)+:(NrClusters/NumBuffers)])) begin
            // Change to next instruction for the particular buffer
            // Since each buffer can complete at different times, we maintain a different instruction pointer
            rd_issue_pnt_d[b] = (rd_issue_pnt_q[b] == NumTrackers-1) ? '0 : rd_issue_pnt_q[b] + 1;
            
            // clear buffer for the next instruction
            buf_valid_d[b] = 1'b0;
            shift_d[b] = 1'b0;

            // Set the clusters corresponding to the buffer as completed for the instruction
            rd_cluster_completed_d[b*(NrClusters/NumBuffers)+:(NrClusters/NumBuffers)] = '0;

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
  end

  ///// INDEXED LOADS /////

  // If indexed operation, just forward the data coming from GLSU without shuffling or buffering
  if (rd_op == VLXE) begin
    automatic logic cluster_valid = 1'b0;
    for (int c=0; c < NrClusters; c++) begin
      axi_resp_buf_out[c].r_valid = axi_resp_i[c].r_valid;
      axi_resp_buf_out[c].r = axi_resp_i[c].r;
      cluster_valid |= axi_resp_i[c].r_valid;
    end

    // Do this if we have any valid response
    if (cluster_valid) begin
      rd_cnt_d -= 1;
      for (int i=0; i <NumStages; i++) begin
        rd_issue_pnt_d[i] = rd_issue_pnt_q[i] + 1;
      end
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

  wr_buffer_completed_d = wr_buffer_completed_q;

  if (wr_in_state) begin
    // If a valid write packet, add it to the buffer
    for (int c=0; c < NrClusters; c++) begin
      automatic axi_req_t req = axi_req_i[c];
      if (req.w_valid & ~wrbuf_full_q[c]) begin
        wrbuf_d[c][wr_shift_d[c] * ClusterAxiDataWidth +: ClusterAxiDataWidth] = req.w.data;
        wrbuf_be_d[c][wr_shift_d[c] * ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8] = req.w.strb;
        wr_shift_d[c] += 1'b1;
        wrbuf_valid[c] = 1'b1;
        if (wr_shift_q[c] == 1'b1) begin
          wrbuf_full[c] = 1'b1;
          wr_shift_d[c] = 1'b0;
        end
      end
    end
    
    // We take data from half of the clusters and send a response
    for (int c=0; c < (NrClusters/2); c++) begin
      automatic int cluster = wrbuf_pnt_q + c;

      // Check if we have 2 buffers filled for every cluster
      // Or if the cluster has less number of elements, just check if we have something valid
      // If neither, check if the cluster has nothing to send, i.e. it has lesser number of elements or no elements
      // at all and has completed the instruction
      // In this case, use wr_cluster_completed to send a fake valid signal
      wr_out_valid &= (wrbuf_full[cluster] || 
                      (wrbuf_valid[cluster] && (wr_tracker_q[wr_issue_pnt_q[0]].vl[wrbuf_pnt_q + c] <= 2))) 
                       ? 1'b1 : wr_cluster_completed[cluster];
      if (wrbuf_valid[cluster]) begin
        // If have a valid data, assign it to the output
        for (int b=0; b < 2; b++) begin
          wr_out_ready &= axi_resp_i[c*2 + b].w_ready;
          axi_req_buf_out[c*2 + b].w.data = wrbuf_d   [cluster][b*ClusterAxiDataWidth   +: ClusterAxiDataWidth  ];
          axi_req_buf_out[c*2 + b].w.strb = wrbuf_be_d[cluster][b*ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8];
        end
      end
    end

    // If a valid handshake, if the Global ld-st stage is ready to receive data on all ports
    if (wr_out_ready & wr_out_valid) begin
      automatic logic [$clog2(NrClusters*ClusterAxiDataWidth/8):0] nelem = NrLanes;
      
      // For a valid handshake set valid to 1
      // All interfaces are send together in a synchronized way
      for (int c=0; c<NrClusters; c++) begin
        axi_req_buf_out[c].w_valid = 1'b1;
      end

      // Next we want to take data from the rest of the clusters
      wrbuf_pnt_d = wrbuf_pnt_q + (NrClusters/NumBuffers);
      
      for (int c=0; c < (NrClusters/NumBuffers); c++) begin
        // Since data from the buffer has been used, set buffer not valid
        wrbuf_valid[wrbuf_pnt_q + c] = 1'b0;
        wrbuf_be_d[wrbuf_pnt_q + c] = '0;
        wrbuf_full [wrbuf_pnt_q + c] = 1'b0;
        wr_tracker_d[wr_issue_pnt_q[0]].len[wrbuf_pnt_q + c] -= 2;

        if (wr_tracker_q[wr_issue_pnt_q[0]].vl[wrbuf_pnt_q + c] <= nelem) begin 
          // If this is the last data sent from the cluster
          // start writing from offset of 0
          wr_tracker_d[wr_issue_pnt_q[0]].vl[wrbuf_pnt_q + c] = '0;
          wr_shift_d[(wrbuf_pnt_q + c)] = '0;
          
          // Since cluster 0 always has the most elements, just set once for cluster0
          if (c==0)
            wr_buffer_completed_d[wrbuf_pnt_q >> $clog2(NrClusters/2)] = 1'b1;

        end else begin 
          wr_tracker_d[wr_issue_pnt_q[0]].vl[wrbuf_pnt_q + c] -= nelem;
        end
      end

      // If the instruction has been completed
      // Or if there are less elements that the second buffer is not used
      if ((&wr_buffer_completed_d) || 
          (wr_buffer_completed_d == 2'b01 && wr_tracker_d[wr_issue_pnt_q[0]].second_buffer_unused)) begin
          wr_buffer_completed_d = '0;
          wrbuf_pnt_d = '0;
          axi_req_buf_out[0].w.last = 1'b1;
        end
    end

    // If the last cluster sends the data, remove request from tracker
    if (axi_req_buf_out[0].w_valid & axi_req_buf_out[0].w.last) begin
      for (int s=0; s < NumStages ; s++) begin
        wr_issue_pnt_d[s] = (wr_issue_pnt_q[s] == NumTrackers-1) ? '0 : wr_issue_pnt_q[s] + 1;
        wr_tracker_d[wr_issue_pnt_q[s]].shuffle_en = '0;
        wr_tracker_d[wr_issue_pnt_q[s]].datapath = SHUFFLE;
      end
      wr_cnt_d -= 1'b1;
    end
  end else begin

    // Update vl tracked for every write packet received from clusters
    // All clusters synchronized, use only cluster 0 for handshaking
    for (int c=0; c <NrClusters; c++) begin
      if (axi_resp_o[c].w_ready & axi_req_i[c].w_valid) begin
        automatic logic [$clog2(NrClusters*ClusterAxiDataWidth/8):0] nelem = (ClusterAxiDataWidth/8) >> wr_tracker_q[wr_issue_pnt_q[0]].vew;
        // wr_tracker_d[wr_issue_pnt_q[0]].vl[c] -= (wr_tracker_q[wr_issue_pnt_q[0]].vl[c] >= nelem) ? nelem : '0;
        wr_tracker_d[wr_issue_pnt_q[0]].len[c] -= 1;
        if (wr_tracker_q[wr_issue_pnt_q[0]].vl[c] <= nelem) begin
          wr_tracker_d[wr_issue_pnt_q[0]].vl[c] = 0;
        end else begin
          wr_tracker_d[wr_issue_pnt_q[0]].vl[c] -= nelem;
        end
      end
    end
  end

end

/// Output input interface assignments
// Handle Response path
for (genvar c=0; c < NrClusters; c++) begin  
  // Bypass the registers for signals other than R channel
  assign axi_resp_o[c].aw_ready = axi_resp_i[c].aw_ready && !wr_full;

  // If indexed load send ready only to one of the clusters
  assign axi_resp_o[c].ar_ready = ((cluster_metadata_i.op == VLXE) ? (c==cluster_ar_q) ? 1'b1 : 1'b0 : axi_resp_i[c].ar_ready) && !rd_full;
  
  assign axi_resp_o[c].b_valid = axi_resp_i[c].b_valid;
  assign axi_resp_o[c].b = axi_resp_i[c].b;

  // Reads  
  assign r_data_in[0][c] = (rd_datapath == BUFFER) ? '0 : axi_resp_i[c].r;             // Copy input resp to first stage

  // Take resp from the shuffle or the buffer datapath as necessary
  assign axi_resp_o[c].r = r_valid_o[c] ? r_data_out[NumStages-1][c] : axi_resp_buf_out[c].r;  // Copy output resp from last stage
  assign axi_resp_o[c].r_valid = r_valid_o[c] ? ((rd_tracker_q[rd_issue_pnt_q[NumStages-1]].vl[c] == 0) ? 1'b0 : 1'b1) : axi_resp_buf_out[c].r_valid;
  
  // Writes
  assign axi_resp_o[c].w_ready = wr_in_state ? ~wrbuf_full_q[c] : w_ready_o[c];          // Copy ready from stream join output to response

end

// Valid input signal to use shuffle datapath
assign r_valid[0]   = (rd_datapath == BUFFER) ? 1'b0 : (rd_tracker_q[rd_issue_pnt_q[0]].op == VLXE) ? 1'b0 : axi_resp_i[0].r_valid;

// Handle Request path
for (genvar c=0; c < NrClusters; c++) begin
  assign axi_req_o[c].aw = axi_req_i[c].aw;
  assign axi_req_o[c].aw_valid = axi_req_i[c].aw_valid;
  assign axi_req_o[c].ar = axi_req_i[c].ar;
  assign axi_req_o[c].ar_valid = (cluster_metadata_i.op == VLXE) ? ((c==cluster_ar_q) ? axi_req_i[c].ar_valid : 1'b0) : axi_req_i[c].ar_valid;
  assign axi_req_o[c].b_ready = axi_req_i[c].b_ready;
  
  // Reads
  assign axi_req_o[c].r_ready = (rd_datapath == BUFFER) ? r_ready_buf_q : r_ready[0];
  assign r_ready_i[c] = axi_req_i[c].r_ready;           // From input request, get ready inputs to stream fork

  // Writes
  assign w_data_in[0][c] = wr_in_state ? '0 : axi_req_i[c].w_valid ? axi_req_i[c].w : '0;              // Copy input write data to first shuffle stage

  // If other cluster have completed writes, and cluster 0 has a write packet remaining, assume a fake write valid to the stream fork module
  assign wr_cluster_completed[c] = (wr_cnt_q > 0) && (wr_tracker_q[wr_issue_pnt_q[0]].vl[c] == '0);
  
  assign w_valid_i[c]    = (wr_cluster_completed[c] & axi_req_i[0].w_valid) ? 1'b1 : wr_in_state? 1'b0 : axi_req_i[c].w_valid;   // Copy valid signals to stream join
   
  assign axi_req_o[c].w       = w_valid[NumStages-1] ? w_data_out[NumStages-1][c] : axi_req_buf_out[c].w;   // Copy last stage data to req output
  assign axi_req_o[c].w_valid = w_valid[NumStages-1] ? 1'b1  : axi_req_buf_out[c].w_valid;   // valid signal is the output valid of stream join

end
assign w_ready[NumStages-1] = axi_resp_i[0].w_ready; // The Global Ld-St is ready to receive write packets together. Hence using only cluster-0 's w_ready.

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
