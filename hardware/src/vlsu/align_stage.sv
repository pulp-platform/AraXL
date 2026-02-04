// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// This module does the alignment of data coming from System in stages (each stage shifting by power of 2 bytes)
// Since alignment is done here, the alignment of responses in the cluster VLSU can be avoided

module align_stage import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int           unsigned NrClusters          = 0,
  parameter  int           unsigned AxiDataWidth        = 0,
  parameter  int           unsigned AxiAddrWidth        = 0,
  parameter  type                   axi_ar_t            = logic,
  parameter  type                   axi_r_t             = logic,
  parameter  type                   axi_aw_t            = logic,
  parameter  type                   axi_w_t             = logic,
  parameter  type                   axi_b_t             = logic,
  parameter  type                   axi_req_t           = logic,
  parameter  type                   axi_resp_t          = logic,
  parameter  type                   axi_addr_t          = logic [AxiAddrWidth-1:0],
  parameter  type                   axi_data_t          = logic [AxiDataWidth-1:0],
  localparam int           unsigned NumStages           = $clog2(AxiDataWidth/8)

) (
  // Clock and Reset
  input  logic              clk_i,
  input  logic              rst_ni,

  input cluster_metadata_t  cluster_metadata_i,
  
  input  axi_req_t axi_req_i,
  output axi_req_t axi_req_o, 

  input  axi_resp_t axi_resp_i, 
  output axi_resp_t axi_resp_o
);

localparam int unsigned NumTrackers=8;
typedef logic [$clog2(NumTrackers)-1:0] pnt_t; 
typedef logic [$clog2(NumTrackers):0] cnt_t; 

typedef struct packed {
  axi_addr_t addr;
  vlen_cluster_t len;
  elen_t stride;
  vew_e vew;
  logic is_load;
  logic is_burst;
  cnt_t [NumStages-1:0] num_requests;
  logic [NumStages-1:0] shift_en;
  logic valid;
} req_track_t;

// Tracking read requests
req_track_t [NumTrackers-1:0] tracker_d, tracker_q, tracker_q_del, tracker_d_del;
pnt_t w_pnt_d, w_pnt_q;
pnt_t [NumStages-1:0] r_pnt_d, r_pnt_q, r_pnt_d_del, r_pnt_q_del;
cnt_t cnt_d, cnt_q;

logic [NumStages:0] axi_req_cut_ready;
axi_resp_t [NumStages:0] axi_resp_i_cut;
axi_resp_t [NumStages-1:0] axi_resp_o_cut;

axi_data_t data_d, data_q;
logic data_valid_d, data_valid_q;

typedef logic [AxiDataWidth/8-1:0] be_data_t;
be_data_t [NumStages:0] be_d, be_q;
be_data_t be_final_d, be_final_q;

vlen_cluster_t vl_d, vl_q;

logic last_d, last_q;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    vl_q <= '0;
  end else begin
    vl_q <= vl_d;
  end
end

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    tracker_q         <= '0;
    tracker_q_del     <= '0;
    w_pnt_q           <= '0;
    r_pnt_q           <= '0;
    r_pnt_q_del       <= '0;
    cnt_q             <= '0;
    data_q            <= '0;
    data_valid_q      <= 1'b0;
    be_final_q        <= '0;
    for (int s=0; s<=NumStages; s++) begin 
      be_q[s]         <= '0;
    end
    last_q            <= 1'b0;
  end else begin
    tracker_q         <= tracker_d;
    tracker_q_del     <= tracker_d_del;     
    w_pnt_q           <= w_pnt_d;
    r_pnt_q           <= r_pnt_d;
    r_pnt_q_del       <= r_pnt_d_del;
    cnt_q             <= cnt_d;
    data_q            <= data_d;
    data_valid_q      <= data_valid_d;
    be_final_q        <= be_final_d;
    for (int s=1; s<=NumStages; s++) begin 
      be_q[s]         <= be_d[s];
    end
    be_q[0]           <= '1;
    last_q            <= last_d;
  end
end

for (genvar s=0; s < NumStages; s++) begin 

  stream_register #(
    .T       ( axi_r_t )
  ) i_align_reg_r  (
    .clk_i      ( clk_i                     ),
    .rst_ni     ( rst_ni                    ),
    .clr_i      ( 1'b0                      ),
    .testmode_i ( 1'b0                      ),
    .valid_i    ( axi_resp_i_cut[s].r_valid ),
    .ready_o    ( axi_req_cut_ready[s]    ),
    .data_i     ( axi_resp_i_cut[s].r       ),
    .valid_o    ( axi_resp_o_cut[s].r_valid ),
    .ready_i    ( axi_req_cut_ready[s+1]      ),
    .data_o     ( axi_resp_o_cut[s].r       )
  );
  
  shift #(
      .AxiDataWidth( AxiDataWidth ), 
      .axi_data_t  ( axi_resp_t   ),
      .ShiftVal    ( 1<<(s)       )
    ) i_shift (
      .data_i    ( axi_resp_o_cut[s]                 ),
      .data_o    ( axi_resp_i_cut[s+1]               ),
      .sld_valid ( tracker_q[r_pnt_q[s]].shift_en[s] )
    );
end

// Tracker status
logic tracker_full, tracker_empty;
assign tracker_full = (cnt_q==NumTrackers);
assign tracker_empty = (cnt_q==0);

// Req Channel assignments
assign axi_req_o.aw = axi_req_i.aw;
assign axi_req_o.aw_valid = axi_req_i.aw_valid && axi_resp_o.aw_ready;
// assign axi_req_o.w = axi_req_i.w;
assign axi_req_o.w_valid = axi_req_i.w_valid;
assign axi_req_o.ar = axi_req_i.ar;
assign axi_req_o.ar_valid = axi_req_i.ar_valid && axi_resp_o.ar_ready;
assign axi_req_o.b_ready = axi_req_i.b_ready;

assign axi_req_o.r_ready  = axi_req_cut_ready[0];
assign axi_req_cut_ready[NumStages] = axi_req_i.r_ready;

// Resp channel assignments
// assign axi_resp_o.aw_ready = axi_resp_i.aw_ready && !wr_tracker_full;
assign axi_resp_o.ar_ready = axi_resp_i.ar_ready && !tracker_full; 
assign axi_resp_o.w_ready = axi_resp_i.w_ready;
// assign axi_resp_o.b_valid = axi_resp_i.b_valid;
// assign axi_resp_o.b = axi_resp_i.b;

assign axi_resp_i_cut[0].r = axi_resp_i.r;
assign axi_resp_i_cut[0].r_valid = axi_resp_i.r_valid;

/////////////////////////
// Handle Vector Loads //
/////////////////////////

always_comb begin

  // Initialize state
  w_pnt_d      = w_pnt_q;
  cnt_d        = cnt_q;
  tracker_d    = tracker_q;
  tracker_d_del = tracker_q;
  r_pnt_d      = r_pnt_q;
  r_pnt_d_del  = r_pnt_q;
  data_d       = data_q;
  data_valid_d = data_valid_q;
  vl_d         = vl_q;
  be_d         = be_q;
  last_d       = last_q;

  //////////////////////
  // Request Handling //
  //////////////////////
 
  // If a request arrives, add to tracker.
  // Assign shift enable for different stages
  if (axi_req_i.ar_valid && axi_resp_o.ar_ready) begin
    automatic vew_e     vew          = cluster_metadata_i.vew;  
    automatic int       burst        = axi_req_i.ar.len + 1;
    automatic int       axi_bytes    = AxiDataWidth/8;
    automatic vlen_cluster_t vlen_request = ((burst << $clog2(AxiDataWidth/8)) - (axi_req_i.ar.addr[$clog2(AxiDataWidth/8)-1:0])) >> vew;
    vl_d = vl_q + vlen_request;

    tracker_d[w_pnt_q].addr  = axi_req_i.ar.addr;
    // Track vl expected to receive including misalignment
    tracker_d[w_pnt_q].len   = cluster_metadata_i.vl;
    tracker_d[w_pnt_q].vew   = vew;
    for (int s=0; s < NumStages; s++)
      tracker_d[w_pnt_q].num_requests[s] += 1;
    
    // Logic to handle axi requests split by the global_ldst into a single tracker
    // If the first request
    // Use the address of the first request to track misalignment
    if (vl_q == 0) begin
      for (int s=0; s<NumStages; s++) begin 
        if (axi_req_i.ar.addr & (1<<s)) begin 
          tracker_d[w_pnt_q].shift_en[s] = 1'b1;
        end
      end
    end

    // If last request
    if (vl_d >= cluster_metadata_i.vl) begin
      // Reset vl
      vl_d = 0;
      
      // Update pointers
      w_pnt_d = w_pnt_q + 1;
      if (w_pnt_q == NumTrackers-1) begin 
        w_pnt_d = 0;
      end
      cnt_d = cnt_d + 1;
    end
  end

  ///////////////////////
  // Response Handling //
  ///////////////////////

  ///// Handling unaligned data using byte enable /////

  // If a stage receives a valid packet, shift the byte enable
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s].r_valid) begin
      be_d[s+1] = tracker_q[r_pnt_q[s]].shift_en[s] ? be_q[s] >> (1 << s) : be_q[s];
    end
  end
  be_final_d = be_q[NumStages];

  ///// Handle incoming AXI responses /////

  // Track the previous data packet and along with the byte enable
  // combine the current packet and the previous packet.

  axi_resp_o.r_valid    = 1'b0;
  axi_resp_o.r          = axi_resp_i_cut[NumStages].r;
  axi_resp_o.r.last     = 1'b0;
  
  // For a valid handshake assign to buffer to be used later
  if (axi_resp_i_cut[NumStages].r_valid && axi_req_cut_ready[NumStages]) begin
    // Buffer data in this cycle
    data_d = axi_resp_i_cut[NumStages].r.data;
    data_valid_d     = 1'b1;
    last_d = axi_resp_i_cut[NumStages].r.last;
  end

  // Combine the previous data and the current data packets using byte enable
  if (data_valid_q && axi_req_cut_ready[NumStages]) begin
    // Number of elements in a single AXI transaction
    automatic vlen_t axi_valid_el = (AxiDataWidth/8) >> tracker_q[r_pnt_q_del[NumStages-1]].vew;

    // If misaligned, make sure you have a valid beat in the current cycle
    // or if the transaction is short that is check if previous beat was the last beat
    // Otherwise, we have a valid data if the request is aligned
    automatic logic valid_data = (~be_final_d[AxiDataWidth/8-1] & (axi_resp_i_cut[NumStages].r_valid | last_q)) | be_final_d[AxiDataWidth/8-1];
    for (int b=0; b<AxiDataWidth/8; b++) begin
      axi_resp_o.r.data[b*8 +: 8] = be_final_d[b] ? data_q[b*8 +: 8] : axi_resp_i_cut[NumStages].r.data[b*8 +: 8];
    end

    if (valid_data) begin
      // If valid data, set r_valid
      axi_resp_o.r_valid  = 1'b1;

      // Update vector length counter
      tracker_d[r_pnt_q_del[NumStages-1]].len -= axi_valid_el;

      // If aligned request, set data valid only if available valid beat
      data_valid_d = be_final_d[AxiDataWidth/8-1] ? axi_resp_i_cut[NumStages].r_valid : 1'b1;
    end

    // Use vl from tracker to check if this is the last data packet or not
    // Since using delayed data, using delayed pointer to the tracker
    if (tracker_q[r_pnt_q_del[NumStages-1]].len <= axi_valid_el) begin
      // Last packet
      axi_resp_o.r.last = 1'b1;

      // If the current data is not misaligned and we have a valid data
      // Set valid data for the next subsequent load to avoid bubble
      data_valid_d = be_final_d[AxiDataWidth/8-1] & axi_resp_i_cut[NumStages].r_valid;
      last_d = 1'b0;
    end
  end

  ///// Pointer updates to align stages /////

  // Update read pointer of each stage
  // Once last packet is received by each stage, point to the next tracker.
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s].r.last && axi_resp_o_cut[s].r_valid && axi_req_cut_ready[s+1]) begin
      tracker_d[r_pnt_q[s]].num_requests[s] -= 1;

      if (tracker_d[r_pnt_q[s]].num_requests[s] == 0) begin
        r_pnt_d[s] = r_pnt_q[s] + 1;
        if (r_pnt_q[s] == NumTrackers-1) begin
          r_pnt_d[s] = 0;
        end
        
        // In the last stage, reset the shift enable for all stages
        if (s==(NumStages-1)) begin
          tracker_d[r_pnt_q[s]].shift_en = '0;
          cnt_d = cnt_d - 1;
        end
      end
    end
  end

end

typedef struct packed {
  int len;
} wr_req_track_t;

// Tracking write requests
wr_req_track_t [NumTrackers-1:0] wr_track_d, wr_track_q;
pnt_t wr_pnt_d, wr_pnt_q;
pnt_t wr_commit_pnt_d, wr_commit_pnt_q; 
cnt_t wr_cnt_d, wr_cnt_q;

vlen_cluster_t wr_vl_d, wr_vl_q;
vlen_cluster_t wr_commit_len_d, wr_commit_len_q;

typedef struct packed {
  cnt_t count;
} b_track_t;

b_track_t [NumTrackers-1:0] b_track_d, b_track_q;
pnt_t b_pnt_d, b_pnt_q;
pnt_t b_commit_pnt_d, b_commit_pnt_q;

logic wr_tracker_full;
assign wr_tracker_full = (wr_cnt_q == NumTrackers);
assign axi_resp_o.aw_ready = axi_resp_i.aw_ready && !wr_tracker_full;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    wr_track_q  <= '0;
    wr_pnt_q    <= '0;
    wr_cnt_q    <= '0;
    wr_vl_q     <= '0;
    wr_commit_len_q  <= '0;
    wr_commit_pnt_q  <= '0;
    b_pnt_q     <= '0;
    b_track_q   <= '0;
    b_commit_pnt_q <= '0;
  end else begin
    wr_track_q  <= wr_track_d;
    wr_pnt_q    <= wr_pnt_d;
    wr_cnt_q    <= wr_cnt_d;
    wr_vl_q     <= wr_vl_d;
    wr_commit_len_q   <= wr_commit_len_d;
    wr_commit_pnt_q  <= wr_commit_pnt_d;
    b_pnt_q     <= b_pnt_d;
    b_track_q   <= b_track_d;
    b_commit_pnt_q <= b_commit_pnt_d;
  end
end

// Handling write requests
always_comb begin
  wr_track_d = wr_track_q; 
  wr_pnt_d   = wr_pnt_q;
  wr_cnt_d   = wr_cnt_q;
  wr_vl_d    = wr_vl_q;
  wr_commit_len_d = wr_commit_len_q;
  wr_commit_pnt_d = wr_commit_pnt_q;
  
  b_track_d  = b_track_q;
  b_pnt_d    = b_pnt_q;
  b_commit_pnt_d = b_commit_pnt_q;

  if (axi_req_i.aw_valid && axi_resp_o.aw_ready) begin
    automatic vew_e     vew          = cluster_metadata_i.vew;
    automatic int       burst        = axi_req_i.aw.len + 1;
    automatic int       axi_bytes    = AxiDataWidth/8;
    automatic vlen_cluster_t vlen_request = ((burst << $clog2(AxiDataWidth/8)) - (axi_req_i.aw.addr[$clog2(AxiDataWidth/8)-1:0])) >> vew;
    wr_vl_d = wr_vl_q + vlen_request;
    
    wr_track_d[wr_pnt_q].len           = axi_req_i.aw.len;
    b_track_d[b_pnt_q].count          += 1;

    wr_cnt_d += 1;
    wr_pnt_d = (wr_pnt_q == NumTrackers-1) ? 0 : wr_pnt_q + 1;
    if (wr_vl_d >= cluster_metadata_i.vl) begin
      wr_vl_d = 0;
      b_pnt_d = (b_pnt_q == NumTrackers-1) ? 0 : b_pnt_q + 1;
    end
  end

  axi_req_o.w = axi_req_i.w;
  axi_req_o.w.last = 1'b0;

  if (axi_req_i.w_valid && axi_resp_o.w_ready) begin
    wr_commit_len_d += 1;
    // If received all write packets for the request
    if (wr_commit_len_q == wr_track_d[wr_commit_pnt_q].len) begin
      // Update commit pnt & len
      wr_commit_len_d = '0;
      wr_commit_pnt_d = (wr_commit_pnt_q == NumTrackers-1) ? 0 : wr_commit_pnt_q + 1;
      wr_cnt_d -= 1;
      axi_req_o.w.last = 1'b1;
    end
    
  end

  // Ignore all b responses except last one
  axi_resp_o.b_valid = 1'b0;
  axi_resp_o.b       = '0;

  if (axi_resp_i.b_valid && axi_req_o.b_ready) begin
    b_track_d[b_commit_pnt_q].count -= 1;
    if (b_track_q[b_commit_pnt_q].count==1) begin
      b_commit_pnt_d     = (b_commit_pnt_q == NumTrackers-1) ? 0 : b_commit_pnt_q + 1;
      axi_resp_o.b_valid = 1'b1;
      axi_resp_o.b       = axi_resp_i.b;
    end
  end
end

endmodule

module shift #(
  parameter  int           unsigned AxiDataWidth        = 0,
  parameter  type                   axi_data_t          = logic,
  parameter  int           unsigned ShiftVal            = 0
) (
  input axi_data_t data_i, 
  output axi_data_t data_o,

  input logic sld_valid
);

  always_comb begin 
    data_o = data_i;
    if (sld_valid)
      data_o.r.data = {data_i.r.data[ShiftVal*8-1:0], data_i.r.data[AxiDataWidth-1:ShiftVal*8]};
  end

endmodule
