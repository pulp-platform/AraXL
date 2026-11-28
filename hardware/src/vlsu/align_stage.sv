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
  localparam int           unsigned AxiDataBytes        = AxiDataWidth/8,
  localparam int           unsigned NumStages           = $clog2(AxiDataBytes)

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
  ara_op_e op;
} req_track_t;

// Tracking read requests
req_track_t [NumTrackers-1:0] tracker_d, tracker_q, tracker_q_del, tracker_d_del;
pnt_t rd_req_pnt_d, rd_req_pnt_q;
pnt_t [NumStages-1:0] rd_resp_pnt_d, rd_resp_pnt_q, rd_resp_pnt_d_del, rd_resp_pnt_q_del;
cnt_t rd_cnt_d, rd_cnt_q;

logic [NumStages:0] axi_req_cut_ready;
axi_resp_t [NumStages:0] axi_resp_i_cut;
axi_resp_t [NumStages-1:0] axi_resp_o_cut;

axi_data_t data_d, data_q;
logic data_valid_d, data_valid_q;

typedef logic [AxiDataBytes-1:0] be_data_t;
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
    rd_req_pnt_q      <= '0;
    rd_resp_pnt_q     <= '0;
    rd_resp_pnt_q_del <= '0;
    rd_cnt_q          <= '0;
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
    rd_req_pnt_q      <= rd_req_pnt_d;
    rd_resp_pnt_q     <= rd_resp_pnt_d;
    rd_resp_pnt_q_del <= rd_resp_pnt_d_del;
    rd_cnt_q          <= rd_cnt_d;
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
    .ready_o    ( axi_req_cut_ready[s]      ),
    .data_i     ( axi_resp_i_cut[s].r       ),
    .valid_o    ( axi_resp_o_cut[s].r_valid ),
    .ready_i    ( axi_req_cut_ready[s+1]    ),
    .data_o     ( axi_resp_o_cut[s].r       )
  );
  
  shift #(
      .AxiDataWidth( AxiDataWidth ), 
      .axi_data_t  ( axi_resp_t   ),
      .ShiftVal    ( 1<<(s)       )
    ) i_shift (
      .data_i    ( axi_resp_o_cut[s]                       ),
      .data_o    ( axi_resp_i_cut[s+1]                     ),
      .sld_valid ( tracker_q[rd_resp_pnt_q[s]].shift_en[s] )
    );
end

// Tracker status
logic tracker_full, tracker_empty;
assign tracker_full = (rd_cnt_q==NumTrackers);
assign tracker_empty = (rd_cnt_q==0);

// Req Channel assignments
assign axi_req_o.aw_valid = axi_req_i.aw_valid && axi_resp_o.aw_ready;
assign axi_req_o.ar = axi_req_i.ar;
assign axi_req_o.ar_valid = axi_req_i.ar_valid && axi_resp_o.ar_ready;
assign axi_req_o.b_ready = axi_req_i.b_ready;

assign axi_req_o.r_ready  = axi_req_cut_ready[0];

// Resp channel assignments
assign axi_resp_o.ar_ready = axi_resp_i.ar_ready && !tracker_full; 

assign axi_resp_i_cut[0].r = axi_resp_i.r;
assign axi_resp_i_cut[0].r_valid = axi_resp_i.r_valid;

/////////////////////////
// Handle Vector Loads //
/////////////////////////

logic is_load_aligned, is_load_resp_valid;

always_comb begin

  // Initialize state
  rd_req_pnt_d       = rd_req_pnt_q;
  rd_cnt_d           = rd_cnt_q;
  tracker_d          = tracker_q;
  tracker_d_del      = tracker_q;
  rd_resp_pnt_d      = rd_resp_pnt_q;
  rd_resp_pnt_d_del  = rd_resp_pnt_q;
  data_d             = data_q;
  data_valid_d       = data_valid_q;
  vl_d               = vl_q;
  be_d               = be_q;
  last_d             = last_q;

  //////////////////////
  // Request Handling //
  //////////////////////
 
  // If a request arrives, add to tracker.
  // Assign shift enable for different stages
  if (axi_req_i.ar_valid && axi_resp_o.ar_ready) begin
    automatic vew_e     vew          = cluster_metadata_i.vew;  
    automatic int       burst        = axi_req_i.ar.len + 1;
    automatic int       axi_bytes    = AxiDataBytes;
    automatic vlen_cluster_t vlen_request = ((burst << $clog2(AxiDataBytes)) - (axi_req_i.ar.addr[$clog2(AxiDataBytes)-1:0])) >> vew;
    vl_d = vl_q + vlen_request;

    tracker_d[rd_req_pnt_q].addr  = axi_req_i.ar.addr;
    // Track vl expected to receive including misalignment
    // Set len only the for the first ar req received in case the req was split by global_ldst unit
    tracker_d[rd_req_pnt_q].len   = (tracker_q[rd_req_pnt_q].num_requests[0] == 0) ? cluster_metadata_i.vl : tracker_q[rd_req_pnt_q].len;
    tracker_d[rd_req_pnt_q].vew   = vew;
    tracker_d[rd_req_pnt_q].op    = cluster_metadata_i.op;
    for (int s=0; s < NumStages; s++)
      tracker_d[rd_req_pnt_q].num_requests[s] += 1;
    
    // Logic to handle axi requests split by the global_ldst into a single tracker
    // If the first request
    // Use the address of the first request to track misalignment
    if (vl_q == 0) begin
      for (int s=0; s<NumStages; s++) begin 
        if (axi_req_i.ar.addr & (1<<s)) begin 
          tracker_d[rd_req_pnt_q].shift_en[s] = 1'b1;
        end
      end
    end

    // If last request
    if (vl_d >= cluster_metadata_i.vl || (cluster_metadata_i.op inside {VLXE, VLSE})) begin
      // Reset vl
      vl_d = 0;
      
      // Update pointers
      rd_req_pnt_d = rd_req_pnt_q + 1;
      if (rd_req_pnt_q == NumTrackers-1) begin 
        rd_req_pnt_d = 0;
      end
      rd_cnt_d = rd_cnt_d + 1;
    end
  end

  ///////////////////////
  // Response Handling //
  ///////////////////////

  ///// Handling unaligned data using byte enable /////

  be_final_d = be_q[NumStages];

  is_load_aligned = be_final_d[AxiDataBytes-1];
  is_load_resp_valid = axi_resp_i_cut[NumStages].r_valid;

  ///// Handle incoming AXI responses /////

  // Track the previous data packet and along with the byte enable
  // combine the current packet and the previous packet.

  axi_resp_o.r_valid    = 1'b0;
  axi_resp_o.r          = axi_resp_i_cut[NumStages].r;
  axi_resp_o.r.last     = 1'b0;

  axi_req_cut_ready[NumStages] = axi_req_i.r_ready;
  
  // For a valid handshake assign to buffer to be used later
  if (is_load_resp_valid && axi_req_i.r_ready) begin
    // Buffer data in this cycle
    data_d = axi_resp_i_cut[NumStages].r.data;
    data_valid_d     = 1'b1;
    // Only propagate last from AXI when this is the final split request
    // tracked at the last align stage.
    last_d = axi_resp_i_cut[NumStages].r.last && (tracker_q[rd_resp_pnt_q[NumStages-1]].num_requests[NumStages-1] == 1);
  end

  if (!(tracker_q[rd_resp_pnt_q[NumStages-1]].op inside {VLXE, VLSE})) begin
    // Combine the previous data and the current data packets using byte enable
    if (data_valid_q && axi_req_i.r_ready) begin
      // Number of elements in a single AXI transaction
      automatic vlen_t axi_valid_el = AxiDataBytes >> tracker_q[rd_resp_pnt_q_del[NumStages-1]].vew;

      // If misaligned, make sure you have a valid beat in the current cycle
      // or if the transaction is short that is check if previous beat was the last beat
      // Otherwise, we have a valid data if the request is aligned
      automatic logic valid_data = (~is_load_aligned & (is_load_resp_valid | last_q)) | is_load_aligned;
      for (int b=0; b<AxiDataBytes; b++) begin
        axi_resp_o.r.data[b*8 +: 8] = be_final_d[b] ? data_q[b*8 +: 8] : axi_resp_i_cut[NumStages].r.data[b*8 +: 8];
      end

      if (valid_data) begin
        // If valid data, set r_valid
        axi_resp_o.r_valid  = 1'b1;

        // Update vector length counter
        tracker_d[rd_resp_pnt_q_del[NumStages-1]].len -= axi_valid_el;

        // If aligned request, set data valid only if available valid beat
        data_valid_d = is_load_aligned ? is_load_resp_valid : 1'b1;
      
        // Use vl from tracker to check if this is the last data packet or not
        // Since using delayed data, using delayed pointer to the tracker
        if (tracker_q[rd_resp_pnt_q_del[NumStages-1]].len <= axi_valid_el) begin
          // Last packet
          axi_resp_o.r.last = 1'b1;

          // If the current data is not misaligned and we have a valid data
          // Set valid data for the next subsequent load to avoid bubble
          data_valid_d = is_load_aligned & is_load_resp_valid;

          // If misaligned stall it for 1 cycle
          if (!is_load_aligned & last_q) begin
            axi_req_cut_ready[NumStages] = 1'b0;
          end
          last_d = 1'b0;
        end
      end
    end else if (data_valid_q & !axi_req_i.r_ready) begin
      // Maintain the delayed pointer in the case data cannot be sent out
      rd_resp_pnt_d_del[NumStages-1] = rd_resp_pnt_q_del[NumStages-1];
    end
  end else begin
    // Indexed operation
    axi_resp_o.r_valid  = data_valid_d;
    axi_resp_o.r.last   = last_d;
    axi_resp_o.r.data = data_d;
    last_d = 1'b0;
    data_valid_d = 1'b0;
  end

  // If a stage receives a valid packet, shift the byte enable
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s].r_valid & axi_req_cut_ready[NumStages]) begin
      be_d[s+1] = tracker_q[rd_resp_pnt_q[s]].shift_en[s] ? be_q[s] >> (1 << s) : be_q[s];
    end
  end

  ///// Pointer updates to align stages /////

  // Update read pointer of each stage
  // Once last packet is received by each stage, point to the next tracker.
  for (int s=0; s < NumStages; s++) begin
    if (axi_resp_o_cut[s].r.last && axi_resp_o_cut[s].r_valid && axi_req_cut_ready[s+1]) begin
      tracker_d[rd_resp_pnt_q[s]].num_requests[s] -= 1;

      if (tracker_d[rd_resp_pnt_q[s]].num_requests[s] == 0) begin
        rd_resp_pnt_d[s] = rd_resp_pnt_q[s] + 1;
        if (rd_resp_pnt_q[s] == NumTrackers-1) begin
          rd_resp_pnt_d[s] = 0;
        end
        
        // In the last stage, reset the shift enable for all stages
        if (s==(NumStages-1)) begin
          tracker_d[rd_resp_pnt_q[s]].shift_en = '0;
          rd_cnt_d = rd_cnt_d - 1;
        end
      end
    end
  end

end

//////////////////////////
// Handle Vector Stores //
//////////////////////////

typedef struct packed {
  int len;
  ara_op_e op;
  axi_addr_t addr;
  logic [$clog2(AxiDataBytes)-1:0] off; // store offset
  logic flush; // is a flush expected for this instruction
  logic is_last_aw;
  logic [$clog2(AxiDataBytes):0] lastb;
  int unsigned in_base;  // number of input beats already consumed by the earlier AWs of the same instruction
  int unsigned in_total; // total number of input beats of the whole instruction
} wr_req_track_t;

// Tracking write requests
wr_req_track_t [NumTrackers-1:0] wr_track_d, wr_track_q;
wr_req_track_t wr_head;
pnt_t wr_pnt_d, wr_pnt_q;
pnt_t wr_commit_pnt_d, wr_commit_pnt_q; 
cnt_t wr_cnt_d, wr_cnt_q;
logic wr_tracker_full, wr_tracker_empty;

assign wr_head             = wr_track_q[wr_commit_pnt_q];
assign wr_tracker_full     = (wr_cnt_q == NumTrackers);
assign wr_tracker_empty    = (wr_cnt_q == 0);
assign axi_resp_o.aw_ready = axi_resp_i.aw_ready && !wr_tracker_full;

vlen_cluster_t wr_vl_d, wr_vl_q;
vlen_cluster_t wr_commit_len_d, wr_commit_len_q;
vlen_cluster_t wr_vl_sum; // number of elements covered by all AWs of this instruction including the one being accepted now

// Offset for current store instruction
logic [$clog2(AxiDataBytes)-1:0] off_inst_d, off_inst_q;

typedef struct packed {
  cnt_t count;
} b_track_t;

b_track_t [NumTrackers-1:0] b_track_d, b_track_q;
pnt_t b_pnt_d, b_pnt_q;
pnt_t b_commit_pnt_d, b_commit_pnt_q;

logic [$bits(cluster_metadata_i.vl)+2:0] nbytes; // total nb of bytes to write
logic [$clog2(AxiDataBytes):0] lastb; // number of valid bytes in the last input beat of instruction
assign nbytes = cluster_metadata_i.vl << cluster_metadata_i.vew;
assign lastb  = ((nbytes - 1'b1) % AxiDataBytes) + 1'b1;

logic is_first_aw, is_last_aw; // is it the first/last aw of instruction
logic flush_inst_d, flush_inst_q;
logic flush_active_d, flush_active_q;
logic flush_only_head, flush_now;

// Special case :
// If the flush causes a 4kb page crossing, the flush beat
// will not be preceded by any other beat
// we must look at the tracker instead
assign flush_only_head = !wr_tracker_empty && wr_head.flush && wr_head.is_last_aw && (wr_head.len == 0);

// Dictates when flush is injected
assign flush_now = flush_active_q || flush_only_head;

int unsigned wr_in_base_d, wr_in_base_q;
logic [AxiDataBytes-1:0] trim_mask;
// Only last real input beat of a VSE can be partial, trim strobes beyond real byte count
always_comb begin
  trim_mask = '1;
  if (wr_track_q[wr_commit_pnt_q].op == VSE &&
      (wr_track_q[wr_commit_pnt_q].in_base + wr_commit_len_q) ==
      (wr_track_q[wr_commit_pnt_q].in_total - 1) &&
      wr_track_q[wr_commit_pnt_q].lastb < AxiDataBytes)
    trim_mask = (AxiDataBytes'(1) << wr_track_q[wr_commit_pnt_q].lastb) - 1;
end

logic [AxiDataWidth-1:0] shifted_data;
logic [AxiDataBytes-1:0] shifted_strb;
logic [AxiDataWidth-1:0] spill_data_q, spill_data_d, spill_data_temp;
logic [AxiDataBytes-1:0] spill_strb_q, spill_strb_d, spill_strb_temp;

shift_w_beat #(
    .AxiDataWidth( AxiDataWidth ), 
    .AxiDataBytes( AxiDataBytes )
  ) i_shift_w_beat (
    .data_i       (axi_req_i.w.data            ),
    .data_o       (shifted_data                ),
    .strb_i       (axi_req_i.w.strb & trim_mask),
    .strb_o       (shifted_strb                ),
    .off_i        (wr_head.off                 ),
    .spill_data_i (spill_data_q                ),
    .spill_data_o (spill_data_temp             ),
    .spill_strb_i (spill_strb_q                ),
    .spill_strb_o (spill_strb_temp             )
  );

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
    off_inst_q     <= '0;
    flush_inst_q   <= '0;
    flush_active_q <= '0;
    spill_data_q   <= '0;
    spill_strb_q   <= '0;
    wr_in_base_q   <= '0;
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
    off_inst_q     <= off_inst_d;
    flush_inst_q   <= flush_inst_d;
    flush_active_q <= flush_active_d;
    spill_data_q   <= spill_data_d;
    spill_strb_q   <= spill_strb_d;
    wr_in_base_q   <= wr_in_base_d;
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

  off_inst_d     = off_inst_q;
  flush_inst_d   = flush_inst_q;
  flush_active_d = flush_active_q;
  spill_data_d   = spill_data_q;
  spill_strb_d   = spill_strb_q;
  wr_in_base_d   = wr_in_base_q;
  is_first_aw    = '0;
  is_last_aw     = '0;
  wr_vl_sum      = '0;

  //////////////////////
  // Request Handling //
  //////////////////////
  
  // Indexed/strided stores issue bus-aligned addresses and the element offset is applied on the W channel byte-enables
  axi_req_o.aw      = axi_req_i.aw;
  axi_req_o.aw.addr = axi_req_i.aw.addr - (axi_req_i.aw.addr % AxiDataBytes); // Align the address
  if (cluster_metadata_i.op inside {VSXE, VSSE})
    axi_req_o.aw.addr = axi_req_i.aw.addr & ~axi_addr_t'(AxiDataBytes-1);

  if (axi_req_i.aw_valid && axi_resp_o.aw_ready) begin
    automatic vew_e     vew          = cluster_metadata_i.vew;
    automatic int       burst        = axi_req_i.aw.len + 1;
    automatic int       axi_bytes    = AxiDataBytes;
    automatic vlen_cluster_t vlen_request = ((burst << $clog2(AxiDataBytes)) - (axi_req_i.aw.addr[$clog2(AxiDataBytes)-1:0])) >> vew;

    wr_vl_sum    = wr_vl_q + vlen_request;
    is_first_aw  = (wr_vl_q == '0);
    is_last_aw   = (wr_vl_sum >= cluster_metadata_i.vl) || (cluster_metadata_i.op inside {VSXE, VSSE});
    wr_vl_d      = wr_vl_sum;
    wr_in_base_d = is_last_aw ? 0 : (is_first_aw ? 0 : wr_in_base_q) + (axi_req_i.aw.len + 1);
    // Only update offset at first aw
    off_inst_d = is_first_aw ? (axi_req_i.aw.addr % AxiDataBytes) : off_inst_q;
    // A flush is injected if the number of valid bytes in the last beat + the offset exceeds the size of a beat
    // Only update at first aw
    flush_inst_d = is_first_aw ? ((cluster_metadata_i.op == VSE) && off_inst_d != 0 && (off_inst_d + lastb > AxiDataBytes)) : flush_inst_q;
    
    wr_track_d[wr_pnt_q].addr       = axi_req_i.aw.addr;
    wr_track_d[wr_pnt_q].len        = axi_req_i.aw.len;
    wr_track_d[wr_pnt_q].op         = cluster_metadata_i.op;
    wr_track_d[wr_pnt_q].off        = cluster_metadata_i.op == VSE ? off_inst_d : '0;
    wr_track_d[wr_pnt_q].flush      = cluster_metadata_i.op == VSE ? flush_inst_d : '0;
    wr_track_d[wr_pnt_q].is_last_aw = is_last_aw;
    wr_track_d[wr_pnt_q].lastb      = lastb;
    wr_track_d[wr_pnt_q].in_base    = is_first_aw ? 0 : wr_in_base_q;
    wr_track_d[wr_pnt_q].in_total   = (nbytes + AxiDataBytes - 1) / AxiDataBytes;
    b_track_d[b_pnt_q].count        += 1;

    wr_cnt_d += 1;
    wr_pnt_d = (wr_pnt_q == NumTrackers-1) ? 0 : wr_pnt_q + 1;
    if (is_last_aw) begin
      wr_vl_d = 0;
      b_pnt_d = (b_pnt_q == NumTrackers-1) ? 0 : b_pnt_q + 1;
    end
  end

  ///////////////////////
  // Response Handling //
  ///////////////////////

  axi_req_o.w = axi_req_i.w;
  axi_req_o.w.last = 1'b0;

  // W is accepted when there are no flush beats being sent
  if (axi_req_i.w_valid && axi_resp_o.w_ready && !flush_now) begin
    // Manage spill registers
    if (wr_track_q[wr_commit_pnt_q].op == VSE && wr_track_q[wr_commit_pnt_q].off != 0) begin
      // It is possible to have last_aw but not wr_commit_len_q == wr_track_q[wr_commit_pnt_q].len (Split requests!)
      // Set spill register data depending on wether flush is needed or not
      // Clearing is needed to avoid leftover spill leaking into the next store that goes through the shifter
      if (wr_track_q[wr_commit_pnt_q].is_last_aw && (wr_commit_len_q == wr_track_q[wr_commit_pnt_q].len) && !wr_track_q[wr_commit_pnt_q].flush) begin
        spill_data_d = '0;
        spill_strb_d = '0;
      end else begin
        spill_data_d = spill_data_temp;
        spill_strb_d = spill_strb_temp;
      end
    end
    wr_commit_len_d += 1;
    // If received all write packets for the request
    if (wr_commit_len_q == wr_track_q[wr_commit_pnt_q].len- ((wr_track_q[wr_commit_pnt_q].flush && wr_track_q[wr_commit_pnt_q].is_last_aw) ? 1 : 0)) begin
      // Update commit pnt & len
      if (wr_track_q[wr_commit_pnt_q].flush && wr_track_q[wr_commit_pnt_q].is_last_aw) begin
        flush_active_d = 1'b1;
      end else begin
        wr_commit_len_d  = '0;
        wr_commit_pnt_d  = (wr_commit_pnt_q == NumTrackers-1) ? 0 : wr_commit_pnt_q + 1;
        wr_cnt_d        -= 1;
        axi_req_o.w.last = 1'b1;
      end
    end
    
    // Set the strobe and data according to the address
    if (wr_track_q[wr_commit_pnt_q].op inside {VSXE, VSSE}) begin
      automatic axi_addr_t addr = wr_track_q[wr_commit_pnt_q].addr;
      automatic logic [$clog2(AxiDataBytes)-1:0] start_byte_pos = addr[$clog2(AxiDataBytes)-1:0];
      axi_req_o.w.strb = '0;
      axi_req_o.w.data = '0;
      // Set the strb at the correct byte position depending on the address and the element width
      unique case (cluster_metadata_i.vew)
        EW8:  begin 
          axi_req_o.w.strb[start_byte_pos +: 1]    = axi_req_i.w.strb[0 +: 1];
          axi_req_o.w.data[start_byte_pos*8 +: 8]  = axi_req_i.w.data[0 +: 8];
        end
        EW16: begin
          axi_req_o.w.strb[start_byte_pos +: 2]    = axi_req_i.w.strb[0 +: 2];
          axi_req_o.w.data[start_byte_pos*8 +: 16] = axi_req_i.w.data[0 +: 16];
        end
        EW32: begin
          axi_req_o.w.strb[start_byte_pos +: 4]    = axi_req_i.w.strb[0 +: 4];
          axi_req_o.w.data[start_byte_pos*8 +: 32] = axi_req_i.w.data[0 +: 32];
        end
        EW64: begin
          axi_req_o.w.strb[start_byte_pos +: 8]    = axi_req_i.w.strb[0 +: 8];
          axi_req_o.w.data[start_byte_pos*8 +: 64] = axi_req_i.w.data[0 +: 64];
        end
        default: axi_req_o.w.strb = '0;
      endcase
    end
  end

  if (wr_track_q[wr_commit_pnt_q].op == VSE && wr_head.off != 0) begin
    axi_req_o.w.data = shifted_data;
    axi_req_o.w.strb = shifted_strb;
  end else if (wr_track_q[wr_commit_pnt_q].op == VSE) begin
    axi_req_o.w.strb = axi_req_i.w.strb & trim_mask;
  end

  // Inject flush
  if (flush_now) begin
      // Instead of forwarding data from clusters, forward data from spill regs
      axi_req_o.w.data = spill_data_q;
      axi_req_o.w.strb = spill_strb_q;
      axi_req_o.w.last = 1'b1;
      // Update tracker
      if (axi_resp_i.w_ready) begin
        flush_active_d  = 1'b0;
        wr_commit_len_d = '0;
        wr_commit_pnt_d = (wr_commit_pnt_q == NumTrackers-1) ? 0 : wr_commit_pnt_q + 1;
        wr_cnt_d        = wr_cnt_d - 1;
        // reset spill regs
        spill_data_d = '0;
        spill_strb_d = '0;
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

// If no request present, do not receive write packet yet
// Maybe this is not strictly necessary, but done to avoid counter going to negative values

// Write response not ready if flush is being sent
assign axi_resp_o.w_ready = axi_resp_i.w_ready && !wr_tracker_empty && !flush_now;
// Write valis must also be set for flush beats
assign axi_req_o.w_valid  = flush_now || (axi_req_i.w_valid && !wr_tracker_empty);

// Assertion: Verify AXI response data does not change when there is no valid handshake
`ifndef VERILATOR
`ifndef TARGET_SYNTHESIS
assert property (@(posedge clk_i) disable iff (~rst_ni)
  axi_req_o.aw_valid |-> (axi_req_o.aw.addr[$clog2(AxiDataBytes)-1:0] == '0))
  else $error("AXI write address %0h is not aligned to %0d bytes, Misaligned stores not supported in AraXL now, Ensure alignment of data structure in Software", axi_req_o.aw.addr, AxiDataBytes);

assert property (@(posedge clk_i) disable iff (~rst_ni)
  (axi_resp_i_cut[NumStages].r_valid && axi_req_cut_ready[NumStages]) || 
  (data_q == $past(data_d)))
  else $error("AXI response data changed without valid request-response handshake");
`endif
`endif

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

module shift_w_beat #(
  parameter int unsigned AxiDataWidth = 0,
  parameter int unsigned AxiDataBytes = 0
) (
  input  logic [AxiDataWidth-1:0]         data_i,
  input  logic [AxiDataBytes-1:0]         strb_i,
  input  logic [AxiDataWidth-1:0]         spill_data_i,
  input  logic [AxiDataBytes-1:0]         spill_strb_i,
  input  logic [$clog2(AxiDataBytes)-1:0] off_i,
  output logic [AxiDataWidth-1:0]         data_o,
  output logic [AxiDataBytes-1:0]         strb_o,
  output logic [AxiDataWidth-1:0]         spill_data_o,
  output logic [AxiDataBytes-1:0]         spill_strb_o
);
  logic [2*AxiDataWidth-1:0] wide_d;
  logic [2*AxiDataBytes-1:0] wide_s;

  always_comb begin
    wide_d       = {{AxiDataWidth{1'b0}}, data_i} << (off_i * 8);
    wide_s       = {{AxiDataBytes{1'b0}}, strb_i} << off_i;
    data_o       = wide_d[AxiDataWidth-1:0] | spill_data_i;
    strb_o       = wide_s[AxiDataBytes-1:0] | spill_strb_i;
    spill_data_o = wide_d[2*AxiDataWidth-1:AxiDataWidth];
    spill_strb_o = wide_s[2*AxiDataBytes-1:AxiDataBytes];
  end
endmodule