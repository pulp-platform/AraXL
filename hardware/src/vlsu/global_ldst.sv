// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Global load store unit that receives LD-ST AXI request from Ara instances
// and generates/receives an AXI request/response to/from the System XBAR.

module global_ldst import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int unsigned NrLanes             = 0,
  parameter  int unsigned NrClusters          = 0,
  parameter  int unsigned AxiDataWidth        = 0,
  parameter  int unsigned ClusterAxiDataWidth = 0,
  parameter  int unsigned AxiAddrWidth        = 0, 
  parameter type cluster_axi_req_t            = logic,
  parameter type cluster_axi_resp_t           = logic,
  parameter type axi_req_t                    = logic,
  parameter type axi_resp_t                   = logic,
  parameter type axi_addr_t                   = logic [AxiAddrWidth-1:0],

  localparam int size_axi                 = $clog2(AxiDataWidth/8),
  localparam int numShuffleStages         = $clog2(AxiDataWidth/(8*NrLanes))-1,
  localparam logic is_full_bw             = (NrClusters == (AxiDataWidth/ClusterAxiDataWidth)) ? 1'b1 : 1'b0,
  localparam int MaxAxiBurst              = 256
  ) (
  input  logic                           clk_i,
  input  logic                           rst_ni,

  input  cluster_metadata_t             cluster_metadata_i,
  output cluster_metadata_t             cluster_metadata_o,
  
  // To ARA
  input  cluster_axi_req_t   [NrClusters-1:0] axi_req_i,
  output cluster_axi_resp_t  [NrClusters-1:0] axi_resp_o,
  
  // To System AXI 
  input  axi_resp_t                     axi_resp_i,
  output axi_req_t                      axi_req_o
);

import cf_math_pkg::idx_width;
import axi_pkg::aligned_addr;
import axi_pkg::BURST_INCR;
import axi_pkg::CACHE_MODIFIABLE;

logic w_ready_q, w_ready_d; // If this unit is ready to receive data from ARA
logic w_valid_d, w_valid_q; // If this unit has a walid write data to System 
logic w_last_d, w_last_q;   // If this is a last write packer

// Pointers to clusters to which data has to be written or read from
logic [$clog2(NrClusters)-1:0] cluster_start_r_d, cluster_start_r_q, cluster_start_wr_d, cluster_start_wr_q;

// Tracks which cluster's request should be taken for VLXE
logic [$clog2(NrClusters)-1:0] cluster_ar_d, cluster_ar_q, cluster_aw_d, cluster_aw_q;

// Tracks which cluster should receive read response for VLXE
logic [$clog2(NrClusters)-1:0] cluster_r_d, cluster_r_q;  
logic [$clog2(NrLanes):0] lane_ar_d, lane_ar_q;
logic [$clog2(NrLanes):0] lane_aw_d, lane_aw_q;
logic [$clog2(NrLanes):0] r_resp_lane_d, r_resp_lane_q;

// Tracks remaining responses for current load
vlen_cluster_t r_resp_rem_d, r_resp_rem_q;

cluster_axi_resp_t [NrClusters-1:0] cluster_axi_resp_data_d, cluster_axi_resp_data_q;
cluster_axi_req_t  [NrClusters-1:0] axi_req_data_d, axi_req_data_q; 

// For Shuffling
logic [NrClusters-1:0] w_cluster_valid;
logic [NrClusters-1:0] w_cluster_ready_d, w_cluster_ready_q;
logic [NrClusters-1:0] w_cluster_last_d, w_cluster_last_q; 

int len_r, len_w;

// Handle unaligned AXI
cluster_axi_req_t req_d, req_q;
axi_req_t req_final;
logic r_req_valid_d, r_req_valid_q, r_req_ready;
vlen_cluster_t vl_done;

axi_req_t req_wrmem;
logic w_req_valid_d, w_req_valid_q, w_req_ready;
vlen_cluster_t vl_w_done;

vew_e vew_d, vew_q;
vlen_cluster_t vl_ldst_rd_d, vl_ldst_rd_q, vl_ldst_wr_d, vl_ldst_wr_q;

// These are updated only when a new aw/ar is accepted
assign cluster_metadata_o.vew = vew_d;
assign cluster_metadata_o.vl = cluster_metadata_i.vl; 
assign cluster_metadata_o.use_eew1 = cluster_metadata_i.use_eew1;
assign cluster_metadata_o.op = cluster_metadata_i.op;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    r_req_valid_q <= 1'b0;
    req_q         <= '0;
    w_req_valid_q <= 1'b0;
    vew_q         <= vew_e'(0);
    vl_ldst_rd_q  <= '0;
    vl_ldst_wr_q  <= '0;
    cluster_ar_q  <= '0;
    cluster_aw_q  <= '0;
    cluster_r_q   <= '0;
    lane_ar_q <= '0;
    lane_aw_q <= '0;
    r_resp_lane_q <= '0;
    r_resp_rem_q <= '0;
  end else begin
    r_req_valid_q <= r_req_valid_d;
    req_q         <= req_d;
    w_req_valid_q <= w_req_valid_d;
    vew_q         <= vew_d;
    vl_ldst_rd_q  <= vl_ldst_rd_d;
    vl_ldst_wr_q  <= vl_ldst_wr_d;
    cluster_ar_q  <= cluster_ar_d;
    cluster_aw_q  <= cluster_aw_d;
    cluster_r_q   <= cluster_r_d;
    lane_ar_q <= lane_ar_d;
    lane_aw_q <= lane_aw_d;
    r_resp_lane_q <= r_resp_lane_d;
    r_resp_rem_q <= r_resp_rem_d;
  end
end

typedef struct packed {
  vlen_cluster_t vl;
  ara_op_e op;
  vew_e vew;
} indexed_op_metadata_t;

indexed_op_metadata_t idx_metadata_i, idx_metadata_o;
logic fifo_push_i, fifo_pop_o;
logic fifo_full_o, fifo_empty_o;

// Fifo to track the request metadata when response arrives
fifo_v3 # (
  .DATA_WIDTH ( $bits(idx_metadata_i) ),
  .DEPTH      ( 8 ) // Maximum number of requests in flight for VLXE
) i_vl_fifo (
  .clk_i  (clk_i          ),
  .rst_ni (rst_ni         ),
  .push_i (fifo_push_i    ),
  .data_i (idx_metadata_i ),
  .pop_i  (fifo_pop_o     ),
  .data_o (idx_metadata_o ),
  .full_o (fifo_full_o    ),
  .empty_o(fifo_empty_o   )
);

always_comb begin : p_global_ldst
  
  // Copy data between ARA<->System
  // Combine Request from Lane Groups
  // aw channel
  req_d = req_q;
  w_req_valid_d = w_req_valid_q;
  w_req_ready = ~w_req_valid_q;

  vew_d = vew_q;
  vl_ldst_rd_d = vl_ldst_rd_q;
  vl_ldst_wr_d = vl_ldst_wr_q;

  req_wrmem = '0; 
  req_wrmem.aw_valid = 1'b0;

  fifo_push_i = 1'b0;

  // Initialize cluster pointers and request counters
  cluster_aw_d = cluster_aw_q;
  lane_aw_d = lane_aw_q;

  // If we have an indexed load, choose the axi request from the desired cluster instead of cluster-0.
  // Start with cluster-0 and update the cluster by 1 if we have processed NrLanes requests and we have more requests to process
  if (axi_req_i[cluster_aw_q].aw_valid && w_req_ready) begin
    req_d.aw = axi_req_i[cluster_aw_q].aw;
    w_req_valid_d = 1'b1;
    vew_d = cluster_metadata_i.vew;
    vl_ldst_wr_d = cluster_metadata_i.vl;

    // If not an indexed load, always use cluster 0
    if (cluster_metadata_i.op != VSXE) begin
      cluster_aw_d = '0;
    end else begin
      // Increment request counter for current cluster
      lane_aw_d = lane_aw_q + 1;
      
      // If we have processed NrLanes requests and have more requests to process, move to next cluster
      if (lane_aw_q == NrLanes - 1) begin
        lane_aw_d = '0;
        if (cluster_aw_q == NrClusters - 1) begin
          cluster_aw_d = '0;
        end else begin
          cluster_aw_d = cluster_aw_q + 1;
        end
      end
    end
  end

  if (w_req_valid_d==1'b1 && axi_resp_i.aw_ready) begin
    automatic logic [8:0] w_burst_length;
    automatic axi_addr_t wr_aligned_start_addr_d, wr_aligned_next_start_addr_d, wr_aligned_end_addr_d;
    automatic logic [($bits(wr_aligned_start_addr_d) - 12)-1:0] wr_next_2page_msb_d;

    req_wrmem.aw        = req_d.aw;             // Copy request state
    req_wrmem.aw.size   = size_axi;
    req_wrmem.aw.cache  = CACHE_MODIFIABLE;
    req_wrmem.aw.burst  = BURST_INCR;
    req_wrmem.aw_valid = 1'b1;

    // Check if the address is unaligned for AxiDataWidth bits
    wr_aligned_start_addr_d = aligned_addr(req_wrmem.aw.addr, size_axi);
    wr_aligned_next_start_addr_d = aligned_addr(req_wrmem.aw.addr + (vl_ldst_wr_d << vew_d) -1, size_axi) + AxiDataWidth/8;
    wr_aligned_end_addr_d = wr_aligned_next_start_addr_d - 1;
    wr_next_2page_msb_d = wr_aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
    // 1 - Check for 4KB page boundary
    if (wr_aligned_start_addr_d[AxiAddrWidth-1:12] != wr_aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      wr_aligned_end_addr_d        = {wr_aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      wr_aligned_next_start_addr_d = {                       wr_next_2page_msb_d, 12'h000};
    end
    // 2 - AXI bursts are at most 256 beats long.
    w_burst_length = MaxAxiBurst;
    if (w_burst_length > ((wr_aligned_end_addr_d - wr_aligned_start_addr_d) >> size_axi) + 1) begin
      w_burst_length = ((wr_aligned_end_addr_d - wr_aligned_start_addr_d) >> size_axi) + 1;
    end else begin
      wr_aligned_next_start_addr_d = wr_aligned_start_addr_d + ((w_burst_length) << size_axi);
      wr_aligned_end_addr_d = wr_aligned_next_start_addr_d - 1;
    end

    req_wrmem.aw.len = w_burst_length - 1;

    vl_w_done = (wr_aligned_next_start_addr_d - req_d.aw.addr) >> int'(vew_d);
    if (vl_ldst_wr_d > vl_w_done) begin
      vl_ldst_wr_d -= vl_w_done;
      req_d.aw.addr = wr_aligned_next_start_addr_d;     // Update request state
      w_req_valid_d = 1'b1;
    end else begin
      w_req_valid_d = 1'b0;
    end
  end
  axi_req_o.aw = req_wrmem.aw;
  axi_req_o.aw_valid = req_wrmem.aw_valid;

  // Alignment is only done for the read request channel AR
  // ar channel
  r_req_valid_d = r_req_valid_q;
  r_req_ready = ~r_req_valid_q;     // As long as a request is valid, not ready to receive another request

  req_final = '0;                   // Request to be send on AXI
  req_final.ar_valid = 1'b0;

  // Initialize cluster pointer and request counter for AR channel
  cluster_ar_d = cluster_ar_q;
  cluster_r_d = cluster_r_q;
  lane_ar_d = lane_ar_q;
  r_resp_lane_d = r_resp_lane_q;
  r_resp_rem_d = r_resp_rem_q;

  // For indexed loads, select AR request from appropriate cluster
  // Start with cluster-0 and update the cluster by 1 if we have processed NrLanes requests and we have more requests to process
  if (axi_req_i[cluster_ar_q].ar_valid && r_req_ready) begin
    req_d.ar = axi_req_i[cluster_ar_q].ar;
    r_req_valid_d = 1'b1;
    vew_d = cluster_metadata_i.vew;
    vl_ldst_rd_d = (vl_ldst_rd_q == '0) ? cluster_metadata_i.vl : vl_ldst_rd_q;

    // If not an indexed load, always use cluster 0
    if (cluster_metadata_i.op != VLXE) begin
      cluster_ar_d = '0;
    end else begin
      // Increment request counter for current cluster
      lane_ar_d = lane_ar_q + 1;
      
      // If we have processed NrLanes requests and have more requests to process, move to next cluster
      if (lane_ar_q == NrLanes - 1) begin
        lane_ar_d = '0;
        if (cluster_ar_q == NrClusters - 1) begin
          cluster_ar_d = '0;
        end else begin
          cluster_ar_d = cluster_ar_q + 1;
        end
      end
    end
  end

  if (r_req_valid_d==1'b1 && axi_resp_i.ar_ready) begin
    automatic logic [8:0] burst_length;
    axi_addr_t aligned_start_addr_d, aligned_next_start_addr_d, aligned_end_addr_d;
    automatic logic [($bits(aligned_start_addr_d) - 12)-1:0] next_2page_msb_d;
    automatic vlen_cluster_t vl_req = (cluster_metadata_i.op == VLXE) ? 1 : vl_ldst_rd_d;
    automatic vlen_cluster_t vl_split;

    req_final.ar        = req_d.ar;             // Copy request state
    req_final.ar.size   = size_axi;
    req_final.ar.cache  = CACHE_MODIFIABLE;
    req_final.ar.burst  = BURST_INCR;
    req_final.ar_valid = 1'b1;

    // Check if the address is unaligned for AxiDataWidth bits
    aligned_start_addr_d = aligned_addr(req_final.ar.addr, size_axi);
    aligned_next_start_addr_d = aligned_addr(req_final.ar.addr + (vl_req << vew_d) -1, size_axi) + AxiDataWidth/8;
    aligned_end_addr_d = aligned_next_start_addr_d - 1;
    next_2page_msb_d = aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
    // 1 - Check for 4KB page boundary
    if (aligned_start_addr_d[AxiAddrWidth-1:12] != aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      aligned_end_addr_d        = {aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      aligned_next_start_addr_d = {                       next_2page_msb_d, 12'h000};
    end
    // 2 - AXI bursts are at most 256 beats long.
    burst_length = MaxAxiBurst;
    if (burst_length > ((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1) begin
      burst_length = ((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1;
    end else begin
      aligned_next_start_addr_d = aligned_start_addr_d + ((burst_length) << size_axi);
      aligned_end_addr_d = aligned_next_start_addr_d - 1;
    end

    req_final.ar.len = burst_length - 1;
    vl_done = (aligned_next_start_addr_d - req_d.ar.addr) >> int'(vew_d);

    if (cluster_metadata_i.op != VLXE) begin
      if (vl_ldst_rd_d > vl_done) begin
        vl_split = vl_done;
        vl_ldst_rd_d -= vl_done;
        req_d.ar.addr = aligned_next_start_addr_d;     // Update request state
        r_req_valid_d = 1'b1;
      end else begin
        vl_split = vl_ldst_rd_d;
        vl_ldst_rd_d = '0;
        req_d = '0;
        r_req_valid_d = 1'b0;
      end
    end else begin
      // For VLXE, we decrement the vl by 1 as each request is for 1 element
      vl_ldst_rd_d -= 1;
      req_d = '0;
      r_req_valid_d = 1'b0;
      if (vl_ldst_rd_q == 1) begin
        // If we have sent all the requests for this indexed vector load, we can move the AR pointer back to cluster 0.
        cluster_ar_d = '0;
        lane_ar_d = '0;
      end
    end

    // Push vector length and op info into the fifo to read later when responses come back. This is needed to know how many responses to expect for each vector load and when a vector load is completed.
    idx_metadata_i.vl = (cluster_metadata_i.op == VLXE) ? cluster_metadata_i.vl : vl_split;  // For VLE add the required vector length if the request was split
    idx_metadata_i.op = cluster_metadata_i.op;
    idx_metadata_i.vew = vew_d;
    fifo_push_i = 1'b1;
  end
  axi_req_o.ar = req_final.ar;
  axi_req_o.ar_valid = req_final.ar_valid;
  
  // b channel
  axi_req_o.b_ready = axi_req_i[cluster_aw_q].b_ready;
  // r channel
  axi_req_o.r_ready = 1'b1;
  for (int i=0; i<NrClusters; i++)
    axi_req_o.r_ready &= axi_req_i[i].r_ready;

  // Distribute response to Lane Groups
  for (int i=0; i<NrClusters; i++) begin
    // b
    axi_resp_o[i].b_valid = axi_resp_i.b_valid;
    axi_resp_o[i].b = axi_resp_i.b;
    // aw
    axi_resp_o[i].aw_ready = w_req_ready;
    // ar
    if (cluster_metadata_i.op == VLXE) begin
      // For VLXE, only send ready to the cluster from which we have taken the request
      axi_resp_o[i].ar_ready = (i == cluster_ar_q) ? r_req_ready : 1'b0;
    end else begin
      axi_resp_o[i].ar_ready = r_req_ready;
    end
  end
  
  ////////////// Handle BW mismatch between System and ARA for Read Responses
  // Collect AxiDataWidth data and distribute amongst NrClusters*ClusterAxiDataWidth
  // Send data to all Clusters once NrClusters*ClusterAxiDataWidth is filled.
  cluster_axi_resp_data_d = cluster_axi_resp_data_q;
  for (int i=0; i<NrClusters; i++) begin
    cluster_axi_resp_data_d[i].r_valid = 1'b0;
    axi_resp_o[i].r_valid = 1'b0;
  end
  
  fifo_pop_o = 1'b0;

  // If we are expecting a response
  if (!fifo_empty_o) begin
    r_resp_rem_d = r_resp_rem_q ? r_resp_rem_q : idx_metadata_o.vl;
    
    if (axi_resp_i.r_valid) begin : p_valid_read_resp
      // Assign the valid data from System to required to AxiDataWidth/ClusterAxiDataWidth clusters.
      for (int i=0; i<(AxiDataWidth/ClusterAxiDataWidth); i++) begin
        cluster_axi_resp_data_d[cluster_r_q+i].r.data = axi_resp_i.r.data[i*ClusterAxiDataWidth +: ClusterAxiDataWidth];
        cluster_axi_resp_data_d[cluster_r_q+i].r.id   = axi_resp_i.r.id;
        cluster_axi_resp_data_d[cluster_r_q+i].r.resp = axi_resp_i.r.resp;
        cluster_axi_resp_data_d[cluster_r_q+i].r.last = 1'b0;
        cluster_axi_resp_data_d[cluster_r_q+i].r.user = axi_resp_i.r.user;
      end
      cluster_start_r_d = cluster_r_q + (AxiDataWidth/ClusterAxiDataWidth);
      if ((cluster_r_q == (NrClusters - (AxiDataWidth/ClusterAxiDataWidth))) || axi_resp_i.r.last) begin
        cluster_start_r_d = 0;
        for (int i=0; i<NrClusters; i++) begin
          cluster_axi_resp_data_d[i].r_valid = 1'b1;
          if (axi_resp_i.r.last)
            cluster_axi_resp_data_d[i].r.last = 1'b1;
        end
      end
    end : p_valid_read_resp
    
    for (int i=0; i<NrClusters; i++) begin  
      axi_resp_o[i].r = cluster_axi_resp_data_d[i].r;
      if (idx_metadata_o.op == VLXE) begin
        // If indexed load send reponse only to the desired cluster
        axi_resp_o[i].r_valid = (i == cluster_r_q) ? cluster_axi_resp_data_d[i].r_valid : 1'b0;
        // Update which cluster should receive the read response for VLXE
        if (axi_resp_o[i].r_valid) begin
          // Count responses being sent in this cycle
          // r_responses_to_decrement = r_responses_to_decrement + 1;

          r_resp_rem_d -= 1;

          // pop from fifo for every idx response 
          fifo_pop_o = 1'b1;
          
          // Increment response counter for current cluster
          r_resp_lane_d = r_resp_lane_q + 1;
          
          // Move to next cluster after processing NrLanes responses
          if (r_resp_lane_q == NrLanes - 1) begin
            r_resp_lane_d = '0;
            cluster_r_d = cluster_r_q + 1;
            if (cluster_r_q == NrClusters - 1) begin
              cluster_r_d = '0;
            end
          end
        end
      end else begin
        axi_resp_o[i].r_valid = cluster_axi_resp_data_d[0].r_valid;
      end
    end

    // Since all clusters receive data synchronously for VLE, just check for valid from cluster 0
    if (idx_metadata_o.op != VLXE && axi_resp_o[0].r_valid && axi_req_i[0].r_ready) begin
      automatic logic [$clog2(AxiDataWidth/8):0] nelem = ((AxiDataWidth/8) >> idx_metadata_o.vew);
      if (r_resp_rem_d > nelem) begin
        r_resp_rem_d -= nelem;
      end else begin
        r_resp_rem_d = '0;
        fifo_pop_o = 1'b1;
      end
    end

    if (r_resp_rem_d ==0) begin
      cluster_r_d = '0;
      r_resp_lane_d = '0;
    end
  end
  
  /////////// Handle BW mismatch between System and ARA for Write Request

  for (int i=0; i<NrClusters; i++) begin
    w_cluster_valid[i] = axi_req_i[i].w_valid;
    w_cluster_ready_d[i] = w_cluster_ready_q[i] ? ~axi_req_i[i].w_valid : w_cluster_ready_q[i];
  end

  cluster_start_wr_d = cluster_start_wr_q;
  axi_req_data_d = axi_req_data_q;
  
  w_valid_d = w_valid_q;
  w_ready_d = w_ready_q;
  // If we are ready to receive data, receive new request
  // otherwise assign previous request state.
  for (int i=0; i<NrClusters; i++) begin
    axi_req_data_d[i] = w_cluster_ready_q[i] ? axi_req_i[i] : axi_req_data_q[i];
    w_cluster_last_d[i] = w_cluster_ready_q[i] ? axi_req_i[0].w.last : w_cluster_last_q[i];  // Only expecting last signal from cluster-0
  end
  w_last_d = w_ready_q ? axi_req_i[0].w.last : w_last_q;

  //if (axi_req_i[0].w_valid) begin : p_valid_write_data
  if (&w_cluster_valid) begin : p_valid_write_data
    // If Total BW of all clusters == System AXI BW, we can support full write
    // BW, otherwise set not ready to receive data.
    w_ready_d = is_full_bw ? axi_resp_i.w_ready : 1'b0;
    w_valid_d = 1'b1;
  end : p_valid_write_data

  axi_req_o.w_valid = 1'b0;
  axi_req_o.w = '0;
  
  if (w_valid_d) begin
    // Have a valid write data to send to System AXI
    for (int i=0; i<(AxiDataWidth/ClusterAxiDataWidth); i++) begin
      axi_req_o.w.data[i*ClusterAxiDataWidth +: ClusterAxiDataWidth] = axi_req_data_d[cluster_start_wr_q + i].w.data;
      axi_req_o.w.strb[i*ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8] = axi_req_data_d[cluster_start_wr_q + i].w.strb;
      axi_req_o.w.user = axi_req_data_d[cluster_start_wr_q + i].w.user;
    end
    axi_req_o.w_valid = 1'b1;
    axi_req_o.w.last = 1'b0;
    
    if (axi_resp_i.w_ready) begin
      // If downstream AXI is ready to receive request only then update the
      // pointer to cluster data.
      cluster_start_wr_d = cluster_start_wr_q + (AxiDataWidth/ClusterAxiDataWidth);
      if (cluster_start_wr_q == (NrClusters - (AxiDataWidth/ClusterAxiDataWidth))) begin
        cluster_start_wr_d = 0;
        w_ready_d = 1'b1; // Once all write data from all clusters sent, then we are ready to receive data from clusters.
        w_cluster_ready_d = '1; 
        w_valid_d = 1'b0; // We don't have valid data anymore
        if (&w_cluster_last_d) begin
          axi_req_o.w.last = 1'b1;
          w_last_d = 1'b0;
          w_cluster_last_d = '0; 
        end
      end
    end
  end

  for (int i=0; i<NrClusters; i++) begin
    axi_resp_o[i].w_ready = w_cluster_ready_q[i];
  end

end : p_global_ldst

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    cluster_axi_resp_data_q <= '0;
    cluster_start_r_q <= 0;

    w_valid_q <= 1'b0;
    w_last_q <= 1'b0;
    cluster_start_wr_q <= 0;
    axi_req_data_q <= '0;
    w_ready_q <= 1'b1;

    w_cluster_ready_q <= '1;
    w_cluster_last_q  <= '0;
  end else begin
    cluster_axi_resp_data_q <= cluster_axi_resp_data_d;
    cluster_start_r_q <= cluster_start_r_d;

    w_valid_q <= w_valid_d;
    w_last_q <= w_last_d;
    cluster_start_wr_q <= cluster_start_wr_d;
    axi_req_data_q <= axi_req_data_d;
    w_ready_q <= w_ready_d;

    w_cluster_ready_q <= w_cluster_ready_d;
    w_cluster_last_q  <= w_cluster_last_d; 
  end
end

if (AxiDataWidth/ClusterAxiDataWidth > NrClusters)
  $error("AxiDataWidth > (NrClusters * ClusterAxiDataWidth) is not supported!! ");

endmodule : global_ldst

