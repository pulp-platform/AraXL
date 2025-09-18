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

  // Interfaces with Ariane
  input  accelerator_req_t               acc_req_i,
  
  // To ARA
  input  cluster_axi_req_t   [NrClusters-1:0] axi_req_i,
  output cluster_axi_resp_t  [NrClusters-1:0] axi_resp_o,
  
  // To System AXI 
  input  axi_resp_t                     axi_resp_i,
  output axi_req_t                      axi_req_o
);

localparam int unsigned MAXVL_CL = VLEN * NrClusters;
typedef logic [$clog2(MAXVL_CL+1)-1:0] vlen_cl_t;
vlen_cl_t vl; 
vtype_t vtype;

global_dispatcher #(
  .NrClusters   (NrClusters),
  .vlen_cl_t    (vlen_cl_t )
) i_global_dispatcher (
  .clk_i            (clk_i),
  .rst_ni           (rst_ni),
  .acc_req_i        (acc_req_i),
  .vl_o             (vl),
  .vtype_o          (vtype)
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

cluster_axi_resp_t [NrClusters-1:0] cluster_axi_resp_data_d, cluster_axi_resp_data_q;
cluster_axi_resp_t [NrClusters-1:0] cluster_axi_resp_data_shuffle;
cluster_axi_req_t  [NrClusters-1:0] axi_req_data_d, axi_req_data_q; 

// For Shuffling
logic [NrClusters-1:0] data_valid;
logic [NrClusters-1:0] w_cluster_valid;
logic [NrClusters-1:0] w_cluster_ready_d, w_cluster_ready_q;
logic [NrClusters-1:0] w_cluster_last_d, w_cluster_last_q; 

int len_r, len_w;

// Handle unaligned AXI
cluster_axi_req_t req_d, req_q;
axi_req_t req_final;
logic r_req_valid_d, r_req_valid_q, r_req_ready;
vlen_cl_t vl_req_d, vl_req_q, vl_done;

axi_req_t req_wrmem;
logic w_req_valid_d, w_req_valid_q, w_req_ready;
vlen_cl_t vl_w_d, vl_w_q, vl_w_done;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    r_req_valid_q <= 1'b0;
    req_q         <= '0;
    vl_req_q      <= '0;

    w_req_valid_q <= 1'b0;
    vl_w_q        <= '0;
  end else begin
    r_req_valid_q <= r_req_valid_d;
    req_q         <= req_d;
    vl_req_q      <= vl_req_d;

    w_req_valid_q <= w_req_valid_d;
    vl_w_q        <= vl_w_d;
  end
end

always_comb begin : p_global_ldst
  
  // Copy data between ARA<->System
  // Combine Request from Lane Groups
  // Here using Cluster-0 as the request and ignoring the other requests.
  // aw channel
  req_d = req_q;
  w_req_valid_d = w_req_valid_q;
  w_req_ready = ~w_req_valid_q;
  vl_w_d = vl_w_q;

  req_wrmem = '0; 
  req_wrmem.aw_valid = 1'b0; 
  
  if (axi_req_i[0].aw_valid && w_req_ready) begin
    req_d.aw = axi_req_i[0].aw;
    w_req_valid_d = 1'b1;
    vl_w_d = vl;
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
    wr_aligned_next_start_addr_d = aligned_addr(req_wrmem.aw.addr + (vl_w_d << vtype.vsew) -1, size_axi) + AxiDataWidth/8;
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

    vl_w_done = (wr_aligned_next_start_addr_d - req_d.aw.addr) >> int'(vtype.vsew);
    if (vl_w_d > vl_w_done) begin
      vl_w_d -= vl_w_done;
      req_d.aw.addr = wr_aligned_next_start_addr_d;     // Update request state
      w_req_valid_d = 1'b1;
    end else begin
      w_req_valid_d = 1'b0;
    end
  end
  axi_req_o.aw = req_wrmem.aw;
  axi_req_o.aw_valid = req_wrmem.aw_valid;

  // axi_req_o.aw = axi_req_i[0].aw;
  // len_w = (axi_req_i[0].aw.len + 1) << axi_req_i[0].aw.size << $clog2(NrClusters) >> size_axi;
  // axi_req_o.aw.len = len_w ? len_w-1 : 0;
  // axi_req_o.aw.size = size_axi;
  // axi_req_o.aw_valid = axi_req_i[0].aw_valid;
  
  // Alignment is only done for the read request channel AR
  // ar channel
  r_req_valid_d = r_req_valid_q;
  r_req_ready = ~r_req_valid_q;     // As long as a request is valid, not ready to receive another request
  vl_req_d = vl_req_q;

  req_final = '0;                   // Request to be send on AXI
  req_final.ar_valid = 1'b0;

  if (axi_req_i[0].ar_valid && r_req_ready) begin 
    req_d.ar = axi_req_i[0].ar;
    r_req_valid_d = 1'b1;
    vl_req_d = vl;
  end

  if (r_req_valid_d==1'b1 && axi_resp_i.ar_ready) begin
    automatic logic [8:0] burst_length;
    axi_addr_t aligned_start_addr_d, aligned_next_start_addr_d, aligned_end_addr_d;
    automatic logic [($bits(aligned_start_addr_d) - 12)-1:0] next_2page_msb_d;

    req_final.ar        = req_d.ar;             // Copy request state
    req_final.ar.size   = size_axi;
    req_final.ar.cache  = CACHE_MODIFIABLE;
    req_final.ar.burst  = BURST_INCR;
    req_final.ar_valid = 1'b1;

    // Check if the address is unaligned for AxiDataWidth bits
    aligned_start_addr_d = aligned_addr(req_final.ar.addr, size_axi);
    aligned_next_start_addr_d = aligned_addr(req_final.ar.addr + (vl_req_d << vtype.vsew) -1, size_axi) + AxiDataWidth/8;
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
    vl_done = (aligned_next_start_addr_d - req_d.ar.addr) >> int'(vtype.vsew);
    if (vl_req_d > vl_done) begin
      vl_req_d -= vl_done;
      req_d.ar.addr = aligned_next_start_addr_d;     // Update request state
      r_req_valid_d = 1'b1;
    end else begin
      req_d = '0;
      r_req_valid_d = 1'b0;
    end
  end
  axi_req_o.ar = req_final.ar;
  axi_req_o.ar_valid = req_final.ar_valid;
  
  // axi_req_o.ar = axi_req_i[0].ar;
  // len_r = (axi_req_i[0].ar.len + 1) << axi_req_i[0].ar.size << $clog2(NrClusters) >> size_axi;
  // axi_req_o.ar.len = len_r ? len_r-1 : 0;
  // axi_req_o.ar.size = size_axi;
  // axi_req_o.ar_valid = axi_req_i[0].ar_valid;
  
  // b channel
  axi_req_o.b_ready = axi_req_i[0].b_ready;                                            
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
    axi_resp_o[i].aw_ready = w_req_ready; // axi_resp_i.aw_ready && w_req_ready;
    // ar
    axi_resp_o[i].ar_ready = r_req_ready; // axi_resp_i.ar_ready && r_req_ready;
  end
  
  ////////////// Handle BW mismatch between System and ARA for Read Responses
  // Collect AxiDataWidth data and distribute amongst NrClusters*ClusterAxiDataWidth
  // Send data to all Clusters once NrClusters*ClusterAxiDataWidth is filled.
  cluster_axi_resp_data_d = cluster_axi_resp_data_q;
  for (int i=0; i<NrClusters; i++) begin
    cluster_axi_resp_data_d[i].r_valid = 1'b0;
  end
  cluster_start_r_d = cluster_start_r_q;
  if (axi_resp_i.r_valid) begin : p_valid_read_resp
    // Assign the valid data from System to required to AxiDataWidth/ClusterAxiDataWidth clusters.
    for (int i=0; i<(AxiDataWidth/ClusterAxiDataWidth); i++) begin
      cluster_axi_resp_data_d[cluster_start_r_q+i].r.data = axi_resp_i.r.data[i*ClusterAxiDataWidth +: ClusterAxiDataWidth];
      cluster_axi_resp_data_d[cluster_start_r_q+i].r.id   = axi_resp_i.r.id;
      cluster_axi_resp_data_d[cluster_start_r_q+i].r.resp = axi_resp_i.r.resp;
      cluster_axi_resp_data_d[cluster_start_r_q+i].r.last = 1'b0; //axi_resp_i.r.last;
      cluster_axi_resp_data_d[cluster_start_r_q+i].r.user = axi_resp_i.r.user;
    end
    cluster_start_r_d = cluster_start_r_q + (AxiDataWidth/ClusterAxiDataWidth);
    if ((cluster_start_r_q == (NrClusters - (AxiDataWidth/ClusterAxiDataWidth))) || axi_resp_i.r.last) begin
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
    axi_resp_o[i].r_valid = cluster_axi_resp_data_d[i].r_valid;
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

