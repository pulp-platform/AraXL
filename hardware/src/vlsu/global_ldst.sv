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

  input vew_e                           vew_ar_i,
  input vew_e                           vew_aw_i,
  input vlen_t                          vl_ldst_i,

  // vew information to align stage
  output vew_e                           vew_ar_o,
  output vew_e                           vew_aw_o,
  output vlen_t                          vl_ldst_rd_o,
  output vlen_t                          vl_ldst_wr_o,
  
  // To System AXI 
  input  axi_resp_t                     axi_resp_i,
  output axi_req_t                      axi_req_o
);

import cf_math_pkg::idx_width;
import axi_pkg::aligned_addr;
import axi_pkg::beat_lower_byte;
import axi_pkg::beat_upper_byte;
import axi_pkg::BURST_INCR;
import axi_pkg::CACHE_MODIFIABLE;

localparam clog2_AxiStrobeWidth = $clog2(AxiDataWidth/8);
localparam int unsigned MAXVL_CL = VLEN * NrClusters;

typedef logic [$clog2(MAXVL_CL+1)-1:0] vlen_cl_t;

typedef struct packed {
  axi_pkg::largest_addr_t addr;
  axi_pkg::size_t size;
  axi_pkg::len_t len;
  logic is_load;
  vlen_cl_t vl_ldst;
  vew_e vsew;
} addrgen_cluster_axi_req_t;

logic ar_addrgen_ack, aw_addrgen_ack;

// Pointers to clusters to which data has to be written or read from
logic [$clog2(NrClusters)-1:0] cluster_start_r_d, cluster_start_r_q, cluster_start_wr_d, cluster_start_wr_q;
vew_e vew_ar_d, vew_ar_q, vew_aw_d, vew_aw_q;
vlen_t vl_ldst_rd_d, vl_ldst_rd_q, vl_ldst_wr_d, vl_ldst_wr_q;

cluster_axi_resp_t [NrClusters-1:0] cluster_axi_resp_data_d, cluster_axi_resp_data_q;
cluster_axi_req_t  [NrClusters-1:0] axi_req_data_d, axi_req_data_q; 

// For Shuffling
logic [NrClusters-1:0] w_cluster_valid;
logic [NrClusters-1:0] w_cluster_ready_d, w_cluster_ready_q;
logic [NrClusters-1:0] w_cluster_last_d, w_cluster_last_q; 

// Handle unaligned AXI
cluster_axi_req_t req_d, req_q;
axi_req_t req_final;
logic r_req_valid_d, r_req_valid_q, r_req_ready;
vlen_cl_t vl_req_d, vl_req_q, vl_done;

axi_req_t req_wrmem;
logic w_req_valid_d, w_req_valid_q, w_req_ready;
vlen_cl_t vl_w_d, vl_w_q, vl_w_done;

// AXI burst length
logic [8:0] w_burst_length;
// NOTE: all these variables could be narrowed to the minimum number of bits
logic [12:0] num_beats;

/////////////////////////////////////
//  Support for misaligned stores  //
/////////////////////////////////////

// Narrower AXI Data Byte-Width used for misaligned stores
logic [clog2_AxiStrobeWidth-1:0]            narrow_axi_data_bwidth;
// Helper signal to calculate the narrow_axi_data_bwidth
// It carries information about the misalignment of the start address w.r.t. the AxiDataWidth
logic [clog2_AxiStrobeWidth-1:0]            axi_addr_misalignment;
// Number of trailing 0s of axi_addr_misalignment
logic [idx_width(clog2_AxiStrobeWidth)-1:0] zeroes_cnt;

// If this is the first beat of the write signal
logic first_beat_d, first_beat_q;
// A counter of how many beats are left in the current AXI burst
axi_pkg::len_t wr_axi_len_d, wr_axi_len_q;
// A pointer to which byte in the full VRF word we are reading data from.
logic [idx_width(ClusterAxiDataWidth*NrClusters/8):0] wr_data_pnt_d, wr_data_pnt_q;

// Get the misalignment information for this vector memory instruction
assign axi_addr_misalignment = req_d.aw.addr[clog2_AxiStrobeWidth-1:0];

int unsigned wr_data_seq_byte;

vlen_cl_t wr_issue_cnt_bytes_d, wr_issue_cnt_bytes_q;

vlen_cl_t wr_vinsn_valid_bytes;
logic [idx_width(ClusterAxiDataWidth*NrClusters/8):0] wr_data_valid_bytes;
logic [idx_width(AxiDataWidth/8):0] wr_axi_valid_bytes;
logic [idx_width(AxiDataWidth/8):0] wr_valid_bytes;

logic [ClusterAxiDataWidth*NrClusters-1:0] wr_data_coalesce;
logic [ClusterAxiDataWidth*NrClusters/8-1:0] wr_strb_coalesce;

logic [idx_width(ClusterAxiDataWidth*NrClusters/8):0] data_eff_write_bytes;

// Have a valid write data to send to System AXI
generate
  for (genvar i=0; i<NrClusters; i++) begin
    assign wr_data_coalesce[i*ClusterAxiDataWidth +: ClusterAxiDataWidth] = axi_req_data_d[i].w.data;
    assign wr_strb_coalesce[i*ClusterAxiDataWidth/8 +: ClusterAxiDataWidth/8] = axi_req_data_d[i].w.strb;
  end
endgenerate

// Calculate the maximum number of Bytes we can send in a store-misaligned beat.
// This number must be a power of 2 not to get misaligned wrt the pack of data that the
// store unit receives from the lanes
lzc #(
  .WIDTH(clog2_AxiStrobeWidth),
  .MODE (1'b0                  )
) i_lzc (
  .in_i   (axi_addr_misalignment),
  .cnt_o  (zeroes_cnt           ),
  .empty_o(/* Unconnected */    )
);

// Effective AXI data width for misaligned stores
assign narrow_axi_data_bwidth = (AxiDataWidth/8) >> (clog2_AxiStrobeWidth - zeroes_cnt);

//////////////////////////////
//  AXI Request Generation  //
//////////////////////////////

enum logic [2:0] {
  AXI_ADDRGEN_IDLE,
  AXI_ADDRGEN_WAITING_CORE_STORE_PENDING,
  AXI_ADDRGEN_AXI_DW_STORE_MISALIGNED,    // Misaligned vector store to AxiDataWidth/8, needs special treatement
  AXI_ADDRGEN_REQUESTING                  // Perform AW/AR transactions and push addrgen_req to VSTU/VLDU
} axi_addrgen_state_d, axi_addrgen_state_q;

axi_addr_t wr_aligned_start_addr_d, wr_aligned_start_addr_q;
axi_addr_t wr_aligned_next_start_addr_d, wr_aligned_next_start_addr_q, wr_aligned_next_start_addr_temp;
axi_addr_t wr_aligned_end_addr_d, wr_aligned_end_addr_q, wr_aligned_end_addr_temp;

// MSb of the next-next page (page selector for page 2 positions after the current one)
logic [($bits(wr_aligned_start_addr_d)-12)-1:0] wr_next_2page_msb_d, wr_next_2page_msb_q;

logic [clog2_AxiStrobeWidth:0]            eff_axi_dw_d, eff_axi_dw_q;
logic [idx_width(clog2_AxiStrobeWidth):0] eff_axi_dw_log_d, eff_axi_dw_log_q;

function automatic void set_end_addr (
    input  logic [($bits(axi_addr_t)-12)-1:0]       next_2page_msb,
    input  vlen_cl_t                                  num_bytes,
    input  axi_addr_t                                 addr,
    input  logic [clog2_AxiStrobeWidth:0]             eff_axi_dw,
    input  logic [idx_width(clog2_AxiStrobeWidth):0]  eff_axi_dw_log,
    input  axi_addr_t                                 aligned_start_addr,
    output axi_addr_t                                 aligned_end_addr,
    output axi_addr_t                                 aligned_next_start_addr
);

  automatic int unsigned max_burst_bytes = 256 << eff_axi_dw_log;

  // The final address can be found similarly...
  if (num_bytes >= max_burst_bytes) begin
      aligned_next_start_addr = aligned_addr(addr + max_burst_bytes, eff_axi_dw_log);
  end else begin
      aligned_next_start_addr = aligned_addr(addr + num_bytes - 1, eff_axi_dw_log) + eff_axi_dw;
  end
  aligned_end_addr = aligned_next_start_addr - 1;

  // But since AXI requests are aligned in 4 KiB pages, aligned_end_addr must be in the
  // same page as aligned_start_addr
  if (aligned_start_addr[AxiAddrWidth-1:12] != aligned_end_addr[AxiAddrWidth-1:12]) begin
      aligned_end_addr        = {aligned_start_addr[AxiAddrWidth-1:12], 12'hFFF};
      aligned_next_start_addr = {                     next_2page_msb  , 12'h000};
  end
endfunction

//////////////////////////////
//  Address Queue for Store //
//////////////////////////////

// Address queue for the vector load/store units
addrgen_cluster_axi_req_t axi_addrgen_queue;
addrgen_cluster_axi_req_t axi_addrgen_req;
logic             axi_addrgen_queue_push;
logic             axi_addrgen_queue_full;
logic             axi_addrgen_queue_empty;
logic             axi_addrgen_queue_pop;
logic             axi_addrgen_req_ready;
logic             axi_addrgen_req_valid;

assign axi_addrgen_queue_pop = axi_addrgen_req_ready;

fifo_v3 #(
  .DEPTH(VaddrgenInsnQueueDepth     ),
  .dtype(addrgen_cluster_axi_req_t  )
) i_addrgen_req_queue (
  .clk_i     (clk_i                                                    ),
  .rst_ni    (rst_ni                                                   ),
  .flush_i   (1'b0                                                     ),
  .testmode_i(1'b0                                                     ),
  .data_i    (axi_addrgen_queue                                        ),
  .push_i    (axi_addrgen_queue_push                                   ),
  .full_o    (axi_addrgen_queue_full                                   ),
  .data_o    (axi_addrgen_req                                          ),
  .pop_i     (axi_addrgen_queue_pop                                    ),
  .empty_o   (axi_addrgen_queue_empty                                  ),
  .usage_o   (/* Unused */                                             )
);

assign axi_addrgen_req_valid = !axi_addrgen_queue_empty;

// These are updated only when a new aw/ar is accepted
assign vew_ar_o = vew_ar_d;
assign vew_aw_o = vew_aw_d;
assign vl_ldst_rd_o = vl_ldst_rd_d;
assign vl_ldst_wr_o = vl_ldst_wr_d;



always_comb begin : p_global_ldst
  // Maintain state
  axi_addrgen_state_d = axi_addrgen_state_q;
  // Copy data between ARA<->System
  // Combine Request from Lane Groups
  // Here using Cluster-0 as the request and ignoring the other requests.
  // aw channel
  req_d = req_q;

  wr_aligned_start_addr_d = wr_aligned_start_addr_q;
  wr_aligned_next_start_addr_d = wr_aligned_next_start_addr_q;
  wr_aligned_end_addr_d = wr_aligned_end_addr_q;

  wr_aligned_next_start_addr_temp = wr_aligned_next_start_addr_q;
  wr_aligned_end_addr_temp = wr_aligned_end_addr_q;

  wr_next_2page_msb_d = wr_next_2page_msb_q;

  eff_axi_dw_d = eff_axi_dw_q;
  eff_axi_dw_log_d = eff_axi_dw_log_q;

  w_req_valid_d = w_req_valid_q;
  w_req_ready = ~w_req_valid_q;
  vl_w_d = vl_w_q;
  vew_aw_d = vew_aw_q;

  wr_axi_len_d = wr_axi_len_q;
  wr_issue_cnt_bytes_d = wr_issue_cnt_bytes_q;
  wr_data_pnt_d = wr_data_pnt_q;

  vl_ldst_wr_d = vl_ldst_wr_q;
  vl_ldst_rd_d = vl_ldst_rd_q;

  ar_addrgen_ack = 1'b0;
  aw_addrgen_ack = 1'b0;

  req_wrmem = '0; 
  req_wrmem.aw_valid = 1'b0; 

  axi_addrgen_queue = '0;
  axi_addrgen_queue_push = 1'b0;


  wr_data_seq_byte = 0;

  if (axi_req_i[0].aw_valid && w_req_ready) begin
    req_d.aw = axi_req_i[0].aw;
    w_req_valid_d = 1'b1;
    vew_aw_d = vew_aw_i;
    vl_ldst_wr_d = vl_ldst_i;
    // vl in terms of byte
    vl_w_d = (vl_ldst_i << $clog2(NrClusters)) << vew_aw_i;
  end

  case (axi_addrgen_state_q)
      AXI_ADDRGEN_IDLE: begin
        // The start address is found by aligning the original request address by the width of
        // the memory interface.
        wr_aligned_start_addr_d = aligned_addr(req_d.aw.addr, clog2_AxiStrobeWidth);
        // Pre-calculate the next_2page_msb. This should not require much energy if the addr
        // has zeroes in the upper positions.
        // We can use this also for the misaligned address calculation, as the next 2 page msb
        // will be the same either way.
        wr_next_2page_msb_d = wr_aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
        // The final address can be found similarly...
        set_end_addr (
          wr_next_2page_msb_d,
          vl_w_d,
          req_d.aw.addr,
          AxiDataWidth/8,
          clog2_AxiStrobeWidth,
          wr_aligned_start_addr_d,
          wr_aligned_end_addr_d,
          wr_aligned_next_start_addr_d
        );

        if (w_req_valid_d==1'b1) begin
          axi_addrgen_state_d = acc_req_i.store_pending ? AXI_ADDRGEN_WAITING_CORE_STORE_PENDING : AXI_ADDRGEN_REQUESTING;

          eff_axi_dw_log_d = clog2_AxiStrobeWidth;

          if (req_d.aw.addr[clog2_AxiStrobeWidth-1:0] != '0) begin
            // Calculate the start and the end addresses in the AXI_ADDRGEN_AXI_DW_STORE_MISALIGNED state
            axi_addrgen_state_d = AXI_ADDRGEN_AXI_DW_STORE_MISALIGNED;

            eff_axi_dw_d     = {1'b0, narrow_axi_data_bwidth};
            eff_axi_dw_log_d = zeroes_cnt;
          end else begin
            eff_axi_dw_d     = AxiDataWidth/8;
            eff_axi_dw_log_d = clog2_AxiStrobeWidth;
          end
        end
      end

      AXI_ADDRGEN_AXI_DW_STORE_MISALIGNED: begin
        axi_addrgen_state_d = acc_req_i.store_pending ? AXI_ADDRGEN_WAITING_CORE_STORE_PENDING : AXI_ADDRGEN_REQUESTING;

        // The start address is found by aligning the original request address by the width of
        // the memory interface.
        wr_aligned_start_addr_d = aligned_addr(req_q.aw.addr, eff_axi_dw_log_q);

        set_end_addr (
          wr_next_2page_msb_q,
          vl_w_d,
          req_q.aw.addr,
          eff_axi_dw_q,
          eff_axi_dw_log_q,
          wr_aligned_start_addr_d,
          wr_aligned_end_addr_d,
          wr_aligned_next_start_addr_d
        );
      end

      AXI_ADDRGEN_WAITING_CORE_STORE_PENDING: begin
        if (!acc_req_i.store_pending) begin
          axi_addrgen_state_d = AXI_ADDRGEN_REQUESTING;
        end
      end

      AXI_ADDRGEN_REQUESTING : begin
        automatic logic axi_aw_ready = axi_resp_i.aw_ready;
        automatic logic [12:0] num_bytes; // Cannot consume more than 4 KiB
        automatic vlen_cl_t remaining_bytes;

        // Pre-calculate the next_2page_msb. This should not require much energy if the addr
        // has zeroes in the upper positions.
        wr_next_2page_msb_d = wr_aligned_next_start_addr_q[AxiAddrWidth-1:12] + 1;

        // Pre-calculate the bytes used in a unit-strided access and the remaining bytes to ask.
        // The proper way to do so would be aligned_end_addr_q[11:0] + 1 - req_q.aw.addr[11:0].
        // Avoid explicit computation of aligned_next_start_addr + 1 with a trick that works if
        // aligned_next_start_addr <= 12'hFFF.
        if (wr_aligned_end_addr_q[11:0] != 12'hFFF) begin
          num_bytes = wr_aligned_next_start_addr_q[11:0] - req_q.aw.addr[11:0];
        end else begin
        // Special case: aligned_next_start_addr > 12'hFFF.
          num_bytes = 13'h1000 - req_q.aw.addr[11:0];
        end
        remaining_bytes = vl_w_q - num_bytes;
        if (vl_w_q < num_bytes) begin
          remaining_bytes = 0;
        end

        if (!axi_addrgen_queue_full && axi_aw_ready) begin
          automatic axi_addr_t paddr;

          // Mux target address
          paddr = req_q.aw.addr;

          // 1 - AXI bursts are at most 256 beats long.
          w_burst_length = 256;
          // 2 - The AXI burst length cannot be longer than the number of beats required
          //     to access the memory regions between aligned_start_addr and
          //     aligned_end_addr
          num_beats = ((wr_aligned_end_addr_q[11:0] - wr_aligned_start_addr_q[11:0]) >> eff_axi_dw_log_q) + 1;
          if (w_burst_length > num_beats) begin
            w_burst_length = num_beats;
          end

          // AW Channel
          req_wrmem.aw = '{
            addr   : paddr,
            len    : w_burst_length - 1,
            // If misaligned store access, reduce the effective AXI width
            // This hurts performance
            size   : eff_axi_dw_log_q,
            cache  : CACHE_MODIFIABLE,
            burst  : BURST_INCR,
            default: '0
          };

          // Send this request to the load/store units
          axi_addrgen_queue = '{
            addr         : paddr,
            len          : w_burst_length - 1,
            size         : eff_axi_dw_log_q,
            vl_ldst      : w_burst_length << eff_axi_dw_log_q,
            vsew         : vew_aw_q,
            default: '0
          };

          // Calculate the addresses for the next iteration
          // TODO: test this for SEW!=64, otherwise this computation is never used
          // The start address is found by aligning the original request address by the width of
          // the memory interface. In our case, we have it already.
          set_end_addr (
            wr_next_2page_msb_d,
            vl_w_q - num_bytes,
            wr_aligned_next_start_addr_q,
            eff_axi_dw_q,
            eff_axi_dw_log_q,
            wr_aligned_next_start_addr_q,
            wr_aligned_end_addr_temp,
            wr_aligned_next_start_addr_temp
          );

          // AW Channel
          req_wrmem.aw_valid  = 1'b1;
          // Send this request to the load/store units
          axi_addrgen_queue_push = 1'b1;
          // We pre-calculated the values already
          vl_w_d = remaining_bytes;
          req_d.aw.addr = wr_aligned_next_start_addr_q;

          wr_aligned_start_addr_d      = req_d.aw.addr;
          wr_aligned_end_addr_d        = wr_aligned_end_addr_temp;
          wr_aligned_next_start_addr_d = wr_aligned_next_start_addr_temp;

          // Finished generating AXI requests
          if (vl_w_d == 0) begin
            w_req_valid_d = 1'b0;
            aw_addrgen_ack = 1'b1;
            axi_addrgen_state_d = AXI_ADDRGEN_IDLE;
          end
        end
      end
    endcase

    axi_req_o.aw = req_wrmem.aw;
    axi_req_o.aw_valid = req_wrmem.aw_valid;


  // Alignment is only done for the read request channel AR
  // ar channel
  r_req_valid_d = r_req_valid_q;
  vew_ar_d = vew_ar_q;
  r_req_ready = ~r_req_valid_q;     // As long as a request is valid, not ready to receive another request
  vl_req_d = vl_req_q;

  req_final = '0;                   // Request to be send on AXI
  req_final.ar_valid = 1'b0;

  if (axi_req_i[0].ar_valid && r_req_ready && axi_resp_i.ar_ready) begin 
    req_d.ar = axi_req_i[0].ar;
    r_req_valid_d = 1'b1;
    // vl In terms of element number 
    vl_req_d = vl_ldst_i << $clog2(NrClusters);
    vew_ar_d = vew_ar_i;
    vl_ldst_rd_d = vl_ldst_i;
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
    aligned_next_start_addr_d = aligned_addr(req_final.ar.addr + (vl_req_d << vew_ar_d) -1, size_axi) + AxiDataWidth/8;
    aligned_end_addr_d = aligned_next_start_addr_d - 1;
    next_2page_msb_d = aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
    // 1 - Check for 4KB page boundary
    if (aligned_start_addr_d[AxiAddrWidth-1:12] != aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      aligned_end_addr_d        = {aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      aligned_next_start_addr_d = {                       next_2page_msb_d, 12'h000};
    end
    // 2 - AXI bursts are at most 256 beats long.
    burst_length = MaxAxiBurst;
    // if (burst_length > ((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1) begin
    //   burst_length = ((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1;
    if (burst_length > (((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1)) begin
      burst_length = ((aligned_end_addr_d - aligned_start_addr_d) >> size_axi) + 1;
    end else begin
      aligned_next_start_addr_d = aligned_start_addr_d + ((burst_length) << size_axi);
      aligned_end_addr_d = aligned_next_start_addr_d - 1;
    end

    req_final.ar.len = burst_length - 1;
    vl_done = (aligned_next_start_addr_d - req_d.ar.addr) >> int'(vew_ar_d);
    if (vl_req_d > vl_done) begin
      vl_req_d -= vl_done;
      req_d.ar.addr = aligned_next_start_addr_d;     // Update request state
      r_req_valid_d = 1'b1;
    end else begin
      req_d = '0;
      r_req_valid_d = 1'b0;
      ar_addrgen_ack = 1'b1;
    end
  end
  axi_req_o.ar = req_final.ar;
  axi_req_o.ar_valid = req_final.ar_valid;
  
  // b channel
  axi_req_o.b_ready = 1'b1;
  for (int i=0; i<NrClusters; i++)
    axi_req_o.b_ready &= axi_req_i[i].b_ready;                                          
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
  axi_req_data_d = axi_req_data_q;
  first_beat_d   = first_beat_q;

  axi_req_o.w_valid = 1'b0;
  axi_req_o.w       = '0;

  axi_addrgen_req_ready = 1'b0;

  for (int i=0; i<NrClusters; i++) begin
    w_cluster_valid[i] = axi_req_i[i].w_valid;
    w_cluster_ready_d[i] = w_cluster_ready_q[i] ? ~axi_req_i[i].w_valid : w_cluster_ready_q[i];
  end

  for (int i=0; i<NrClusters; i++) begin
    axi_req_data_d[i] = w_cluster_ready_q[i] ? axi_req_i[i] : axi_req_data_q[i];
    w_cluster_last_d[i] = w_cluster_ready_q[i] ? axi_req_i[0].w.last : w_cluster_last_q[i];  // Only expecting last signal from cluster-0
  end

  if (axi_addrgen_req_valid && axi_resp_i.w_ready) begin
    // Bytes valid in the current W beat
    automatic shortint unsigned lower_byte = beat_lower_byte(axi_addrgen_req.addr,
      axi_addrgen_req.size, axi_addrgen_req.len, BURST_INCR, AxiDataWidth/8, wr_axi_len_q);
    automatic shortint unsigned upper_byte = beat_upper_byte(axi_addrgen_req.addr,
      axi_addrgen_req.size, axi_addrgen_req.len, BURST_INCR, AxiDataWidth/8, wr_axi_len_q);

    if (first_beat_q) begin
      wr_issue_cnt_bytes_d = axi_addrgen_req.vl_ldst;
    end

    // Account for the issued bytes
    // How many bytes are valid in this VRF word
    wr_data_valid_bytes  = AxiDataWidth/8 - wr_data_pnt_q;
    // How many bytes are valid in this AXI word
    wr_axi_valid_bytes   = upper_byte - lower_byte + 1;

    wr_valid_bytes = (wr_data_valid_bytes  < wr_axi_valid_bytes) ? wr_data_valid_bytes    : wr_axi_valid_bytes;

    if (&w_cluster_valid) begin
      first_beat_d  = 1'b0;
      wr_data_pnt_d = wr_data_pnt_q + wr_valid_bytes;

      // Copy data from the operands into the W channel
      for (int unsigned axi_byte = 0; axi_byte < AxiDataWidth/8; axi_byte++) begin
        // Is this byte a valid byte in the W beat?
        if (axi_byte >= lower_byte && axi_byte <= upper_byte) begin
          // Map axy_byte to the corresponding byte in the VRF word (sequential)
          wr_data_seq_byte = axi_byte - lower_byte + wr_data_pnt_q;
          // Copy data
          axi_req_o.w.data[8*axi_byte +: 8] = wr_data_coalesce[8*wr_data_seq_byte +: 8];
          axi_req_o.w.strb[axi_byte]        = wr_strb_coalesce[wr_data_seq_byte];
          // To check if we write any x data into the memory
          if ($isunknown(axi_req_o.w.data[8*axi_byte +: 8]) && axi_req_o.w.strb[axi_byte]) begin
            $error("ASSERTION FAILED: data is X when valid=1");
          end
          // Currently directly wired to cluster 0 user signal, can be changed according to needs
          axi_req_o.w.user                  = axi_req_data_d[0].w.user;
        end
      end

      // Send the W beat
      axi_req_o.w_valid = 1'b1;
      // Account for the beat we sent
      wr_axi_len_d  = wr_axi_len_q + 1;
      // We wrote all the beats for this AW burst
      if ($unsigned(wr_axi_len_d) == axi_pkg::len_t'($unsigned(axi_addrgen_req.len) + 1)) begin : beats_complete
        axi_req_o.w.last        = 1'b1;
        // Ask for another burst by the address generator
        axi_addrgen_req_ready   = 1'b1;
        // Prepare for the first beat for next aw req
        first_beat_d         = 1'b1;
        // Reset AXI pointers
        wr_axi_len_d            = '0;
      end : beats_complete

      // We consumed a whole word from the lanes
      if (wr_data_pnt_d == (ClusterAxiDataWidth*NrClusters/8)) begin
        // Reset the pointer in the VRF word
        wr_data_pnt_d         = '0;

        // Acknowledge the operands from the clusters
        w_cluster_ready_d = '1; 
        // Account for the results that were issued
        data_eff_write_bytes = ClusterAxiDataWidth*NrClusters/8;

        wr_issue_cnt_bytes_d = wr_issue_cnt_bytes_q - data_eff_write_bytes;
        if (wr_issue_cnt_bytes_q < data_eff_write_bytes) begin : issue_cnt_bytes_overflow
          wr_issue_cnt_bytes_d = '0;
        end : issue_cnt_bytes_overflow
      end
    end
  end

  for (int i=0; i<NrClusters; i++) begin
    axi_resp_o[i].w_ready = w_cluster_ready_d[i];
  end
end : p_global_ldst

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    r_req_valid_q <= 1'b0;
    req_q         <= '0;
    vl_req_q      <= '0;
    vew_ar_q      <= vew_e'(0);
    vew_aw_q      <= vew_e'(0);
    vl_ldst_rd_q  <= '0;
    vl_ldst_wr_q  <= '0;

    w_req_valid_q <= 1'b0;
    vl_w_q        <= '0;

    cluster_axi_resp_data_q <= '0;
    cluster_start_r_q <= 0;

    cluster_start_wr_q <= 0;
    axi_req_data_q <= '0;

    w_cluster_ready_q <= '1;
    w_cluster_last_q  <= '0;

    axi_addrgen_state_q <= AXI_ADDRGEN_IDLE;

    wr_aligned_start_addr_q <= '0;
    wr_aligned_next_start_addr_q <= '0;
    wr_aligned_end_addr_q <= '0;
    wr_next_2page_msb_q <= '0;
    eff_axi_dw_q <= '0;
    eff_axi_dw_log_q <= '0;

    first_beat_q <= 1'b1;
    wr_axi_len_q <= '0;
    wr_issue_cnt_bytes_q <= '0;
    wr_data_pnt_q <= '0;
  end else begin
    r_req_valid_q <= r_req_valid_d;
    req_q         <= req_d;
    vl_req_q      <= vl_req_d;
    vew_ar_q      <= vew_ar_d;
    vew_aw_q      <= vew_aw_d;
    vl_ldst_rd_q  <= vl_ldst_rd_d;
    vl_ldst_wr_q  <= vl_ldst_wr_d;

    w_req_valid_q <= w_req_valid_d;
    vl_w_q        <= vl_w_d;

    cluster_axi_resp_data_q <= cluster_axi_resp_data_d;
    cluster_start_r_q <= cluster_start_r_d;

    cluster_start_wr_q <= cluster_start_wr_d;
    axi_req_data_q <= axi_req_data_d;

    w_cluster_ready_q <= w_cluster_ready_d;
    w_cluster_last_q  <= w_cluster_last_d; 

    axi_addrgen_state_q <= axi_addrgen_state_d;

    wr_aligned_start_addr_q <= wr_aligned_start_addr_d;
    wr_aligned_next_start_addr_q <= wr_aligned_next_start_addr_d;
    wr_aligned_end_addr_q <= wr_aligned_end_addr_d;
    wr_next_2page_msb_q <= wr_next_2page_msb_d;
    eff_axi_dw_q <= eff_axi_dw_d;
    eff_axi_dw_log_q <= eff_axi_dw_log_d;

    first_beat_q <= first_beat_d;
    wr_axi_len_q <= wr_axi_len_d;
    wr_issue_cnt_bytes_q <= wr_issue_cnt_bytes_d;
    wr_data_pnt_q <= wr_data_pnt_d;
  end
end

if (AxiDataWidth/ClusterAxiDataWidth > NrClusters)
  $error("AxiDataWidth > (NrClusters * ClusterAxiDataWidth) is not supported!! ");

endmodule : global_ldst

