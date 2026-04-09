// Copyright 2021-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This is Ara's slide unit. It is responsible for running the vector slide (up/down)
// instructions, which need access to the whole Vector Register File.

module sldu import ara_pkg::*; import rvv_pkg::*; #(
    parameter  int  unsigned NrLanes   = 0,
    parameter  type          vaddr_t = logic, // Type used to address vector register file elements
    // Dependant parameters. DO NOT CHANGE!
    localparam int  unsigned DataWidth = $bits(elen_t), // Width of the lane datapath
    localparam int  unsigned StrbWidth = DataWidth/8,
    localparam type          strb_t    = logic [StrbWidth-1:0], // Byte-strobe type
    localparam type          cluster_reduction_rx_cnt_t = logic [idx_width($clog2(MaxNrClusters)+1):0]
  ) (
    input  logic                   clk_i,
    input  logic                   rst_ni,
    // Id
    input  id_cluster_t            cluster_id_i,
    input num_cluster_t            num_clusters_i,
    // Interface with the main sequencer
    input  pe_req_t                pe_req_i,
    input  logic                   pe_req_valid_i,
    input  logic     [NrVInsn-1:0] pe_vinsn_running_i,
    output logic                   pe_req_ready_o,
    output pe_resp_t               pe_resp_o,
    // Interface with the lanes
    input  elen_t    [NrLanes-1:0] sldu_operand_i,
    input  target_fu_e [NrLanes-1:0] sldu_operand_target_fu_i,
    input  logic     [NrLanes-1:0] sldu_operand_queue_valid_i,
    input  logic     [NrLanes-1:0] sldu_red_operand_valid_i,
    output logic     [NrLanes-1:0] sldu_operand_ready_o,
    output logic     [NrLanes-1:0] sldu_result_req_o,
    output vid_t     [NrLanes-1:0] sldu_result_id_o,
    output vaddr_t   [NrLanes-1:0] sldu_result_addr_o,
    output elen_t    [NrLanes-1:0] sldu_result_wdata_o,
    output strb_t    [NrLanes-1:0] sldu_result_be_o,
    input  logic     [NrLanes-1:0] sldu_result_gnt_i,
    input  logic     [NrLanes-1:0] sldu_result_final_gnt_i,
    // Support for reductions
    output sldu_mux_e              sldu_issue_mux_sel_o,
    output sldu_mux_e              sldu_commit_mux_sel_o,
    output logic     [NrLanes-1:0] sldu_red_valid_o,
    output logic                   sldu_red_pending_o,
    output logic                   sldu_red_completed_o,
    // Interface with the Mask Unit
    input  strb_t    [NrLanes-1:0] mask_i,
    input  logic     [NrLanes-1:0] mask_valid_i,
    output logic                   mask_ready_o,
    // Interface with Ring Interconnect
    output remote_data_t sldu_ring_o,
    output logic  sldu_ring_valid_o, 
    input  logic  sldu_ring_ready_i, 

    input  remote_data_t sldu_ring_i,
    input  logic  sldu_ring_valid_i,
    output logic  sldu_ring_ready_o

  );

  `include "common_cells/registers.svh"

  import cf_math_pkg::idx_width;

  id_cluster_t max_cluster_id; 
  assign max_cluster_id = (1 << num_clusters_i) - 1;

  logic [$clog2(NrLanes*MaxNrClusters):0] total_lanes;
  assign total_lanes = NrLanes << num_clusters_i;

  ////////////////////////////////
  //  Vector instruction queue  //
  ////////////////////////////////

  // We store a certain number of in-flight vector instructions
  localparam VInsnQueueDepth = SlduInsnQueueDepth;

  struct packed {
    pe_req_t [VInsnQueueDepth-1:0] vinsn;

    // Each instruction can be in one of the three execution phases.
    // - Being accepted (i.e., it is being stored for future execution in this
    //   vector functional unit).
    // - Being issued (i.e., its micro-operations are currently being issued
    //   to the corresponding functional units).
    // - Waiting for data from the ring before it can be committed (e.g. for sliding or 
    //   reduction operations)
    // - Being committed (i.e., its results are being written to the vector
    //   register file).
    // We need pointers to index which instruction is at each execution phase
    // between the VInsnQueueDepth instructions in memory.
    logic [idx_width(VInsnQueueDepth)-1:0] accept_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] issue_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] ring_pnt;
    logic [idx_width(VInsnQueueDepth)-1:0] commit_pnt;

    // We also need to count how many instructions are queueing to be
    // issued/committed, to avoid accepting more instructions than
    // we can handle.
    logic [idx_width(VInsnQueueDepth):0] issue_cnt;
    logic [idx_width(VInsnQueueDepth):0] ring_cnt;
    logic [idx_width(VInsnQueueDepth):0] commit_cnt;
  } vinsn_queue_d, vinsn_queue_q;

  pe_req_t vinsn_issue_q;
  logic vinsn_issue_valid_q;
  // Is the vector instruction queue full?
  logic vinsn_queue_full;
  assign vinsn_queue_full = (vinsn_queue_q.commit_cnt == VInsnQueueDepth);

  // Do we have a vector instruction ready to be issued?
  `FF(vinsn_issue_q, vinsn_queue_d.vinsn[vinsn_queue_d.issue_pnt], '0)
  `FF(vinsn_issue_valid_q, vinsn_queue_d.issue_cnt != '0, 1'b0)

  // Do we have a vector instruction with results being committed?
  pe_req_t vinsn_commit;
  logic    vinsn_commit_valid;
  assign vinsn_commit       = vinsn_queue_q.vinsn[vinsn_queue_q.commit_pnt];
  assign vinsn_commit_valid = (vinsn_queue_q.commit_cnt != '0);

  // Do we have a vector instruction waiting for ring packets?
  pe_req_t vinsn_ring;
  logic    vinsn_ring_valid;
  assign vinsn_ring       = vinsn_queue_q.vinsn[vinsn_queue_q.ring_pnt];
  assign vinsn_ring_valid = (vinsn_queue_q.ring_cnt != '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_queue_q <= '0;
    end else begin
      vinsn_queue_q <= vinsn_queue_d;
    end
  end

  /////////////////////
  //  Result queues  //
  /////////////////////

  localparam int unsigned ResultQueueDepth = 3;

  // There is a result queue per lane, holding the results that were not
  // yet accepted by the corresponding lane.
  typedef struct packed {
    vid_t id;
    vaddr_t addr;
    elen_t wdata;
    strb_t be;
  } payload_t;

  // Result queue
  payload_t [ResultQueueDepth-1:0][NrLanes-1:0] result_queue_d, result_queue_q;
  logic     [ResultQueueDepth-1:0][NrLanes-1:0] result_queue_valid_d, result_queue_valid_q;
  // We need two pointers in the result queue. One pointer to
  // indicate with `payload_t` we are currently writing into (write_pnt),
  // and one pointer to indicate which `payload_t` we are currently
  // reading from and writing into the lanes (read_pnt).
  logic     [idx_width(ResultQueueDepth)-1:0]   result_queue_write_pnt_d, result_queue_write_pnt_q, result_queue_write_pnt_ring_d, result_queue_write_pnt_ring_q;
  logic     [idx_width(ResultQueueDepth)-1:0]   result_queue_read_pnt_d, result_queue_read_pnt_q;
  // We need to count how many valid elements are there in this result queue.
  logic     [idx_width(ResultQueueDepth):0]     result_queue_cnt_d, result_queue_cnt_q;
  // Vector to register the final grants from the operand requesters, which indicate
  // that the result was actually written in the VRF (while the normal grant just says
  // that the result was accepted by the operand requester stage
  logic     [NrLanes-1:0]                       result_final_gnt_d, result_final_gnt_q;

  // Is the result queue full?
  logic result_queue_full;
  assign result_queue_full = (result_queue_cnt_q == ResultQueueDepth);
  // Is the result queue empty?
  logic result_queue_empty;
  assign result_queue_empty = (result_queue_cnt_q == '0);

  always_ff @(posedge clk_i or negedge rst_ni) begin: p_result_queue_ff
    if (!rst_ni) begin
      result_queue_q           <= '0;
      result_queue_valid_q     <= '0;
      result_queue_write_pnt_q <= '0;
      result_queue_write_pnt_ring_q <= '0;
      result_queue_read_pnt_q  <= '0;
      result_queue_cnt_q       <= '0;
    end else begin
      result_queue_q           <= result_queue_d;
      result_queue_valid_q     <= result_queue_valid_d;
      result_queue_write_pnt_q <= result_queue_write_pnt_d;
      result_queue_write_pnt_ring_q <= result_queue_write_pnt_ring_d;
      result_queue_read_pnt_q  <= result_queue_read_pnt_d;
      result_queue_cnt_q       <= result_queue_cnt_d;
    end
  end

  ////////////////////////////////
  //  Spill-reg from the lanes  //
  ////////////////////////////////

  elen_t [NrLanes-1:0] sldu_operand;
  logic  [NrLanes-1:0] sldu_operand_valid;
  logic  [NrLanes-1:0] sldu_operand_ready;
  target_fu_e [NrLanes-1:0] sldu_operand_target_fu_d, sldu_operand_target_fu_q;

  // Don't handshake if the operands target the addrgen!
  // Moreover, when computing NP2 slides, loop over the same data!
  elen_t [NrLanes-1:0] sldu_operand_d;
  logic [NrLanes-1:0]  sldu_operand_valid_d;
  logic [NrLanes-1:0]  sldu_operand_ready_q;

  typedef enum logic {
    NP2_BUFFER_PNT,
    NP2_RESULT_PNT
  } np2_result_queue_pnt_e;

  typedef enum logic {
    NP2_EXT_SEL,
    NP2_LOOP_SEL
  } np2_loop_mux_e;
  np2_loop_mux_e np2_loop_mux_sel_d, np2_loop_mux_sel_q;

  logic slide_np2_buf_valid_d, slide_np2_buf_valid_q;
  
  // To have a reference data
  elen_t [NrLanes-1:0] sldu_operand_ref_d, sldu_operand_ref_q, sldu_operand_ref_p2, sldu_operand_ref;
  logic [NrLanes-1:0]  sldu_operand_ref_valid_d, sldu_operand_ref_valid_q, sldu_operand_ref_valid;
  logic [NrLanes-1:0]  sldu_operand_ref_ready;

  for (genvar l = 0; l < NrLanes; l++) begin
    spill_register #(
      .T(elen_t)
    ) i_sldu_spill_register (
      .clk_i  (clk_i                      ),
      .rst_ni (rst_ni                     ),
      .valid_i(sldu_operand_valid_d[l]    ),
      .ready_o(sldu_operand_ready_q[l]    ),
      .data_i (sldu_operand_d[l]          ),
      .valid_o(sldu_operand_valid[l]      ),
      .ready_i(sldu_operand_ready[l]      ),
      .data_o (sldu_operand[l]            )
    );
    
    // To buffer the input data for reference
    // Done since sldu_operand changes for non power of 2 slides
    spill_register #(
      .T(elen_t)
    ) i_sldu_ref_register (
      .clk_i  (clk_i                      ),
      .rst_ni (rst_ni                     ),
      .valid_i(sldu_operand_ref_valid_d[l]    ),
      .ready_o(),
      .data_i (sldu_operand_ref_d[l]          ),
      .valid_o(sldu_operand_ref_valid[l]      ),
      .ready_i(sldu_operand_ref_ready[l]      ),
      .data_o (sldu_operand_ref_p2[l]            )
    );
  end
  
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sldu_operand_ref_q <= '0;
      sldu_operand_ref_valid_q <= 1'b0;
    end else begin
      sldu_operand_ref_q <= sldu_operand_ref_d;
      sldu_operand_ref_valid_q <= sldu_operand_ref_valid_d;
    end
  end

  always_comb begin
    // Tracking a reference to send ring data
    sldu_operand_ref_d = np2_loop_mux_sel_q == NP2_EXT_SEL ? sldu_operand_d : sldu_operand_ref_q;
    sldu_operand_ref_valid_d = np2_loop_mux_sel_q == NP2_EXT_SEL ? sldu_operand_valid_d : sldu_operand_ref_valid_q;
    sldu_operand_ref = vinsn_issue_q.is_stride_np2 ? sldu_operand_ref_d : sldu_operand_ref_p2;
  end

  always_comb begin
    for (int l = 0; l < NrLanes; l++) begin
      sldu_operand_d[l] = sldu_operand_i[l];
      sldu_operand_valid_d[l] = (sldu_operand_queue_valid_i[l] && (sldu_operand_target_fu_i[l] == ALU_SLDU)) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} && sldu_red_operand_valid_i[l]);
      sldu_operand_ready_o[l] = sldu_operand_ready_q[l] & sldu_operand_valid_d[l];
    end
  end

  assign sldu_operand_target_fu_d = sldu_operand_target_fu_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      sldu_operand_target_fu_q <= target_fu_e'(1'b0);
    else
      sldu_operand_target_fu_q <= sldu_operand_target_fu_d;
  end

  //////////////////////////
  //  Cut from the masku  //
  //////////////////////////

  strb_t [NrLanes-1:0] mask_q;
  logic  [NrLanes-1:0] mask_valid_d, mask_valid_q;
  logic                mask_ready_d;
  logic  [NrLanes-1:0] mask_ready_q;

  for (genvar l = 0; l < NrLanes; l++) begin
    stream_register #(
      .T(strb_t)
    ) i_mask_operand_register (
      .clk_i     (clk_i           ),
      .rst_ni    (rst_ni          ),
      .clr_i     (1'b0            ),
      .testmode_i(1'b0            ),
      .data_o    (mask_q[l]       ),
      .valid_o   (mask_valid_q[l] ),
      .ready_i   (mask_ready_d    ),
      .data_i    (mask_i[l]       ),
      .valid_i   (mask_valid_d[l] ),
      .ready_o   (mask_ready_q[l] )
    );

    // Sample only SLDU mask valid
    assign mask_valid_d[l] = mask_valid_i[l] & ~vinsn_issue_q.vm & vinsn_issue_valid_q;
  end


  // Don't upset the masku with a spurious ready
  assign mask_ready_o = mask_ready_q[0] & mask_valid_i[0] & ~vinsn_issue_q.vm & vinsn_issue_valid_q & !(vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});

  ///////////////////
  //  NP2 Support  //
  ///////////////////

  // The SLDU only supports powers of two (p2) strides
  // Decompose the non-power-of-two (np2) slide in multiple p2 slides

  // We implement the np2 support here and fully process every input packet
  // singularly to comply with the undisturbed policy. We cannot use the VRF
  // as intermediate buffer; each VRF write is a commit.

  typedef logic [idx_width(8*NrLanes)-1:0] stride_t;

  stride_t                                  p2_stride_gen_stride_d;
  logic                                     p2_stride_gen_valid_d;
  logic                                     p2_stride_gen_update_d;
  logic [idx_width(idx_width(8*NrLanes)):0] p2_stride_gen_popc_q;
  stride_t                                  p2_stride_gen_stride_q;
  logic                                     p2_stride_gen_valid_q;

  p2_stride_gen #(
    .NrLanes (NrLanes)
  ) i_p2_stride_gen (
    .clk_i       (clk_i                 ),
    .rst_ni      (rst_ni                ),
    .stride_i    (p2_stride_gen_stride_d),
    .valid_i     (p2_stride_gen_valid_d ),
    .update_i    (p2_stride_gen_update_d),
    .popc_o      (p2_stride_gen_popc_q  ),
    .stride_p2_o (p2_stride_gen_stride_q),
    .valid_o     (p2_stride_gen_valid_q )
  );

  //////////////////
  //  Reductions  //
  //////////////////

  // Inter-lane reductions are performed with a logarithmic tree, and the result is
  // accumulated in the last Lane. Then, in the end, the result is passed to the first
  // lane for SIMD reduction
  logic [idx_width(NrLanes)-1:0] red_stride_cnt_d, red_stride_cnt_q;
  logic [idx_width(NrLanes):0] red_stride_cnt_d_wide;

  logic is_issue_reduction, is_issue_alu_reduction, is_issue_vmfpu_reduction;

  assign is_issue_alu_reduction   = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_Alu);
  assign is_issue_vmfpu_reduction = vinsn_issue_valid_q & (vinsn_issue_q.vfu == VFU_MFpu);
  assign is_issue_reduction       = is_issue_alu_reduction | is_issue_vmfpu_reduction;

  // Logic to request/write data from/to the ALU/MFPU during reductions. 
  always_comb begin
    sldu_issue_mux_sel_o = NO_RED;
    sldu_commit_mux_sel_o = NO_RED;

    if (is_issue_alu_reduction) begin
      sldu_issue_mux_sel_o = ALU_RED;
    end else if (is_issue_vmfpu_reduction) begin
      sldu_issue_mux_sel_o = MFPU_RED;
    end

    if (vinsn_commit_valid && (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu})) begin
      if (vinsn_commit.vfu == VFU_Alu) begin
        sldu_commit_mux_sel_o = ALU_RED;
      end else if (vinsn_commit.vfu == VFU_MFpu) begin
        sldu_commit_mux_sel_o = MFPU_RED;
      end
    end
  end

  /////////////////////
  //  SLDU DataPath  //
  /////////////////////

  // Input/output non-flat operands
  elen_t [NrLanes-1:0] sld_op_src;
  elen_t [NrLanes-1:0] sld_op_dst;

  // Input and output eew for reshuffling
  rvv_pkg::vew_e sld_eew_src;
  rvv_pkg::vew_e sld_eew_dst;

  // 0: slidedown, 1: slideup
  logic sld_dir;

  // The SLDU slides by powers of two
  logic [idx_width(4*NrLanes):0] sld_slamt;

  sldu_op_dp #(
    .NrLanes  (NrLanes    )
  ) i_sldu_op_dp (
    .op_i     (sld_op_src ),
    .slamt_i  (sld_slamt  ),
    .eew_src_i(sld_eew_src),
    .eew_dst_i(sld_eew_dst),
    .dir_i    (sld_dir    ),
    .op_o     (sld_op_dst )
  );

  //////////////////
  //  Slide unit  //
  //////////////////

  // Vector instructions currently running
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // Interface with the main sequencer
  pe_resp_t pe_resp;

  // State of the slide FSM
  typedef enum logic [3:0] {
    SLIDE_IDLE,
    SLIDE_RUN,
    SLIDE_RUN_VSLIDE1UP_FIRST_WORD,
    SLIDE_RUN_OSUM,
    SLIDE_WAIT_OSUM,
    SLIDE_NP2_SETUP,
    SLIDE_NP2_RUN,
    SLIDE_NP2_COMMIT,
    SLIDE_NP2_WAIT,
    SLIDE_LATENCY
  } slide_state_e;
  slide_state_e state_d, state_q;

  logic  [8*NrLanes-1:0] out_en_flat, out_en_seq;
  strb_t [NrLanes-1:0]   out_en;

  // Pointers in the input operand and the output result
  logic   [idx_width(NrLanes*StrbWidth):0] in_pnt_d, in_pnt_q;
  logic   [idx_width(NrLanes*StrbWidth):0] out_pnt_d, out_pnt_q;
  vaddr_t                                  vrf_pnt_d, vrf_pnt_q;

  // Remaining bytes of the current instruction in the issue phase
  vlen_t issue_cnt_d, issue_cnt_q;
  vlen_t inter_cluser_issue_limit_d, inter_cluser_issue_limit_q;

  // Respected by default: input_limit_d  = 8*NrLanes + out_pnt_d - in_pnt_d;
  // To enforce: output_limit_d = out_pnt_d + issue_cnt_d;
  logic [idx_width(MAXVL+1):0] output_limit_d, output_limit_q;

  // Remaining bytes of the current instruction in the commit phase
  vlen_t commit_cnt_d, commit_cnt_q;

  vlen_t ring_cnt_d, ring_cnt_q;

  /////////////////////////////
  //     Ring Interconnect   //
  /////////////////////////////

  elen_t fifo_ring_out, fifo_ring_inp;
  logic fifo_ring_ready_inp, fifo_ring_ready_out;
  logic fifo_ring_valid_out, fifo_ring_valid_inp;

  //// Stream FIFOs to handle ring data
  // Fifo to interact with input from ring
  localparam int RingFifoDepth = 4;
  
  // Assigning Interface
  id_cluster_t dst_cluster;
  id_lane_t src_lane, src_lane_d, src_lane_q;
  id_lane_t dst_lane;
  
  assign fifo_ring_inp = sldu_ring_i.data;
  assign fifo_ring_valid_inp = sldu_ring_valid_i; 
  assign sldu_ring_ready_o = fifo_ring_ready_out; 

  assign sldu_ring_o.data = fifo_ring_out;
  assign sldu_ring_o.src_cluster = cluster_id_i;
  assign sldu_ring_o.dst_cluster = dst_cluster;
  assign sldu_ring_o.dst_lane = dst_lane;
  assign sldu_ring_o.src_lane = src_lane;

  assign sldu_ring_valid_o = fifo_ring_valid_out;
  assign fifo_ring_ready_inp = sldu_ring_ready_i;

  assign sldu_red_pending_o = (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu}) && (commit_cnt_q != '0);

  elen_t [NrLanes-1:0] ring_data_prev_d, ring_data_prev_q;
  logic [NrLanes-1:0] ring_data_prev_valid_d, ring_data_prev_valid_q;
  logic slide_result_valid;
  logic [NrLanes-1:0] slide_data_accepted_d, slide_data_accepted_q;
  logic update_inp_op_pnt;

  // Some helper signals for slide operations
  vlen_cluster_t vl_tot, vl_rem;

  vlen_t last_elem_byte_idx;
  logic [2:0] last_elem_offset;

  id_cluster_t last_cluster_id;
  logic [$clog2(NrLanes)-1:0] last_lane_id;

  // For inter-cluster reductions
  logic send_data_ring, receive_data_ring;
  cluster_reduction_rx_cnt_t cluster_red_cnt_d, cluster_red_cnt_q;
  cluster_reduction_rx_cnt_t cluster_red_limit_d, cluster_red_limit_q;
  logic sld_dir_ring;

  logic is_edge_cluster;
  logic is_ring_reduction;

  // maximum 4 64-bit data send out of a cluster on to the ring
  vlen_t vl_cluster_d, vl_cluster_q;
  vlen_t n_ring_out_d, n_ring_out_q, n_ring_out_init_d, n_ring_out_init_q, n_ring_in_init_d, n_ring_in_init_q;
  vlen_t n_ring_in_d, n_ring_in_q;

  logic init_queue_d, init_queue_q;

  localparam int unsigned ADD_LATENCY_SLIDE = 8;
  logic [idx_width(ADD_LATENCY_SLIDE):0] cnt_latency_q, cnt_latency_d;
  logic use_latency_q, use_latency_d;
  vlen_t eff_stride_d, eff_stride_q;

  // Logic to decide whether to stall receiving packets from the ring
  logic ring_ready_to_receive;

  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ring
    if(~rst_ni) begin
      ring_data_prev_q       <= '0;
      ring_data_prev_valid_q <= '0;
      cluster_red_cnt_q      <= '0;
      ring_cnt_q             <= '0;
      n_ring_out_q           <= '0;
      n_ring_in_q            <= '0;
      vl_cluster_q           <= '0;
      init_queue_q           <= 1'b1;
      src_lane_q             <= '0;
      cnt_latency_q          <= ADD_LATENCY_SLIDE;
      use_latency_q              <= 1'b0;
      n_ring_out_init_q      <= '0;
      n_ring_in_init_q       <= '0;
      eff_stride_q           <= '0;
      slide_data_accepted_q  <= '0;
      cluster_red_limit_q    <= '0;
      inter_cluser_issue_limit_q <= '0;
    end else begin
      ring_data_prev_q       <= ring_data_prev_d;
      ring_data_prev_valid_q <= ring_data_prev_valid_d;
      cluster_red_cnt_q      <= cluster_red_cnt_d;
      ring_cnt_q             <= ring_cnt_d;
      n_ring_out_q           <= n_ring_out_d;
      n_ring_in_q            <= n_ring_in_d;
      vl_cluster_q           <= vl_cluster_d;
      init_queue_q           <= init_queue_d;  
      src_lane_q             <= src_lane_d;
      cnt_latency_q          <= cnt_latency_d;
      use_latency_q              <= use_latency_d;
      n_ring_out_init_q      <= n_ring_out_init_d;
      n_ring_in_init_q       <= n_ring_in_init_d;
      eff_stride_q           <= eff_stride_d;
      slide_data_accepted_q  <= slide_data_accepted_d;
      cluster_red_limit_q    <= cluster_red_limit_d;
      inter_cluser_issue_limit_q <= inter_cluser_issue_limit_d;
    end
  end
  
  always_comb begin: p_sldu
    // Maintain state
    vinsn_queue_d = vinsn_queue_q;
    issue_cnt_d   = issue_cnt_q;
    ring_cnt_d    = ring_cnt_q;
    commit_cnt_d  = commit_cnt_q;
    in_pnt_d      = in_pnt_q;
    out_pnt_d     = out_pnt_q;
    vrf_pnt_d     = vrf_pnt_q;
    state_d       = state_q;

    result_queue_d           = result_queue_q;
    result_queue_valid_d     = result_queue_valid_q;
    result_queue_read_pnt_d  = result_queue_read_pnt_q;
    result_queue_write_pnt_d = result_queue_write_pnt_q;
    result_queue_cnt_d       = result_queue_cnt_q;

    result_final_gnt_d = result_final_gnt_q;
    slide_data_accepted_d = slide_data_accepted_q;

    // Vector instructions currently running
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    out_en_flat    = '0;
    out_en_seq     = '0;
    out_en         = '0;
    output_limit_d = output_limit_q;

    // We are not ready, by default
    pe_resp            = '0;
    mask_ready_d       = 1'b0;
    sldu_operand_ready = '0;


    red_stride_cnt_d = red_stride_cnt_q;

    p2_stride_gen_stride_d = '0;
    p2_stride_gen_valid_d  = 1'b0;
    p2_stride_gen_update_d = 1'b0;

    np2_loop_mux_sel_d    = np2_loop_mux_sel_q;
    slide_np2_buf_valid_d = slide_np2_buf_valid_q;

    red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};

    // Inform the main sequencer if we are idle
    pe_req_ready_o = !vinsn_queue_full;

    // Slide Unit DP
    sld_op_src  = sldu_operand;
    sld_eew_src = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? vinsn_issue_q.vtype.vsew
                : vinsn_issue_q.eew_vs2;
    sld_eew_dst = vinsn_issue_q.vtype.vsew;
    sld_dir     = (vinsn_issue_q.op == VSLIDEUP) || (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu});
    sld_slamt   = (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
                ? red_stride_cnt_q
                : stride_t'(eff_stride_d >> vinsn_issue_q.vtype.vsew);

    // Ring Interconnect states
    sld_dir_ring = (vinsn_issue_q.op == VSLIDEDOWN) ? 1'b0 : 1'b1;
    
    fifo_ring_out = '0; 
    fifo_ring_valid_out = 1'b0;
    fifo_ring_ready_out = 1'b0;

    ring_data_prev_d = ring_data_prev_q;
    ring_data_prev_valid_d = ring_data_prev_valid_q;

    result_queue_write_pnt_ring_d = result_queue_write_pnt_ring_q;

    cluster_red_cnt_d = cluster_red_cnt_q;
    cluster_red_limit_d = cluster_red_limit_q;
    inter_cluser_issue_limit_d = inter_cluser_issue_limit_q;

    slide_result_valid = 1'b0;

    dst_cluster = '0;
    update_inp_op_pnt = 1'b0;

    n_ring_out_d = n_ring_out_q;
    n_ring_in_d = n_ring_in_q;
    dst_lane = '0;
    src_lane = '0;

    sldu_operand_ref_ready = '0;
    vl_cluster_d = vl_cluster_q;
    
    init_queue_d = init_queue_q;
    src_lane_d = src_lane_q;
    
    cnt_latency_d = cnt_latency_q;
    use_latency_d = use_latency_q;

    n_ring_out_init_d = n_ring_out_init_q;
    n_ring_in_init_d = n_ring_in_init_q;
    
    eff_stride_d = eff_stride_q;

    send_data_ring = 1'b0;
    receive_data_ring = 1'b0;

    sldu_red_completed_o = 1'b0;

    // Some helper signals to handle sliding logic
    vl_tot = vinsn_ring.vl_cluster;
    // If no more ring packets to receive and reductions in progress use the issue stage of request to know the vector length
    if (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} && vinsn_issue_valid_q) begin
      vl_tot = vinsn_issue_q.vl_cluster;
    end
    vl_rem = vl_tot & ((1 << num_clusters_i << $clog2(NrLanes)) - 1);

    // Find where the cluster/lane and the offset scalar op has to be written for slidedown operations
    // Or which is the last cluster participating in the reductions
    if (vl_rem == 0) begin
      last_cluster_id = max_cluster_id;
      last_lane_id = NrLanes - 1;
    end else begin
      last_cluster_id = (vl_rem-1) >> $clog2(NrLanes);
      last_lane_id = (vl_rem-1) & (NrLanes-1);
    end

    /////////////////
    //  Slide FSM  //
    /////////////////

    unique case (state_q)
      SLIDE_IDLE: begin
        // To check if ara's in sync to avoid data getting lost in the ring.
        // E.g. for slideup -> slidedown instr., ring direction changes
        // and so if all ring packets not handled properly, it stalls.
        automatic logic inSync = vinsn_issue_valid_q && 
                                   (vinsn_queue_q.issue_pnt == vinsn_queue_q.ring_pnt);
        // if (vinsn_issue_valid_q) begin
        if (inSync) begin
          state_d   = vinsn_issue_q.is_stride_np2 ? SLIDE_NP2_SETUP : SLIDE_RUN;
          vrf_pnt_d = '0;

          unique case (vinsn_issue_q.op)
            VSLIDEUP: begin
              automatic int cluster_strides = vinsn_issue_q.stride >> $clog2(8*NrLanes);
              automatic vlen_t stride = (vinsn_issue_q.stride >> int'(vinsn_issue_q.vtype.vsew));
              automatic vlen_t eff_stride;
              
              // Supporting only slides by 1 for now.
              // vslideup starts reading the source operand from its beginning
              in_pnt_d  = '0;
              // vslideup starts writing the destination vector at the slide offset
              out_pnt_d = '0;
              
              vrf_pnt_d = vinsn_issue_q.stride >> ($clog2(8*NrLanes) + num_clusters_i);
              src_lane_d = '0;
              
              // If the stride is more than 4 but not multiple of 4
              // Then we have 2 adjacent clusters sending packets to the same destination cluster
              // In this scenario, the closest cluster can send the next transaction quicker reaching the destination before the previous data 
              // from the farther cluster reaches the destination.
              // This should be avoided. The simple solution is to wait for a few clocks before sending the next set of packets.
              if ((stride > NrLanes) && (stride[1:0]!=0))
                use_latency_d = 1'b1;

              eff_stride = vinsn_issue_q.stride - (vrf_pnt_d * ((8 * NrLanes) << num_clusters_i));
              n_ring_out_d = eff_stride >> vinsn_issue_q.vtype.vsew;
              n_ring_out_d = (n_ring_out_d > NrLanes) ? NrLanes : n_ring_out_d;
              n_ring_out_init_d = n_ring_out_d;
              eff_stride_d = eff_stride;

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);

              // Initialize be-enable-generation ancillary signals
              output_limit_d = issue_cnt_d;  

              // Go to SLIDE_RUN_VSLIDE1UP_FIRST_WORD if this is a vslide1up instruction
              if (vinsn_issue_q.use_scalar_op)
                state_d = SLIDE_RUN_VSLIDE1UP_FIRST_WORD;
            end
            VSLIDEDOWN: begin
              automatic vlen_t cluster_strides = vinsn_issue_q.stride >> ($clog2(8*NrLanes) + num_clusters_i);
              automatic vlen_t eff_stride = vinsn_issue_q.stride - (cluster_strides * ((8 * NrLanes) << num_clusters_i));
              automatic vlen_t eff_elem_stride = (eff_stride>> int'(vinsn_issue_q.vtype.vsew));
              
              // Supporting only slides by 1 for now.
              
              // vslidedown starts reading the source operand from the slide offset
              in_pnt_d  = 0; // vinsn_issue_q.stride[idx_width(8*NrLanes)-1:0];
              // vslidedown starts writing the destination vector at its beginning
              out_pnt_d = '0;

              if (vinsn_issue_q.stride > ((1 << vinsn_issue_q.vtype.vsew) * NrLanes * cluster_id_i)) begin
                automatic int offset = vinsn_issue_q.stride - ((1 << vinsn_issue_q.vtype.vsew) * NrLanes * cluster_id_i);
                in_pnt_d = offset[$clog2(8*NrLanes)-1:0]; // in bytes
              end else begin
                in_pnt_d = '0;
              end
              src_lane_d = '0;
              vrf_pnt_d = '0;

              if ((eff_elem_stride > NrLanes) && (eff_elem_stride[1:0]!=0))
                use_latency_d = 1'b1;

              n_ring_out_d = eff_elem_stride > NrLanes ? NrLanes : eff_elem_stride;
              n_ring_out_init_d = n_ring_out_d;

              eff_stride_d = eff_stride;

              // Initialize counters
              issue_cnt_d = vinsn_issue_q.vl << int'(vinsn_issue_q.vtype.vsew);
              output_limit_d = issue_cnt_d;

            end
            // Ordered sum reductions
            VFREDOSUM, VFWREDOSUM: begin
              // Ordered redsum instructions doesn't need in/out_pnt
              in_pnt_d  = '0;
              out_pnt_d = '0;

              // The total number of transactions is vl - 1, but the last data is sent
              // to lane 0
              issue_cnt_d  = vinsn_issue_q.vl;

              state_d = SLIDE_RUN_OSUM;
            end
            // Unordered reductions
            default: begin
              // Unordered redsum instructions doesn't need in/out_pnt
              in_pnt_d  = '0;
              out_pnt_d = '0;
              cluster_red_cnt_d = cluster_reduction_rx_cnt_init(cluster_id_i);

              // Initialize issue cnt. Pretend to move NrLanes 64-bit elements for (clog2(NrLanes) + 1) times.
              issue_cnt_d  = NrLanes * ($clog2(NrLanes) + cluster_reduction_rx_cnt_init(cluster_id_i)) << EW64;
              inter_cluser_issue_limit_d = NrLanes * cluster_reduction_rx_cnt_init(cluster_id_i) << EW64;

              // Handle edge cases when all clusters don't participate
              // The last participating cluster has to do the reductions
              if (vinsn_issue_q.vl_cluster <= ((cluster_id_i+1) * NrLanes)) begin
                issue_cnt_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
                inter_cluser_issue_limit_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
                cluster_red_cnt_d += ext_reduction_rx_cnt_init(cluster_id_i);
              end
              output_limit_d = issue_cnt_d;
              cluster_red_limit_d = cluster_red_cnt_d;  
            end
          endcase
        end
      end

      SLIDE_RUN, SLIDE_RUN_VSLIDE1UP_FIRST_WORD, SLIDE_NP2_COMMIT: begin
        // Are we ready?
        // During a reduction (vinsn_issue_q.vfu == VFU_Alu/VFU_MFPU) don't wait for mask bits
        if (&sldu_operand_valid && !result_queue_full && (vinsn_issue_q.vm || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} || (|mask_valid_q)))
        begin
        
          // Build the sequential byte-output-enable
          for (int unsigned b = 0; b < 8*NrLanes; b++)
            if ((b >= out_pnt_q && b < output_limit_q) || vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})
              out_en_seq[b] = 1'b1;

          // Shuffle the output enable
          for (int unsigned b = 0; b < 8*NrLanes; b++)
            out_en_flat[shuffle_index(b, NrLanes, vinsn_issue_q.vtype.vsew)] = out_en_seq[b];

          // Mask the output enable with the mask vector
          out_en = out_en_flat & ({8*NrLanes{vinsn_issue_q.vm | (vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})}} | mask_q);

          // Write in the correct bytes
          if (init_queue_q) begin
            // To initialize the queue only once with the local shifted data
            // The ring data is written later on top of the initialized data
            for (int lane = 0; lane < NrLanes; lane++)
              for (int b = 0; b < 8; b++)
                if (out_en[lane][b]) begin
                  result_queue_d[result_queue_write_pnt_q][lane].wdata[8*b +: 8] = sld_op_dst[lane][8*b +: 8];
                  result_queue_d[result_queue_write_pnt_q][lane].be[b]           = 1'b1;
                end else begin
                  // Unchanged policy
                  result_queue_d[result_queue_write_pnt_q][lane].wdata[8*b +: 8] = sldu_operand_ref[lane][8*b +: 8];
                  result_queue_d[result_queue_write_pnt_q][lane].be[b]           = 1'b0;
                end
            init_queue_d = 1'b0;
          end

          // Initialize id and addr fields of the result queue requests
          for (int lane = 0; lane < NrLanes; lane++) begin
            result_queue_d[result_queue_write_pnt_q][lane].id   = vinsn_issue_q.id;
            result_queue_d[result_queue_write_pnt_q][lane].addr =
              vaddr(vinsn_issue_q.vd, NrLanes) + vrf_pnt_q;
          end

          // Depending on the operation being performed, decide whether to update pointers
          // Also decide whether to use the ring or not for reductions.
          if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN}) begin
            
            if (n_ring_out_d) begin 
              // Slide operation
              if (vinsn_issue_q.op==VSLIDEDOWN) begin
                automatic int dstlane = (NrLanes*cluster_id_i) + src_lane_d - (eff_stride_d >> int'(vinsn_issue_q.vtype.vsew));
                if (dstlane < 0)
                  dstlane += total_lanes;
                dst_cluster = dstlane / NrLanes;
                if (dst_cluster > last_cluster_id && vinsn_issue_q.vl_cluster <= total_lanes)
                  dst_cluster = last_cluster_id;
                dst_lane = dstlane[$clog2(NrLanes)-1:0];
                src_lane = src_lane_q;
              end else begin
                automatic int srclane = NrLanes - n_ring_out_q;
                automatic int dstlane = (NrLanes*cluster_id_i) + srclane + (eff_stride_d >> int'(vinsn_issue_q.vtype.vsew));
                src_lane = srclane;
                if (dstlane < total_lanes)
                  dst_cluster = dstlane / NrLanes;
                else
                  dst_cluster = (dstlane - total_lanes) / NrLanes;
                if (dst_cluster > last_cluster_id && vinsn_issue_q.vl_cluster <= total_lanes)
                  dst_cluster = '0;
                dst_lane = dstlane[$clog2(NrLanes)-1:0];
              end

              fifo_ring_out = sldu_operand_ref[src_lane];
              fifo_ring_valid_out = 1'b1;

              if (fifo_ring_ready_inp) begin
                n_ring_out_d = n_ring_out_q - 1;
                // If this was the last packet to be send on the ring, update pointer
                update_inp_op_pnt = (n_ring_out_q == 1) ? 1'b1 : 1'b0;
                src_lane_d = src_lane_q + 1;
              end
            end

            if (n_ring_in_q == 0) begin
              update_inp_op_pnt = 1'b1;
              slide_result_valid = 1'b1;
            end

            // Assertion: update_inp_op_pnt should be 1 only with valid handshake or when done receiving
            `ifndef VERILATOR
            assert(~update_inp_op_pnt | (n_ring_out_q ? (fifo_ring_valid_out & fifo_ring_ready_inp) : (n_ring_in_q == 0)))
              else $error("update_inp_op_pnt set without valid handshake on FIFO for SLIDE operations");
            `endif

          end else begin
            // Reduction operation
            if (issue_cnt_q <= inter_cluser_issue_limit_q) begin

              ///// INTER-CLUSTER REDUCTION WITH RING /////

              // Now we have Inter Cluster reduction, where ring is used.
              dst_lane = NrLanes-1;

              // Check if cluster is still participating in reduction
              // Receive data until all reduction completes
              // Then send the reduced local result with other clusters
              if (cluster_red_cnt_q > 1) begin
                receive_data_ring = 1'b1;
              end else begin
                send_data_ring = 1'b1;
              end

              if (send_data_ring) begin
                
                ///// SEND DATA ON RING FOR REDUCTION /////

                fifo_ring_out = sldu_operand[NrLanes-1];
                fifo_ring_valid_out = 1'b1;
                
                // Find the next cluster to reduce with
                dst_cluster = cluster_id_i + (1 << (cluster_red_limit_d-1));

                // Handle small vl
                if ((dst_cluster > last_cluster_id) && (vinsn_issue_q.vl_cluster <= total_lanes)) begin
                  dst_cluster = last_cluster_id;
                end

                // Ensure within max cluster id
                if (dst_cluster > max_cluster_id) begin
                  dst_cluster = 0;
                end

                // Also if this cluster is already the last one, send to cluster0
                if ((cluster_id_i == last_cluster_id) && (vl_tot <= (cluster_id_i+1)*NrLanes)) begin
                  dst_cluster = 0;
                end

                if (fifo_ring_ready_inp) begin
                  update_inp_op_pnt = 1'b1;
                  cluster_red_cnt_d = cluster_red_cnt_q - 1;
                  `ifndef VERILATOR
                  assert (cluster_red_cnt_q == 1) else $error("cluster_red_cnt_q should not be 0 when sending data on the ring for reduction");
                  `endif
                end
              end else if (receive_data_ring) begin
                
                ///// RECEIVE DATA FROM RING FOR REDUCTION /////
                
                update_inp_op_pnt = 1'b1;

              end else begin
                `ifndef VERILATOR
                assert (0) else $error("Should be either sending on receiving data on the ring for reduction");
                `endif
              end

            end else begin
              
              ///// INTER-LANE REDUCTION /////

              // Inter lane reduction still in progress, so don't use ring yet
              update_inp_op_pnt = 1'b1;
              slide_result_valid = 1'b1;
            end
          end

          // Sldu can receive next packet from Lanes
          if (update_inp_op_pnt) begin 
            in_pnt_d    = NrLanes * 8;
            out_pnt_d   = NrLanes * 8;
            if (output_limit_q > NrLanes * 8) begin
              output_limit_d = output_limit_q - NrLanes * 8;
            end else begin
              output_limit_d = 0;
              `ifndef VERILATOR
              assert(output_limit_q != 0) else $error("output_limit_d should not be 0 here");
              `endif
            end
          end

          // In Jump to SLIDE_RUN if stride is P2
          if (state_q != SLIDE_NP2_COMMIT)
            state_d = SLIDE_RUN;

          // Read a full word from the VRF or finished the instruction
          if (in_pnt_d == NrLanes * 8) begin
            // Reset the pointer and ask for a new operand
            in_pnt_d           = '0;
            sldu_operand_ready = '1;
            sldu_operand_ref_ready = '1;
            
            // Left-rotate the logarithmic counter. Hacky way to write it, but it's to
            // deal with the 2-lanes design without complaints from Verilator...
            // wide signal to please the tool
            red_stride_cnt_d_wide = {red_stride_cnt_q, red_stride_cnt_q[idx_width(NrLanes)-1]};
            red_stride_cnt_d      = red_stride_cnt_d_wide[idx_width(NrLanes)-1:0];

            // Reset the input data mux
            np2_loop_mux_sel_d = NP2_EXT_SEL;

          end

          // Filled up a word to the VRF or finished the instruction
          if (out_pnt_d == NrLanes * 8) begin
            // Reset the pointer
            out_pnt_d = vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu} ? {'0, red_stride_cnt_d, 3'b0} : '0;
            // We used all the bits of the mask
            if (vinsn_issue_q.op inside {VSLIDEUP, VSLIDEDOWN})
              mask_ready_d = !vinsn_issue_q.vm;

            // Increment VRF address
            vrf_pnt_d = vrf_pnt_q + 1;

            init_queue_d = 1'b1;

            if (issue_cnt_q >= 8*NrLanes) begin
              issue_cnt_d = issue_cnt_q - 8*NrLanes;
            end else begin
              issue_cnt_d = '0;
              `ifndef VERILATOR
              assert(issue_cnt_q != 0) else $error("issue_cnt_q should not be 0 here");
              `endif
            end
            
            // For the last issue only lesser packets need to be send
            n_ring_out_d = n_ring_out_init_d;

            src_lane_d = '0;

            // Go to an intermediate state
            if (state_q == SLIDE_NP2_COMMIT) begin
              // Jump to NP2 setup again
              state_d = SLIDE_NP2_SETUP;
            end

            if (use_latency_q)
              state_d = SLIDE_LATENCY;

            if (issue_cnt_q <= 8*NrLanes) begin
                // Assertion: issue_cnt_q should not be 0 at this point
                `ifndef VERILATOR
                assert(issue_cnt_q != 0)
                  else $error("issue_cnt_q should not be 0 here");
                `endif
                
                state_d = SLIDE_IDLE;
                
                // Reset the logarighmic counter
                red_stride_cnt_d = 1;

                // Go to the next instruction to be issued
                vinsn_queue_d.issue_pnt += 1;
                if (vinsn_queue_d.issue_pnt == VInsnQueueDepth) begin 
                  vinsn_queue_d.issue_pnt = '0; 
                end
                vinsn_queue_d.issue_cnt -= 1;
            end

            // For reductions don't need to update pointers, we just maintain one entry in the result queue
            result_queue_cnt_d += send_data_ring ? 1'b0 : 1'b1;
            if (!(vinsn_issue_q.vfu inside {VFU_Alu, VFU_MFpu})) begin
              result_queue_write_pnt_d = result_queue_write_pnt_q+1;
              if (result_queue_write_pnt_q == ResultQueueDepth-1)
                result_queue_write_pnt_d = '0;
            end
          end

        end
        if (state_q == SLIDE_NP2_COMMIT)
          if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
            // Reset the buffer-valid if the buffer is read, by default
            slide_np2_buf_valid_d = 1'b0;

      end
      SLIDE_LATENCY: begin
        if (cnt_latency_q) begin
          cnt_latency_d = cnt_latency_q - 1;
        end else begin
          cnt_latency_d = ADD_LATENCY_SLIDE;
          state_d = vinsn_issue_q.is_stride_np2 ? SLIDE_NP2_SETUP : SLIDE_RUN;
        end
        if (sldu_operand_queue_valid_i[0])
          np2_loop_mux_sel_d = NP2_LOOP_SEL;
      end
      SLIDE_RUN_OSUM: begin
        // Short Note: For ordered sum reduction instruction, only one lane has a valid data, and it is sent to the next lane
        // Don't wait for mask bits
        if (!result_queue_full) begin
          for (int lane = 0; lane < NrLanes; lane++) begin
            if (sldu_operand_valid[lane]) begin
              automatic int tgt_lane = (lane == NrLanes - 1) ? 0 : lane + 1;
              // Send result to lane 0
              if (issue_cnt_q == 1) tgt_lane = 0;

              // Acknowledge the received operand
              sldu_operand_ready[lane] = 1'b1;

              // Send result to next lane
              result_queue_d[result_queue_write_pnt_q][tgt_lane].wdata =
                sldu_operand[lane];
              result_queue_d[result_queue_write_pnt_q][tgt_lane].be =
                {8{vinsn_issue_q.vm}} | mask_q[tgt_lane];
              result_queue_valid_d[result_queue_write_pnt_q][tgt_lane] = '1;

              issue_cnt_d = issue_cnt_q - 1;
            end
          end
        end

        // Finish the operation
        if (issue_cnt_d == '0) begin
          state_d      = SLIDE_WAIT_OSUM;
          // Increment vector instruction queue pointers and counters
          vinsn_queue_d.issue_pnt += 1;
          vinsn_queue_d.issue_cnt -= 1;
        end
      end
      SLIDE_WAIT_OSUM: begin
        // Wait one cycle for the last result processing
        commit_cnt_d = 1'b0;
        state_d      = SLIDE_IDLE;
      end
      SLIDE_NP2_SETUP: begin
        // Prepare the write pointer
        result_queue_write_pnt_d = NP2_BUFFER_PNT;
        // Prepare the read pointer
        result_queue_read_pnt_d = NP2_RESULT_PNT;
        // Setup the mux sel as soon as we get one operand
        if (sldu_operand_queue_valid_i[0])
          np2_loop_mux_sel_d = NP2_LOOP_SEL;
        // Setup the p2-stride generator
        p2_stride_gen_stride_d = stride_t'(eff_stride_d >> vinsn_issue_q.vtype.vsew); // stride_t'(vinsn_issue_q.stride >> vinsn_issue_q.vtype.vsew);
        p2_stride_gen_stride_d = {'0, p2_stride_gen_stride_d[$clog2(NrLanes)-1:0]}; // Maximum slide amount of only to NrLanes-1
        p2_stride_gen_valid_d  = 1'b1;
        // Start processing the first VRF chunk as soon as the result queue is completely empty
        if (np2_loop_mux_sel_q == NP2_LOOP_SEL && result_queue_empty) begin
          state_d = SLIDE_NP2_RUN;
        end
      end
      SLIDE_NP2_RUN: begin
        // Reset the buffer-valid if the buffer is read, by default
        if (slide_np2_buf_valid_q && sldu_operand_ready_q[0])
          slide_np2_buf_valid_d = 1'b0;
        // Setup the current p2 stride
        sld_slamt = p2_stride_gen_stride_q;
        // Slide the operands as soon as valid
        if (&sldu_operand_valid) begin
          for (int unsigned l = 0; l < NrLanes; l++)
            result_queue_d[result_queue_write_pnt_q][l].wdata = sld_op_dst[l];
          slide_np2_buf_valid_d = 1'b1;
          // Operands correctly read
          sldu_operand_ready     = '1;
          // Update the p2 stride
          p2_stride_gen_update_d = 1'b1;
          // Commit the final result
          if (p2_stride_gen_popc_q == {'0, 1'b1} && result_queue_empty) begin
            state_d = SLIDE_NP2_COMMIT;
            // Prepare the write pointer
            result_queue_write_pnt_d = NP2_RESULT_PNT;
            // Ring data should also update to this pointer
            result_queue_write_pnt_ring_d = NP2_RESULT_PNT;
          end
        end
      end
      SLIDE_NP2_WAIT: begin
        if (result_queue_empty) begin
          result_queue_read_pnt_d  = NP2_RESULT_PNT;
          result_queue_write_pnt_d = NP2_RESULT_PNT;
          state_d = SLIDE_NP2_COMMIT;
        end
      end
      default:;
    endcase

    //////////////////////////////////
    //  Write results into the VRF  //
    //////////////////////////////////

    for (int lane = 0; lane < NrLanes; lane++) begin: result_write
      sldu_result_req_o[lane]   = result_queue_valid_q[result_queue_read_pnt_q][lane] & (~(vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu}));
      sldu_red_valid_o[lane]    = result_queue_valid_q[result_queue_read_pnt_q][lane] & (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu});
      sldu_result_addr_o[lane]  = result_queue_q[result_queue_read_pnt_q][lane].addr;
      sldu_result_id_o[lane]    = result_queue_q[result_queue_read_pnt_q][lane].id;
      sldu_result_wdata_o[lane] = result_queue_q[result_queue_read_pnt_q][lane].wdata;
      sldu_result_be_o[lane]    = result_queue_q[result_queue_read_pnt_q][lane].be;

      // Update the final gnt vector
      result_final_gnt_d[lane] |= sldu_result_final_gnt_i[lane];

      // Received a grant from the VRF (slide) or from the FUs (reduction).
      // Deactivate the request, but do not bump the pointers for now.
      if (((vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu} && sldu_red_valid_o[lane]) || sldu_result_req_o[lane]) && sldu_result_gnt_i[lane]) begin
        result_queue_valid_d[result_queue_read_pnt_q][lane] = 1'b0;
        result_queue_d[result_queue_read_pnt_q][lane]       = '0;
        // Reset the final gnt vector since we are now waiting for another final gnt
        result_final_gnt_d[lane] = 1'b0;
        slide_data_accepted_d[lane] = 1'b1;
      end
    end: result_write

    // All lanes accepted the VRF request
    // If this was the last request, wait for all the final grants!
    // If this is a reduction, no need for the final grants
    if (&slide_data_accepted_d) begin
      slide_data_accepted_d = '0;
      // There is something waiting to be written
      if (!result_queue_empty) begin
        if (!(vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu})) begin
          // Increment the read pointer
          if (result_queue_read_pnt_q == ResultQueueDepth-1)
            result_queue_read_pnt_d = 0;
          else
            result_queue_read_pnt_d = result_queue_read_pnt_q + 1;
        end

        // Decrement the counter of results waiting to be written
        result_queue_cnt_d -= 1;

        // Decrement the counter of remaining vector elements waiting to be written
        commit_cnt_d = commit_cnt_q - NrLanes * 8;
        if (commit_cnt_q <= (NrLanes * 8)) begin
          commit_cnt_d = '0;

          // Signal functional units that reduction has completed
          if (vinsn_commit.vfu inside {VFU_Alu, VFU_MFpu}) begin
            sldu_red_completed_o = 1'b1;
          end

          `ifndef VERILATOR
          assert(commit_cnt_q != 0) 
            else $error("commit_cnt_q should not be 0 here");
          assert((vinsn_queue_q.commit_pnt == vinsn_queue_q.ring_pnt) ? (commit_cnt_q >= ring_cnt_q) : 1'b1)
            else $error("committing to VRF cannot be ahead of receiving packets on the ring for the same instruction");
          `endif
        end
      end
    end

    // Finished committing the results of a vector instruction
    if (vinsn_commit_valid && commit_cnt_d == '0) begin
      automatic elen_t stride = vinsn_queue_d.vinsn[vinsn_queue_d.ring_pnt].stride;
      automatic logic [$clog2(MAXVL/8):0] cluster_strides = stride >> ($clog2(NrLanes * 8) + num_clusters_i);
      // Mark the vector instruction as being done
      pe_resp.vinsn_done[vinsn_commit.id] = 1'b1;

      // Update the commit counters and pointers
      vinsn_queue_d.commit_cnt -= 1;
      if (vinsn_queue_d.commit_pnt == VInsnQueueDepth-1)
        vinsn_queue_d.commit_pnt = '0;
      else
        vinsn_queue_d.commit_pnt += 1;

      // Update the commit counter for the next instruction
      if (vinsn_queue_d.commit_cnt != '0) begin
        automatic pe_req_t commit_vinsn = vinsn_queue_q.vinsn[vinsn_queue_d.commit_pnt];
        automatic elen_t stride = commit_vinsn.stride;
        automatic logic [$clog2(MAXVL/8):0] cluster_strides = stride >> ($clog2(NrLanes * 8) + num_clusters_i);
        commit_cnt_d = commit_vinsn.op inside {VSLIDEUP, VSLIDEDOWN}
                     ? commit_vinsn.vl << int'(commit_vinsn.vtype.vsew)
                     : (NrLanes * $clog2(NrLanes)) << EW64;

        // Add packets for inter cluster reduction
        if (commit_vinsn.vfu inside {VFU_Alu, VFU_MFpu}) begin
          // results to be committed after receiving from ring, cluster0 commits 1 additional packet for final result
          commit_cnt_d += (NrLanes * (cluster_reduction_rx_cnt_init(cluster_id_i) - 1 + (cluster_id_i==0 ? 1 : 0))) << EW64;
          
          // Handle edge case when not all clusters participate in reduction
          if (commit_vinsn.vl_cluster <= (cluster_id_i+1)*NrLanes) begin
            commit_cnt_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
          end
        end

        // Trim vector elements which are not written by the slide unit
        if (commit_vinsn.op == VSLIDEUP) 
          commit_cnt_d -= cluster_strides * 8 * NrLanes;
      end
    end

    //////////////////////////////////
    //    Handle data from Ring     //
    //////////////////////////////////

    // Check if we have a valid data from the input fifo from the ring
    // and set the ring data to the desired lane
    // For the clusters at the edge (Cluster-0 or NrClusters-1), use the current ring data and previous ring data.
    // For the other clusters, set the input ring data to the edge lane of the current cluster.
    // If this is inter lane reduction, where ring is not needed, we just set data to valid.
    
    is_edge_cluster = (cluster_id_i == max_cluster_id && vinsn_ring.op==VSLIDEDOWN) || (cluster_id_i==0 && vinsn_ring.op==VSLIDEUP);
    is_ring_reduction = vinsn_ring.vfu inside {VFU_Alu, VFU_MFpu};
    
    last_elem_byte_idx = shuffle_index((vinsn_ring.vl-1) << vinsn_ring.vtype.vsew, NrLanes, vinsn_ring.vtype.vsew);
    last_elem_offset = last_elem_byte_idx[2:0]; // Find offset within the 8B
    
    
    if (vinsn_ring_valid && (vinsn_ring.vl > 0) && ((n_ring_in_q > 0) || is_ring_reduction)) begin
      // Either we have advanced to the next instruction in issue stage
      // Or we are in the same stage and issue counter is never ahead of the ring to ensure ring packets don't get overwritten by next sldu operand from VRF
      ring_ready_to_receive = (vinsn_queue_d.issue_pnt != vinsn_queue_d.ring_pnt) || 
                         ((issue_cnt_d < ( is_ring_reduction ? inter_cluser_issue_limit_d : ring_cnt_d)) && (vinsn_queue_d.issue_pnt == vinsn_queue_d.ring_pnt) && (state_d != SLIDE_IDLE));

      if  ((result_queue_cnt_d || issue_cnt_d==0) && ring_ready_to_receive) begin

        automatic elen_t scalar_op = vinsn_ring.scalar_op;
        automatic ara_op_e op = vinsn_ring.op;
        automatic vew_e sew = vinsn_ring.vtype.vsew;
        
        if (fifo_ring_valid_inp) begin

          ///// HANDLE RING PACKET /////

          automatic id_lane_t lane_id = sldu_ring_i.dst_lane;
          
          // We have a valid ring packet
          ring_data_prev_d[lane_id] = fifo_ring_inp;
          ring_data_prev_valid_d[lane_id] = 1'b1;

          // If nothing in the result queue, then this is the last set of data from ring received.
          // In this case we need to initialize vrf address and id. And increase result queue counter
          if (!result_queue_cnt_d) begin
            for (int l=0; l<NrLanes; l++) begin
              result_queue_d[result_queue_write_pnt_ring_q][l].addr = vaddr(vinsn_ring.vd, NrLanes) + vrf_pnt_q;
            end
            result_queue_cnt_d += 1;
          end
          result_queue_d[result_queue_write_pnt_ring_q][lane_id].id = vinsn_ring.id;
          
          // By default, acknowledge fifo that data has been accepted.
          fifo_ring_ready_out = 1'b1;
          
          // For the edge cluster, We need to have the current ring packet and the previous ring packet
          if (is_edge_cluster) begin
            
            ///// SLIDES FOR EDGE CLUSTERS /////

            if (ring_data_prev_valid_q[lane_id]) begin
              
              // Combine previous ring data and current ring data for the edge cluster
              automatic elen_t ring_data_prev = ring_data_prev_q[lane_id];
              result_queue_d[result_queue_write_pnt_ring_q][lane_id].wdata = merge_ring_data_slide_by_op(fifo_ring_inp, ring_data_prev, op, sew);
              
              n_ring_in_d = n_ring_in_q - 1;
              slide_result_valid = (n_ring_in_q == 1) ? 1'b1 : 1'b0;
            
            end else begin
            
              if (op==VSLIDEDOWN) begin
                // Write data from the current input packet
                result_queue_d[result_queue_write_pnt_ring_q][lane_id].wdata = merge_ring_data_slide_by_op(fifo_ring_inp, fifo_ring_inp, op, sew);

                // Slidedown needs to wait for 1 more ring packet, so don't commit to VRF just yet
                slide_result_valid = 1'b0;
              
              end else if (op==VSLIDEUP) begin
                // For the 1st packet, we can write the current ring input and the scalar value
                result_queue_d[result_queue_write_pnt_ring_q][lane_id].wdata = merge_ring_data_slide_by_op(fifo_ring_inp, splat_scalar_by_sew(scalar_op, sew), op, sew);
                
                // Update ring pointer
                // If all packets received, set result queue to valid
                n_ring_in_d = n_ring_in_q - 1;
                slide_result_valid = (n_ring_in_q == 1) ? 1'b1 : 1'b0;
              end

            end
          end else begin
            if (vinsn_ring.vfu inside {VFU_Alu, VFU_MFpu}) begin
              
              //// REDUCTIONS WITH RING ////

              // If this is the last transfer, Cluster-0 should receive data into Lane-0 from Lane-(NrLanes-1) of Cluster-(NrCluster-1)
              if (cluster_id_i==0 && ring_cnt_q==8*NrLanes)
                result_queue_d[result_queue_write_pnt_ring_q][0].wdata = fifo_ring_inp;
              else begin
                result_queue_d[result_queue_write_pnt_ring_q][lane_id].wdata = fifo_ring_inp;
                cluster_red_cnt_d = cluster_red_cnt_q - 1;
                `ifndef VERILATOR
                assert (cluster_red_cnt_q != 0) else $error("cluster_red_cnt_q should not be 0 when sending data on the ring for reduction");
                `endif
              end
              slide_result_valid = 1'b1;
            
            end else begin

              ///// SLIDES FOR NON-EDGE CLUSTERS /////

              // Write the ring input into the lane as it is
              result_queue_d[result_queue_write_pnt_ring_q][lane_id].wdata = fifo_ring_inp;
              
              // If it is also the last cluster for slidedown, add the scalar value
              if ((op == VSLIDEDOWN) && (ring_cnt_q > 0) && (ring_cnt_q <= 8*NrLanes) && (cluster_id_i == last_cluster_id)) begin
                result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata = write_scalar_to_result(result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata, last_elem_offset, scalar_op, sew);
              end

              n_ring_in_d = n_ring_in_q - 1;
              slide_result_valid = (n_ring_in_q == 1) ? 1'b1 : 1'b0;
            end
          end
        end else begin

          ///// NO RING PACKET ////

          // This is to handle cases where we have 1 more commit to be made to VRF, but we do not receive a packet on the ring
          if ((ring_cnt_q > 0) && (ring_cnt_q <= 8*NrLanes) && (issue_cnt_q ==0)) begin          
              
            automatic logic finish_last_result = 1'b0;
            
            // For the last data for slide1down use the scalar op
            case (op)
              VSLIDEDOWN: begin
                automatic elen_t ring_data_prev = ring_data_prev_q[NrLanes-1];
                if ((cluster_id_i == max_cluster_id) && |ring_data_prev_valid_q) begin
                  result_queue_d[result_queue_write_pnt_ring_q][NrLanes-1].wdata = merge_ring_data_slide_by_op(ring_data_prev, ring_data_prev, op, sew);
                  
                  // For slidedown, if we have received at least 1 ring packet, we can use the previous ring data and scalar op to generate the last data
                  if (cluster_id_i == last_cluster_id) begin
                    result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata = write_scalar_to_result(result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata, last_elem_offset, scalar_op, sew);
                  end
                  finish_last_result = 1'b1;

                  // If it is not the last cluster, and if we are doing a full word write, we will need to wait and receive one more ring packet
                  if ((cluster_id_i != last_cluster_id) && (ring_cnt_q==8*NrLanes)) begin
                    finish_last_result = 1'b0;
                  end
                end
                else if (cluster_id_i == last_cluster_id) begin 
                  if (|ring_data_prev_valid_q && (ring_cnt_q <= (NrLanes << sew))) begin
                    result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata = write_scalar_to_result(result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata, last_elem_offset, scalar_op, sew);
                    finish_last_result = 1'b1;
                  end else if (vinsn_ring.vl_cluster <= (cluster_id_i+1) * NrLanes) begin
                    // If we small number of elements, we don't receive a ring packet, but we can still generate the last data using the scalar operand
                    result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata = write_scalar_to_result(result_queue_d[result_queue_write_pnt_ring_q][last_lane_id].wdata, last_elem_offset, scalar_op, sew);
                    finish_last_result = 1'b1;
                  end
                end
              end
              VSLIDEUP:begin
                automatic elen_t ring_data_prev = ring_data_prev_q[0];
                if (cluster_id_i == 0) begin
                  // If receiving 1 less packet here
                  if (|ring_data_prev_valid_q && (ring_cnt_q <= (NrLanes << sew))) begin
                    result_queue_d[result_queue_write_pnt_ring_q][0].wdata = merge_ring_data_slide_by_op(ring_data_prev, ring_data_prev, op, sew);
                    finish_last_result = 1'b1;
                  end else if (vinsn_ring.vl_cluster <= total_lanes) begin 
                    result_queue_d[result_queue_write_pnt_ring_q][0].wdata = merge_ring_data_slide_by_op(scalar_op, splat_scalar_by_sew(scalar_op, sew), op, sew);
                    finish_last_result = 1'b1;
                  end
                end
              end
            endcase
            if (finish_last_result) begin
              n_ring_in_d = n_ring_in_q - 1;
              slide_result_valid = 1'b1;
              ring_data_prev_d = '0; 
              ring_data_prev_valid_d = '0;
            end
          end
        end
      end
    end else if (~vinsn_ring_valid) begin

      // If there is no instruction to handle, and we receive a packet, we just drop the packet
      // e.g. cluster 0 and vl <= (NrLanes * NrClusters)
      if (fifo_ring_valid_inp) begin
        fifo_ring_ready_out = 1'b1;  // Acknowledge fifo that data has been accepted.
      end
    end

    // Set the data to valid to be sent to the VRF
    // Update the write_pnt_q2
    if (slide_result_valid) begin
      result_queue_valid_d[result_queue_write_pnt_ring_q] = '1;

      if (!(vinsn_ring.vfu inside {VFU_Alu, VFU_MFpu})) begin
        result_queue_write_pnt_ring_d = result_queue_write_pnt_ring_q + 1;
        if (result_queue_write_pnt_ring_d == ResultQueueDepth)
          result_queue_write_pnt_ring_d = '0;
      end
      
      if (ring_cnt_q >= 8 *NrLanes) begin
        ring_cnt_d = ring_cnt_q - 8*NrLanes;
      end else begin
        ring_cnt_d = '0;
        `ifndef VERILATOR
        assert(ring_cnt_q != 0)
          else $error("ring_cnt_q should not be 0 here");
        `endif
      end

      n_ring_in_d = n_ring_in_init_d;

      // Update counters and pointers
      if (vinsn_ring_valid && ring_cnt_d == '0) begin
        ring_data_prev_d = '0; 
        ring_data_prev_valid_d = '0;
            
        vinsn_queue_d.ring_cnt -= 1;
        vinsn_queue_d.ring_pnt += 1;
        if (vinsn_queue_d.ring_pnt == VInsnQueueDepth) begin
          vinsn_queue_d.ring_pnt = '0; 
        end 
        
        // Update the ring counter for the next instruction
        if (vinsn_queue_d.ring_cnt != '0) begin
          automatic pe_req_t ring_vinsn = vinsn_queue_q.vinsn[vinsn_queue_d.ring_pnt];
          automatic elen_t stride = ring_vinsn.stride;
          automatic logic [$clog2(MAXVL/8):0] cluster_strides = stride >> ($clog2(NrLanes * 8) + num_clusters_i);
          ring_cnt_d = ring_vinsn.op inside {VSLIDEUP, VSLIDEDOWN}
                        ? ring_vinsn.vl << int'(ring_vinsn.vtype.vsew)
                        : (NrLanes * ($clog2(NrLanes))) << EW64;

          // Add packets for inter cluster reduction
          if (ring_vinsn.vfu inside {VFU_Alu, VFU_MFpu}) begin
            // Packets to be received for reduction from ring, cluster0 receives 1 additional packet for final result
            ring_cnt_d += (NrLanes * (cluster_reduction_rx_cnt_init(cluster_id_i) - 1 + (cluster_id_i==0 ? 1 : 0))) << EW64;
            
            // Handle edge case when not all clusters participate in reduction
            if (ring_vinsn.vl_cluster <= (cluster_id_i+1)*NrLanes) begin
              ring_cnt_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
            end
          end

          // Find the number of packets coming in. Maximum possible is NrLanes
          stride -= cluster_strides * (8 * total_lanes); 
          n_ring_in_d = stride >> ring_vinsn.vtype.vsew;
          n_ring_in_d = n_ring_in_d > NrLanes ? NrLanes : n_ring_in_d;
          n_ring_in_init_d = n_ring_in_d;
        end
      end
    end

    //////////////////////////////
    //  Accept new instruction  //
    //////////////////////////////

    if (!vinsn_queue_full && pe_req_valid_i && !vinsn_running_q[pe_req_i.id] &&
      (pe_req_i.vfu == VFU_SlideUnit || pe_req_i.op inside {[VREDSUM:VWREDSUM], [VFREDUSUM:VFWREDOSUM]})) begin
      vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt] = pe_req_i;
      vinsn_running_d[pe_req_i.id]                  = 1'b1;

      // Calculate the slide offset inside the vector register
      if (pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].stride = pe_req_i.stride << int'(pe_req_i.vtype.vsew);
      
      // Always move 64-bit packs of data from one lane to the other
      if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu})
        vinsn_queue_d.vinsn[vinsn_queue_q.accept_pnt].vtype.vsew = EW64;

      if (vinsn_queue_d.commit_cnt == '0) begin
        automatic elen_t stride = vinsn_queue_d.vinsn[vinsn_queue_q.commit_pnt].stride;
        automatic logic [$clog2(MAXVL/8):0] cluster_strides = stride >> ($clog2(NrLanes * 8) + num_clusters_i);
        commit_cnt_d = pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN}
                     ? pe_req_i.vl << int'(pe_req_i.vtype.vsew)
                     : (NrLanes * ($clog2(NrLanes))) << EW64;

        // Add packets for inter cluster reduction
        if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu}) begin
          // results to be committed after receiving from ring, cluster0 commits 1 additional packet for final result
          commit_cnt_d += (NrLanes * (cluster_reduction_rx_cnt_init(cluster_id_i) - 1 + (cluster_id_i==0 ? 1 : 0))) << EW64;
          if (pe_req_i.vl_cluster <= (cluster_id_i+1)*NrLanes) begin
            commit_cnt_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
          end
        end

        // Trim vector elements which are not written by the slide unit
        if (pe_req_i.op == VSLIDEUP) 
          commit_cnt_d -= cluster_strides * 8 * NrLanes;
      end

      // Set expected ring counter packets for the instruction
      if (vinsn_queue_d.ring_cnt == '0) begin
        automatic elen_t stride = vinsn_queue_d.vinsn[vinsn_queue_q.ring_pnt].stride;
        automatic logic [$clog2(MAXVL/8):0] cluster_strides = stride >> ($clog2(NrLanes * 8) + num_clusters_i);
        ring_cnt_d = pe_req_i.op inside {VSLIDEUP, VSLIDEDOWN}
                     ? pe_req_i.vl << int'(pe_req_i.vtype.vsew)
                     : (NrLanes * ($clog2(NrLanes))) << EW64;

        // Add packets for inter cluster reduction
        if (pe_req_i.vfu inside {VFU_Alu, VFU_MFpu}) begin
          // Packets to be received for reduction from ring, cluster0 receives 1 additional packet for final result
          ring_cnt_d += (NrLanes * (cluster_reduction_rx_cnt_init(cluster_id_i) - 1 + (cluster_id_i==0 ? 1 : 0))) << EW64;
          if (pe_req_i.vl_cluster <= (cluster_id_i+1)*NrLanes) begin
            ring_cnt_d += (NrLanes * ext_reduction_rx_cnt_init(cluster_id_i)) << EW64;
          end
        end

        stride -= cluster_strides * (8 * total_lanes);

        // Find the number of packets expected on the ring interface
        n_ring_in_d = stride >> vinsn_queue_d.vinsn[vinsn_queue_q.ring_pnt].vtype.vsew;
        n_ring_in_d = n_ring_in_d > NrLanes ? NrLanes : n_ring_in_d;
        n_ring_in_init_d = n_ring_in_d;
      end

      // Bump pointers and counters of the vector instruction queue
      if (vinsn_queue_q.accept_pnt == VInsnQueueDepth-1) begin
        vinsn_queue_d.accept_pnt = '0;
      end else begin
        vinsn_queue_d.accept_pnt += 1;
      end
      vinsn_queue_d.issue_cnt += 1;
      vinsn_queue_d.ring_cnt += 1;
      vinsn_queue_d.commit_cnt += 1;
    end
  end: p_sldu

  //// Helper functions for handling ring data and slide operations ////

  // Function to merge ring data based on slide operation direction (VSLIDEDOWN vs VSLIDEUP)
  function automatic elen_t merge_ring_data_slide_by_op(elen_t new_data, elen_t prev_data, ara_op_e op, rvv_pkg::vew_e sew);
    if (op == VSLIDEDOWN) begin
      merge_ring_data_slide_by_op = merge_ring_data_slidedown(new_data, prev_data, sew);
    end else begin
      merge_ring_data_slide_by_op = merge_ring_data_slideup(new_data, prev_data, sew);
    end
  endfunction: merge_ring_data_slide_by_op

  // Function to merge new ring data with previous ring data for VSLIDEUP based on element width (vsew)
  function automatic elen_t merge_ring_data_slideup(elen_t new_data, elen_t prev_data, rvv_pkg::vew_e sew);
    case (sew)
      EW64: merge_ring_data_slideup = {prev_data};
      EW32: merge_ring_data_slideup = {new_data[31:0], prev_data[63:32]};
      EW16: merge_ring_data_slideup = {new_data[31:0], new_data[47:32], prev_data[63:48]};
      EW8:  merge_ring_data_slideup = {new_data[31:0], new_data[47:32], new_data[55:48], prev_data[63:56]};
      default: merge_ring_data_slideup = new_data;
    endcase
  endfunction: merge_ring_data_slideup

  // Function to merge new ring data with previous ring data based on element width (vsew)
  function automatic elen_t merge_ring_data_slidedown(elen_t new_data, elen_t prev_data, rvv_pkg::vew_e sew);
    case (sew)
      EW64: merge_ring_data_slidedown = {new_data};
      EW32: merge_ring_data_slidedown = {new_data[31:0], prev_data[63:32]};
      EW16: merge_ring_data_slidedown = {new_data[15:0], prev_data[31:16], prev_data[63:48], prev_data[47:32]};
      EW8:  merge_ring_data_slidedown = {new_data[7:0], prev_data[15:8], prev_data[31:24], prev_data[23:16], prev_data[63:32]};
      default: merge_ring_data_slidedown = new_data;
    endcase
  endfunction: merge_ring_data_slidedown

  // Function to extract and write scalar operand bits based on element width (vsew) at specified offset
  function automatic elen_t write_scalar_to_result(input elen_t wdata, input int offset, input elen_t scalar_op, input rvv_pkg::vew_e sew);
    case (sew)
      EW64: begin write_scalar_to_result = wdata; write_scalar_to_result[offset*8 +: 64] = scalar_op; end
      EW32: begin write_scalar_to_result = wdata; write_scalar_to_result[offset*8 +: 32] = scalar_op[31:0]; end
      EW16: begin write_scalar_to_result = wdata; write_scalar_to_result[offset*8 +: 16] = scalar_op[15:0]; end
      EW8:  begin write_scalar_to_result = wdata; write_scalar_to_result[offset*8 +: 8] = scalar_op[7:0]; end
      default: begin write_scalar_to_result = wdata; write_scalar_to_result[offset*8 +: 64] = scalar_op; end
    endcase
  endfunction: write_scalar_to_result

  // Function to write scalar operand combined with ring data to result queue for VSLIDEUP
  // Function to splat scalar operand based on element width
  function automatic elen_t splat_scalar_by_sew(elen_t scalar_op, rvv_pkg::vew_e sew);
    case (sew)
      EW64: splat_scalar_by_sew = scalar_op;
      EW32: splat_scalar_by_sew = {2{scalar_op[31:0]}};
      EW16: splat_scalar_by_sew = {4{scalar_op[15:0]}};
      EW8:  splat_scalar_by_sew = {8{scalar_op[7:0]}};
      default: splat_scalar_by_sew = scalar_op;
    endcase
  endfunction: splat_scalar_by_sew

  // Function to track how many packets should be pushed into ring by each cluster for inter cluster reduction.
  // E.g. for 4 clusters
  // Step-1: In the first step 0->1 1->2 2->3 and 3->1 - all clusters send a single packet to all other clusters. So atleast 1 present.
  // Step-2: 2nd step 1->3, put 2 in bypass mode (i.e. even numbered clusters other than 0). 
  //         Since 0 is not in bypass 3->0, 0->1 also happens. But the FPU in last lane of cluster-0 and 1, receiving data from SLDU ignores it.
  //         This is tracked by the reduction_rx_cnt_d in FPU. 
  // Last-Step: Finally 3 sends result to 1.
  function automatic cluster_reduction_rx_cnt_t cluster_reduction_rx_cnt_init(id_cluster_t clusterId);
    case (clusterId)
      0:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      1:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      2:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      3:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      4:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      5:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      6:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      7:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(4);
      8:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      9:  cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      10: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      11: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      12: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      13: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      14: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      15: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(5);
      16: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      17: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      18: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      19: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      20: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      21: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      22: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      23: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(4);
      24: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      25: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      26: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      27: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      28: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      29: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      30: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      31: cluster_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(6);
    endcase
  endfunction: cluster_reduction_rx_cnt_init

  // If the vector lengths are smaller than the total number of lanes, some clusters receive additional packets
  function automatic cluster_reduction_rx_cnt_t ext_reduction_rx_cnt_init(id_cluster_t clusterId);
    // The even lanes do not receive intermediate results. Only Lane 0 will receive the final result, but this is not checked here.
    case (clusterId)
      0:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(0);
      1:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(0);
      2:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      3:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(0);
      4:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      5:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      6:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      7:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(0);
      8:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      9:  ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      10: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      11: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(1);
      12: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      13: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(2);
      14: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(3);
      15: ext_reduction_rx_cnt_init = cluster_reduction_rx_cnt_t'(0);
    endcase
  endfunction: ext_reduction_rx_cnt_init

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vinsn_running_q       <= '0;
      issue_cnt_q           <= '0;
      commit_cnt_q          <= '0;
      in_pnt_q              <= '0;
      out_pnt_q             <= '0;
      vrf_pnt_q             <= '0;
      output_limit_q        <= '0;
      state_q               <= SLIDE_IDLE;
      pe_resp_o             <= '0;
      result_final_gnt_q    <= '0;
      red_stride_cnt_q      <= 1;
      np2_loop_mux_sel_q    <= NP2_EXT_SEL;
      slide_np2_buf_valid_q <= 1'b0;
    end else begin
      vinsn_running_q       <= vinsn_running_d;
      issue_cnt_q           <= issue_cnt_d;
      commit_cnt_q          <= commit_cnt_d;
      in_pnt_q              <= in_pnt_d;
      out_pnt_q             <= out_pnt_d;
      vrf_pnt_q             <= vrf_pnt_d;
      output_limit_q        <= output_limit_d;
      state_q               <= state_d;
      pe_resp_o             <= pe_resp;
      result_final_gnt_q    <= result_final_gnt_d;
      red_stride_cnt_q      <= red_stride_cnt_d;
      np2_loop_mux_sel_q    <= np2_loop_mux_sel_d;
      slide_np2_buf_valid_q <= slide_np2_buf_valid_d;      
    end
  end

endmodule: sldu
