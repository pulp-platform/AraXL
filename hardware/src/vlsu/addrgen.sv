// Copyright 2021-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// This unit generates transactions on the AR/AW buses, upon receiving vector
// memory operations.

module addrgen import ara_pkg::*; import rvv_pkg::*; #(
    parameter int  unsigned NrLanes      = 0,
    // AXI Interface parameters
    parameter int  unsigned AxiDataWidth = 0,
    parameter int  unsigned AxiAddrWidth = 0,
    parameter type          axi_ar_t     = logic,
    parameter type          axi_aw_t     = logic,
    // Dependant parameters. DO NOT CHANGE!
    parameter type          axi_addr_t   = logic [AxiAddrWidth-1:0]
  ) (
    input  logic                           clk_i,
    input  logic                           rst_ni,
    // Memory interface
    output axi_ar_t                        axi_ar_o,
    output vew_e                           vew_ar_o, 
    output logic                           axi_ar_valid_o,
    input  logic                           axi_ar_ready_i,
    output axi_aw_t                        axi_aw_o,
    output vew_e                           vew_aw_o,
    output logic                           axi_aw_valid_o,
    input  logic                           axi_aw_ready_i,
    // Interace with the dispatcher
    input  logic                           core_st_pending_i,
    // Interface with the main sequencer
    input  pe_req_t                        pe_req_i,
    input  logic                           pe_req_valid_i,
    input  logic     [NrVInsn-1:0]         pe_vinsn_running_i,
    output logic                           addrgen_error_o,
    output logic                           addrgen_ack_o,
    output vlen_t                          addrgen_error_vl_o,
    // Interface with the load/store units
    output addrgen_axi_req_t               axi_addrgen_req_o,
    output logic                           axi_addrgen_req_valid_o,
    input  logic                           ldu_axi_addrgen_req_ready_i,
    input  logic                           stu_axi_addrgen_req_ready_i,
    // Interface with the lanes (for scatter/gather operations)
    input  elen_t            [NrLanes-1:0] addrgen_operand_i,
    input  target_fu_e       [NrLanes-1:0] addrgen_operand_target_fu_i,
    input  logic             [NrLanes-1:0] addrgen_operand_valid_i,
    output logic                           addrgen_operand_ready_o
  );

  localparam unsigned DataWidth = $bits(elen_t);
  localparam unsigned DataWidthB = DataWidth / 8;

  localparam unsigned Log2NrLanes = $clog2(NrLanes);
  localparam unsigned Log2LaneWordWidthB = $clog2(DataWidthB/1);
  localparam unsigned Log2LaneWordWidthH = $clog2(DataWidthB/2);
  localparam unsigned Log2LaneWordWidthS = $clog2(DataWidthB/4);
  localparam unsigned Log2LaneWordWidthD = $clog2(DataWidthB/8);
  localparam unsigned Log2VRFWordWidthB = Log2NrLanes + Log2LaneWordWidthB;
  localparam unsigned Log2VRFWordWidthH = Log2NrLanes + Log2LaneWordWidthH;
  localparam unsigned Log2VRFWordWidthS = Log2NrLanes + Log2LaneWordWidthS;
  localparam unsigned Log2VRFWordWidthD = Log2NrLanes + Log2LaneWordWidthD;

  import cf_math_pkg::idx_width;
  import axi_pkg::aligned_addr;
  import axi_pkg::BURST_INCR;
  import axi_pkg::CACHE_MODIFIABLE;

  // // Check if the address is aligned to a particular width
  // function automatic logic is_addr_error(axi_addr_t addr, vew_e vew);
  //   is_addr_error = |(addr & (elen_t'(1 << vew) - 1));
  // endfunction // is_addr_error

  // Check if the address is aligned to a particular width
  // Max element width: 8 bytes
  function automatic logic is_addr_error(axi_addr_t addr, logic [1:0] vew);
    // log2(MAX_ELEMENT_WIDTH_BYTE)
    localparam LOG2_MAX_SEW_BYTE = 3;
    typedef logic [LOG2_MAX_SEW_BYTE:0] max_sew_byte_t;

    is_addr_error = |(max_sew_byte_t'(addr[LOG2_MAX_SEW_BYTE-1:0]) & (max_sew_byte_t'(1 << vew) - 1));
  endfunction // is_addr_error

  ////////////////////
  //  PE Req Queue  //
  ////////////////////

  // The address generation process interacts with another process, that
  // generates the AXI requests. They interact through the following signals.
  typedef struct packed {
    axi_addr_t addr;
    vlen_t len;
    elen_t stride;
    logic [1:0] vew;
    logic is_load;
    logic is_burst; // Unit-strided instructions can be converted into AXI INCR bursts
    vlen_t vstart;
    vlen_t vl_ldst;  // Meaning that the cluster should pad zero for write data
  } addrgen_req_t;
  addrgen_req_t addrgen_req;
  logic         addrgen_req_valid;
  logic         addrgen_req_ready;

  // Pipeline the PE requests
  pe_req_t pe_req_d, pe_req_q;

  /////////////////////
  //  Address Queue  //
  /////////////////////

  // Address queue for the vector load/store units
  addrgen_axi_req_t axi_addrgen_queue;
  logic             axi_addrgen_queue_push;
  logic             axi_addrgen_queue_full;
  logic             axi_addrgen_queue_empty;
  logic             axi_addrgen_queue_pop;

  assign axi_addrgen_queue_pop = ldu_axi_addrgen_req_ready_i | stu_axi_addrgen_req_ready_i;

  fifo_v3 #(
    .DEPTH(VaddrgenInsnQueueDepth),
    .dtype(addrgen_axi_req_t     )
  ) i_addrgen_req_queue (
    .clk_i     (clk_i                                                    ),
    .rst_ni    (rst_ni                                                   ),
    .flush_i   (1'b0                                                     ),
    .testmode_i(1'b0                                                     ),
    .data_i    (axi_addrgen_queue                                        ),
    .push_i    (axi_addrgen_queue_push                                   ),
    .full_o    (axi_addrgen_queue_full                                   ),
    .data_o    (axi_addrgen_req_o                                        ),
    .pop_i     (axi_addrgen_queue_pop                                    ),
    .empty_o   (axi_addrgen_queue_empty                                  ),
    .usage_o   (/* Unused */                                             )
  );
  assign axi_addrgen_req_valid_o = !axi_addrgen_queue_empty;

  //////////////////////////
  //  Indexed Memory Ops  //
  //////////////////////////

  // Support for indexed memory operations (scatter/gather)
  logic [$bits(elen_t)*NrLanes-1:0] shuffled_word;
  logic [$bits(elen_t)*NrLanes-1:0] deshuffled_word;
  elen_t                            reduced_word;
  axi_addr_t                        idx_final_addr_d, idx_final_addr_q;
  elen_t                            idx_addr;
  logic                             idx_op_error_d, idx_op_error_q;
  vlen_t                            addrgen_error_vl_d;

  // Pointer to point to the correct
  logic [$clog2(NrLanes)-1:0] word_lane_ptr_d, word_lane_ptr_q;
  logic [$clog2($bits(elen_t)/8)-1:0] elm_ptr_d, elm_ptr_q;
  logic [$clog2($bits(elen_t)/8)-1:0] last_elm_subw_d, last_elm_subw_q;
  vlen_t                              idx_op_cnt_d, idx_op_cnt_q;

  // Spill reg signals
  logic      idx_addr_valid_d, idx_addr_valid_q;
  logic      idx_addr_ready_d, idx_addr_ready_q;

  // Break the path from the VRF to the AXI request
  spill_register #(
    .T(elen_t)
  ) i_addrgen_idx_op_spill_reg (
    .clk_i  (clk_i           ),
    .rst_ni (rst_ni          ),
    .valid_i(idx_addr_valid_d),
    .ready_o(idx_addr_ready_q),
    .data_i (idx_final_addr_d),
    .valid_o(idx_addr_valid_q),
    .ready_i(idx_addr_ready_d),
    .data_o (idx_final_addr_q)
  );

  //////////////////////////
  //  Address generation  //
  //////////////////////////

  vlen_t len_temp;
  axi_addr_t next_addr_strided_temp;

  // Running vector instructions
  logic [NrVInsn-1:0] vinsn_running_d, vinsn_running_q;

  // The Address Generator can be in one of the following three states.
  // IDLE: Waiting for a vector load/store instruction.
  // ADDRGEN: Generates a series of AXI requests from a vector instruction.
  // ADDRGEN_IDX_OP: Generates a series of AXI requests from a
  //    vector instruction, but reading a vector of offsets from Ara's lanes.
  //    This is used for scatter and gather operations.
  enum logic [1:0] {
    IDLE,
    ADDRGEN,
    ADDRGEN_IDX_OP,
    ADDRGEN_IDX_OP_END
  } state_q, state_d;

  axi_addr_t lookahead_addr_e_d, lookahead_addr_e_q;
  axi_addr_t lookahead_addr_se_d, lookahead_addr_se_q;
  vlen_t lookahead_len_d, lookahead_len_q, lookahead_len_ldst_d, lookahead_len_ldst_q;

  logic [VLEN-1:0] vaddr_start;

  always_comb begin: addr_generation
    // Maintain state
    state_d  = state_q;
    pe_req_d = pe_req_q;
    lookahead_addr_e_d  = lookahead_addr_e_q;
    lookahead_addr_se_d = lookahead_addr_se_q;
    lookahead_len_d     = lookahead_len_q;
    lookahead_len_ldst_d = lookahead_len_ldst_q;

    // Running vector instructions
    vinsn_running_d = vinsn_running_q & pe_vinsn_running_i;

    // No request, by default
    addrgen_req       = '0;
    addrgen_req_valid = 1'b0;

    // Nothing to acknowledge
    addrgen_ack_o           = 1'b0;
    addrgen_error_o         = 1'b0;

    // No valid words for the spill register
    idx_addr_valid_d        = 1'b0;
    addrgen_operand_ready_o = 1'b0;
    reduced_word            = '0;
    elm_ptr_d               = elm_ptr_q;
    idx_op_cnt_d            = idx_op_cnt_q;
    word_lane_ptr_d         = word_lane_ptr_q;
    idx_final_addr_d        = idx_final_addr_q;
    last_elm_subw_d         = last_elm_subw_q;

    // Support for indexed operations
    shuffled_word = addrgen_operand_i;
    // Deshuffle the whole NrLanes * 8 Byte word
    for (int unsigned b = 0; b < 8*NrLanes; b++) begin
      automatic shortint unsigned b_shuffled = shuffle_index(b, NrLanes, pe_req_q.eew_vs2);
      deshuffled_word[8*b +: 8] = shuffled_word[8*b_shuffled +: 8];
    end

    // Extract only 1/NrLanes of the word
    for (int unsigned lane = 0; lane < NrLanes; lane++)
      if (lane == word_lane_ptr_q)
        reduced_word = deshuffled_word[word_lane_ptr_q*$bits(elen_t) +: $bits(elen_t)];
    idx_addr = reduced_word;

    case (state_q)
      IDLE: begin
        // Received a new request
        if (pe_req_valid_i &&
            (is_load(pe_req_i.op) || is_store(pe_req_i.op)) && !vinsn_running_q[pe_req_i.id]) begin
          // Mark the instruction as running in this unit
          vinsn_running_d[pe_req_i.id] = 1'b1;

          // Store the PE request
          pe_req_d = pe_req_i;

          // Pre-calculate expensive additions / multiplications
          // pe_req_i shouldn't be that critical at this point
          lookahead_addr_e_d  = pe_req_i.scalar_op + (pe_req_i.vstart << unsigned'(pe_req_i.vtype.vsew));
          lookahead_addr_se_d = pe_req_i.scalar_op + (pe_req_i.vstart * pe_req_i.stride);
          lookahead_len_d     = (pe_req_i.vl - pe_req_i.vstart) << unsigned'(pe_req_i.vtype.vsew[1:0]);
          // For synchronization among clusters, vl for vse is always multiplier of NrLanes*NrClusters, but actual write vl may 
          // be smaller. Here we tell vstu module how many data should not be written (i.e. set write enable to zero)
          lookahead_len_ldst_d = (pe_req_i.vl - pe_req_i.vl_ldst) << unsigned'(pe_req_i.vtype.vsew[1:0]);

          case (pe_req_i.op)
            VLXE, VSXE: begin
              state_d = ADDRGEN_IDX_OP;

              // Load element pointers
              case (pe_req_i.eew_vs2)
                EW8: begin
                  last_elm_subw_d = 7;
                  word_lane_ptr_d = pe_req_i.vstart[Log2VRFWordWidthB-1:Log2LaneWordWidthB];
                  elm_ptr_d       = pe_req_i.vstart[Log2LaneWordWidthB-1:0];
                end
                EW16: begin
                  last_elm_subw_d = 3;
                  word_lane_ptr_d = pe_req_i.vstart[Log2VRFWordWidthH-1:Log2LaneWordWidthH];
                  elm_ptr_d       = pe_req_i.vstart[Log2LaneWordWidthH-1:0];
                end
                EW32: begin
                  last_elm_subw_d = 1;
                  word_lane_ptr_d = pe_req_i.vstart[Log2VRFWordWidthS-1:Log2LaneWordWidthS];
                  elm_ptr_d       = pe_req_i.vstart[Log2LaneWordWidthS-1:0];
                end
                default: begin // EW64
                  last_elm_subw_d = 0;
                  word_lane_ptr_d = pe_req_i.vstart[Log2VRFWordWidthD-1:0];
                  elm_ptr_d       = 0;
                end
              endcase

              // Load element counter
              idx_op_cnt_d = pe_req_i.vl - pe_req_i.vstart;
            end
            default: state_d = ADDRGEN;
          endcase
        end
      end
      // IDLE: begin
      //   // Received a new request
      //   if (pe_req_valid_i &&
      //       (is_load(pe_req_i.op) || is_store(pe_req_i.op)) && !vinsn_running_q[pe_req_i.id]) begin
      //     // Mark the instruction as running in this unit
      //     vinsn_running_d[pe_req_i.id] = 1'b1;

      //     // Store the PE request
      //     pe_req_d = pe_req_i;

      //     case (pe_req_i.op)
      //       VLXE, VSXE: begin
      //         state_d = ADDRGEN_IDX_OP;

      //         // Load element pointers
      //         case (pe_req_i.eew_vs2)
      //           EW8:  last_elm_subw_d = 7;
      //           EW16: last_elm_subw_d = 3;
      //           EW32: last_elm_subw_d = 1;
      //           EW64: last_elm_subw_d = 0;
      //           default:
      //             last_elm_subw_d = 0;
      //         endcase

      //         // Load element counter
      //         idx_op_cnt_d = pe_req_i.vl;
      //       end
      //       default: state_d = ADDRGEN;
      //     endcase
      //   end
      // end
      ADDRGEN: begin
        // NOTE: indexed are not covered here
        case (pe_req_q.op)
          // Unit-stride: address = base + (vstart in elements)
          VLE,  VSE : vaddr_start = lookahead_addr_e_q;
          // Strided: address = base + (vstart * stride)
          VLSE, VSSE: vaddr_start = lookahead_addr_se_q;
          // Indexed: let the next stage take care of vstart
          VLXE, VSXE: vaddr_start = pe_req_q.scalar_op;
          default   : vaddr_start = '0;
        endcase // pe_req_q.op

        // Start the computation already
        addrgen_req = '{
          addr    : vaddr_start,
          len     : lookahead_len_q,
          stride  : pe_req_q.stride,
          vew     : pe_req_q.vtype.vsew[1:0],
          is_load : is_load(pe_req_q.op),
          // Unit-strided loads/stores trigger incremental AXI bursts.
          is_burst: (pe_req_q.op inside {VLE, VSE}),
          vstart  : pe_req_q.vstart,
          vl_ldst : lookahead_len_ldst_q
        };

        // Ara does not support misaligned AXI requests
        if (is_addr_error(pe_req_q.scalar_op, pe_req_q.vtype.vsew[1:0])) begin
          state_d         = IDLE;
          addrgen_ack_o   = 1'b1;
          addrgen_error_o = 1'b1;
        end
        else begin : address_valid
          addrgen_req_valid = 1'b1;

          if (addrgen_req_ready) begin : finished
            addrgen_req_valid = '0;
            addrgen_ack_o     = 1'b1;
            state_d           = IDLE;
          end : finished
        end : address_valid
      end
      // ADDRGEN: begin
      //   // Ara does not support misaligned AXI requests
      //   if (is_addr_error(pe_req_q.scalar_op, pe_req_q.vtype.vsew)) begin
      //     state_d         = IDLE;
      //     addrgen_ack_o   = 1'b1;
      //     addrgen_error_o = 1'b1;
      //   end else begin
      //     addrgen_req = '{
      //       addr    : pe_req_q.scalar_op,
      //       len     : pe_req_q.vl,
      //       stride  : pe_req_q.stride,
      //       vew     : pe_req_q.vtype.vsew,
      //       is_load : is_load(pe_req_q.op),
      //       // Unit-strided loads/stores trigger incremental AXI bursts.
      //       is_burst: (pe_req_q.op inside {VLE, VSE})
      //     };
      //     addrgen_req_valid = 1'b1;

      //     if (addrgen_req_ready) begin
      //       addrgen_req_valid = '0;
      //       addrgen_ack_o     = 1'b1;
      //       state_d           = IDLE;
      //     end
      //   end
      // end
      ADDRGEN_IDX_OP: begin
        // NOTE: vstart is not supported for indexed operations
        //       the logic shuld be introduced:
        //       1. in the addrgen_operand_i operand read
        //       2. in idx_addr computation
        automatic logic [NrLanes-1:0] addrgen_operand_valid;

        // Stall the interface until the operation is over to catch possible exceptions

        // Every address can generate an exception
        addrgen_req = '{
          addr    : pe_req_q.scalar_op,
          len     : lookahead_len_q,
          stride  : pe_req_q.stride,
          vew     : pe_req_q.vtype.vsew[1:0],
          is_load : is_load(pe_req_q.op),
          // Unit-strided loads/stores trigger incremental AXI bursts.
          is_burst: 1'b0,
          vstart  : pe_req_q.vstart,
          vl_ldst : lookahead_len_ldst_q
        };
        addrgen_req_valid = 1'b1;

        // Adjust valid signals to the next block "operands_ready"
        addrgen_operand_valid = addrgen_operand_valid_i;
        for (int unsigned lane = 0; lane < NrLanes; lane++) begin : adjust_operand_valid
          // - We are left with less byte than the maximim to issue,
          //    this means that at least one lane is not going to push us any operand anymore
          // - For the lanes which index % NrLanes != 0
          if (((idx_op_cnt_q << pe_req_q.vtype.vsew) < (NrLanes * DataWidthB))
               && (lane < pe_req_q.vstart[idx_width(NrLanes)-1:0])) begin : vstart_lane_adjust
            addrgen_operand_valid[lane] |= 1'b1;
          end : vstart_lane_adjust
        end : adjust_operand_valid
        // TODO: apply the same vstart logic also to mask_valid_i

        // Handle handshake and data between VRF and spill register
        // We accept all the incoming data, without any checks
        // since Ara stalls on an indexed memory operation
        if (&addrgen_operand_valid) begin

          // Valid data for the spill register
          idx_addr_valid_d = 1'b1;

          // Select the correct element, and zero extend it depending on vsew
          case (pe_req_q.eew_vs2)
            EW8: begin
              for (int unsigned b = 0; b < 8; b++)
                if (b == elm_ptr_q)
                  idx_addr = reduced_word[b*8 +: 8];
            end
            EW16: begin
              for (int unsigned h = 0; h < 4; h++)
                if (h == elm_ptr_q)
                  idx_addr = reduced_word[h*16 +: 16];
            end
            EW32: begin
              for (int unsigned w = 0; w < 2; w++)
                if (w == elm_ptr_q)
                  idx_addr = reduced_word[w*32 +: 32];
            end
            EW64: begin
              for (int unsigned d = 0; d < 1; d++)
                if (d == elm_ptr_q)
                  idx_addr = reduced_word[d*64 +: 64];
            end
            default: begin
              for (int unsigned b = 0; b < 8; b++)
                if (b == elm_ptr_q)
                  idx_addr = reduced_word[b*8 +: 8];
            end
          endcase

          // Compose the address
          idx_final_addr_d = pe_req_q.scalar_op + idx_addr;

          // When the data is accepted
          if (idx_addr_ready_q) begin
            // Consumed one element
            idx_op_cnt_d = idx_op_cnt_q - 1;
            // Have we finished a full NrLanes*64b word?
            if (elm_ptr_q == last_elm_subw_q) begin
              // Bump lane pointer
              elm_ptr_d       = '0;
              word_lane_ptr_d += 1;
              if (word_lane_ptr_q == NrLanes - 1) begin
                // Ready for the next full word
                addrgen_operand_ready_o = 1'b1;
              end
            end else begin
              // Bump element pointer
              elm_ptr_d += 1;
            end
          end

          if (idx_op_cnt_d == '0) begin
            // Give a ready to the lanes if this was not done before
            addrgen_operand_ready_o = 1'b1;
          end
        end

        if (idx_op_error_d || addrgen_req_ready) begin
          state_d = ADDRGEN_IDX_OP_END;
        end
      end
      // ADDRGEN_IDX_OP: begin
      //   // Stall the interface until the operation is over to catch possible exceptions

      //   // Every address can generate an exception
      //   addrgen_req = '{
      //     addr    : pe_req_q.scalar_op,
      //     len     : pe_req_q.vl,
      //     stride  : pe_req_q.stride,
      //     vew     : pe_req_q.vtype.vsew,
      //     is_load : is_load(pe_req_q.op),
      //     // Unit-strided loads/stores trigger incremental AXI bursts.
      //     is_burst: 1'b0
      //   };
      //   addrgen_req_valid = 1'b1;

      //   // Handle handshake and data between VRF and spill register
      //   // We accept all the incoming data, without any checks
      //   // since Ara stalls on an indexed memory operation
      //   if (&addrgen_operand_valid_i & addrgen_operand_target_fu_i[0] == MFPU_ADDRGEN) begin

      //     // Valid data for the spill register
      //     idx_addr_valid_d = 1'b1;

      //     // Select the correct element, and zero extend it depending on vsew
      //     case (pe_req_q.eew_vs2)
      //       EW8: begin
      //         for (int unsigned b = 0; b < 8; b++)
      //           if (b == elm_ptr_q)
      //             idx_addr = reduced_word[b*8 +: 8];
      //       end
      //       EW16: begin
      //         for (int unsigned h = 0; h < 4; h++)
      //           if (h == elm_ptr_q)
      //             idx_addr = reduced_word[h*16 +: 16];
      //       end
      //       EW32: begin
      //         for (int unsigned w = 0; w < 2; w++)
      //           if (w == elm_ptr_q)
      //             idx_addr = reduced_word[w*32 +: 32];
      //       end
      //       EW64: begin
      //         for (int unsigned d = 0; d < 1; d++)
      //           if (d == elm_ptr_q)
      //             idx_addr = reduced_word[d*64 +: 64];
      //       end
      //       default: begin
      //         for (int unsigned b = 0; b < 8; b++)
      //           if (b == elm_ptr_q)
      //             idx_addr = reduced_word[b*8 +: 8];
      //       end
      //     endcase

      //     // Compose the address
      //     idx_final_addr_d = pe_req_q.scalar_op + idx_addr;

      //     // When the data is accepted
      //     if (idx_addr_ready_q) begin
      //       // Consumed one element
      //       idx_op_cnt_d = idx_op_cnt_q - 1;
      //       // Have we finished a full NrLanes*64b word?
      //       if (elm_ptr_q == last_elm_subw_q) begin
      //         // Bump lane pointer
      //         elm_ptr_d       = '0;
      //         word_lane_ptr_d += 1;
      //         if (word_lane_ptr_q == NrLanes - 1) begin
      //           // Ready for the next full word
      //           addrgen_operand_ready_o = 1'b1;
      //         end
      //       end else begin
      //         // Bump element pointer
      //         elm_ptr_d += 1;
      //       end
      //     end

      //     if (idx_op_cnt_d == '0) begin
      //       // Give a ready to the lanes if this was not done before
      //       addrgen_operand_ready_o = 1'b1;
      //     end
      //   end

      //   if (idx_op_error_d || addrgen_req_ready) begin
      //     state_d = ADDRGEN_IDX_OP_END;
      //   end
      // end
      // This state exists not to create combinatorial paths on the interface
      ADDRGEN_IDX_OP_END : begin
        // Acknowledge the indexed memory operation
        addrgen_ack_o     = 1'b1;
        addrgen_req_valid = '0;
        state_d           = IDLE;
        // Reset pointers
        elm_ptr_d       = '0;
        word_lane_ptr_d = '0;
        // Raise an error if necessary
        if (idx_op_error_q) begin
          // In this case, we always get EEW-misaligned exceptions
          addrgen_error_o = 1'b1;
        end
      end
      // ADDRGEN_IDX_OP_END : begin
      //   // Acknowledge the indexed memory operation
      //   addrgen_ack_o     = 1'b1;
      //   addrgen_req_valid = '0;
      //   state_d           = IDLE;
      //   // Reset pointers
      //   elm_ptr_d       = '0;
      //   word_lane_ptr_d = '0;
      //   // Raise an error if necessary
      //   if (idx_op_error_q) begin
      //     addrgen_error_o = 1'b1;
      //   end
      // end
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q            <= IDLE;
      pe_req_q           <= '0;
      vinsn_running_q    <= '0;
      word_lane_ptr_q    <= '0;
      elm_ptr_q          <= '0;
      idx_op_cnt_q       <= '0;
      last_elm_subw_q    <= '0;
      idx_op_error_q     <= '0;
      addrgen_error_vl_o <= '0;
      lookahead_addr_e_q         <= '0;
      lookahead_addr_se_q        <= '0;
      lookahead_len_q            <= '0;
      lookahead_len_ldst_q       <= '0;
    end else begin
      state_q            <= state_d;
      pe_req_q           <= pe_req_d;
      vinsn_running_q    <= vinsn_running_d;
      word_lane_ptr_q    <= word_lane_ptr_d;
      elm_ptr_q          <= elm_ptr_d;
      idx_op_cnt_q       <= idx_op_cnt_d;
      last_elm_subw_q    <= last_elm_subw_d;
      idx_op_error_q     <= idx_op_error_d;
      addrgen_error_vl_o <= addrgen_error_vl_d;
      lookahead_addr_e_q         <= lookahead_addr_e_d;
      lookahead_addr_se_q        <= lookahead_addr_se_d;
      lookahead_len_q            <= lookahead_len_d;
      lookahead_len_ldst_q       <= lookahead_len_ldst_d;
    end
  end

  /////////////////////////////////////
  //  Support for misaligned stores  //
  /////////////////////////////////////

  localparam clog2_AxiStrobeWidth = $clog2(AxiDataWidth/8);

  // AXI Request Generation signals, declared here for convenience
  addrgen_req_t axi_addrgen_d, axi_addrgen_q;

  // Narrower AXI Data Byte-Width used for misaligned stores
  logic [$clog2(AxiDataWidth/8)-1:0]            narrow_axi_data_bwidth;
  // Helper signal to calculate the narrow_axi_data_bwidth
  // It carries information about the misalignment of the start address w.r.t. the AxiDataWidth
  logic [$clog2(AxiDataWidth/8)-1:0]            axi_addr_misalignment;
  // Number of trailing 0s of axi_addr_misalignment
  logic [idx_width($clog2(AxiDataWidth/8))-1:0] zeroes_cnt;

  // Get the misalignment information for this vector memory instruction
  assign axi_addr_misalignment = axi_addrgen_d.addr[$clog2(AxiDataWidth/8)-1:0];

  // Calculate the maximum number of Bytes we can send in a store-misaligned beat.
  // This number must be a power of 2 not to get misaligned wrt the pack of data that the
  // store unit receives from the lanes
  lzc #(
    .WIDTH($clog2(AxiDataWidth/8)),
    .MODE (1'b0                  )
  ) i_lzc (
    .in_i   (axi_addr_misalignment),
    .cnt_o  (zeroes_cnt           ),
    .empty_o(/* Unconnected */    )
  );

  // Effective AXI data width for misaligned stores
  assign narrow_axi_data_bwidth = (AxiDataWidth/8) >> ($clog2(AxiDataWidth/8) - zeroes_cnt);

  //////////////////////////////
  //  AXI Request Generation  //
  //////////////////////////////

  // enum logic [1:0] {
  //   AXI_ADDRGEN_IDLE, AXI_ADDRGEN_MISALIGNED, AXI_ADDRGEN_WAITING, AXI_ADDRGEN_REQUESTING
  // } axi_addrgen_state_d, axi_addrgen_state_q;

  enum logic [1:0] {
    AXI_ADDRGEN_IDLE, AXI_ADDRGEN_WAITING, AXI_ADDRGEN_REQUESTING
  } axi_addrgen_state_d, axi_addrgen_state_q;

  axi_addr_t aligned_start_addr_d, aligned_start_addr_q;
  axi_addr_t aligned_next_start_addr_d, aligned_next_start_addr_q, aligned_next_start_addr_temp;
  axi_addr_t aligned_end_addr_d, aligned_end_addr_q, aligned_end_addr_temp;

  // MSb of the next-next page (page selector for page 2 positions after the current one)
  logic [($bits(aligned_start_addr_d) - 12)-1:0] next_2page_msb_d, next_2page_msb_q;

  logic [$clog2(AxiDataWidth/8):0]            eff_axi_dw_d, eff_axi_dw_q;
  logic [idx_width($clog2(AxiDataWidth/8)):0] eff_axi_dw_log_d, eff_axi_dw_log_q;

  function automatic void set_end_addr (
      input  logic [($bits(axi_addr_t) - 12)-1:0]       next_2page_msb,
      input  vlen_t                                     num_bytes,
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

  always_comb begin: axi_addrgen
    // Maintain state
    axi_addrgen_state_d = axi_addrgen_state_q;
    axi_addrgen_d       = axi_addrgen_q;

    aligned_start_addr_d      = aligned_start_addr_q;
    aligned_next_start_addr_d = aligned_next_start_addr_q;
    aligned_end_addr_d        = aligned_end_addr_q;

    next_2page_msb_d = next_2page_msb_q;

    eff_axi_dw_d     = eff_axi_dw_q;
    eff_axi_dw_log_d = eff_axi_dw_log_q;

    idx_addr_ready_d    = 1'b0;
    addrgen_error_vl_d  = '0;

    // No error by default
    idx_op_error_d = 1'b0;

    // No addrgen request to acknowledge
    addrgen_req_ready = 1'b0;

    // No addrgen command to the load/store units
    axi_addrgen_queue      = '0;
    axi_addrgen_queue_push = 1'b0;

    // No AXI request
    axi_ar_o       = '0;
    axi_ar_valid_o = 1'b0;
    axi_aw_o       = '0;
    axi_aw_valid_o = 1'b0;

    vew_ar_o = vew_e'('0); // To be removed
    vew_aw_o = vew_e'('0); // To be removed

    case (axi_addrgen_state_q)
      AXI_ADDRGEN_IDLE: begin
        // This computation is timing-critical. Look ahead and compute even if addr not valid.
        axi_addrgen_d = addrgen_req;

        // The start address is found by aligning the original request address by the width of
        // the memory interface.
        aligned_start_addr_d = aligned_addr(axi_addrgen_d.addr, clog2_AxiStrobeWidth);
        // Pre-calculate the next_2page_msb. This should not require much energy if the addr
        // has zeroes in the upper positions.
        // We can use this also for the misaligned address calculation, as the next 2 page msb
        // will be the same either way.
        next_2page_msb_d = aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
        // The final address can be found similarly...
        set_end_addr (
          next_2page_msb_d,
          axi_addrgen_d.len,
          axi_addrgen_d.addr,
          AxiDataWidth/8,
          clog2_AxiStrobeWidth,
          aligned_start_addr_d,
          aligned_end_addr_d,
          aligned_next_start_addr_d
        );

        if (addrgen_req_valid) begin
          axi_addrgen_state_d = core_st_pending_i ? AXI_ADDRGEN_WAITING : AXI_ADDRGEN_REQUESTING;

          eff_axi_dw_d     = AxiDataWidth/8;
          eff_axi_dw_log_d = clog2_AxiStrobeWidth;
        end
      end
      // AXI_ADDRGEN_IDLE: begin
      //   if (addrgen_req_valid) begin
      //     axi_addrgen_d       = addrgen_req;
      //     axi_addrgen_state_d = core_st_pending_i ? AXI_ADDRGEN_WAITING : AXI_ADDRGEN_REQUESTING;

      //     // In case of a misaligned store, reduce the effective width of the AXI transaction,
      //     // since the store unit does not support misalignments between the AXI bus and the lanes
      //     if ((axi_addrgen_d.addr[$clog2(AxiDataWidth/8)-1:0] != '0) && !axi_addrgen_d.is_load)
      //     begin
      //       // Calculate the start and the end addresses in the AXI_ADDRGEN_MISALIGNED state
      //       axi_addrgen_state_d = AXI_ADDRGEN_MISALIGNED;

      //       eff_axi_dw_d     = {1'b0, narrow_axi_data_bwidth};
      //       eff_axi_dw_log_d = zeroes_cnt;
      //     end else begin
      //       eff_axi_dw_d     = AxiDataWidth/8;
      //       eff_axi_dw_log_d = $clog2(AxiDataWidth/8);
      //     end
          
      //     /*// The start address is found by aligning the original request address by the width of
      //     // the memory interface.
      //     aligned_start_addr_d = aligned_addr(axi_addrgen_d.addr, $clog2(AxiDataWidth/8));
      //     // Pre-calculate the next_2page_msb. This should not require much energy if the addr
      //     // has zeroes in the upper positions.
      //     next_2page_msb_d = aligned_start_addr_d[AxiAddrWidth-1:12] + 1;
      //     // The final address can be found similarly...
      //     if (axi_addrgen_d.len << int'(axi_addrgen_d.vew) >= (256 << $clog2(AxiDataWidth/8))) begin
      //       aligned_next_start_addr_d =
      //         aligned_addr(axi_addrgen_d.addr + (256 << $clog2(AxiDataWidth/8)), $clog2(AxiDataWidth/8));
      //       aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //     end else begin
      //       aligned_next_start_addr_d =
      //         aligned_addr(axi_addrgen_d.addr + (axi_addrgen_d.len << int'(axi_addrgen_d.vew)) - 1,
      //         $clog2(AxiDataWidth/8)) + AxiDataWidth/8;
      //       aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //     end
      //     // But since AXI requests are aligned in 4 KiB pages, aligned_end_addr must be in the
      //     // same page as aligned_start_addr
      //     if (aligned_start_addr_d[AxiAddrWidth-1:12] != aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      //       aligned_end_addr_d        = {aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      //       aligned_next_start_addr_d = {                       next_2page_msb_d, 12'h000};
      //     end*/

      //     /*aligned_start_addr_d = axi_addrgen_d.addr;
      //     aligned_next_start_addr_d = axi_addrgen_d.addr + (axi_addrgen_d.len << int'(axi_addrgen_d.vew));
      //     aligned_end_addr_d = aligned_next_start_addr_d - 1;*/
      //   end
      // end
      // AXI_ADDRGEN_MISALIGNED: begin
      //   axi_addrgen_state_d = core_st_pending_i ? AXI_ADDRGEN_WAITING : AXI_ADDRGEN_REQUESTING;

      //   // The start address is found by aligning the original request address by the width of
      //   // the memory interface.
      //   aligned_start_addr_d = aligned_addr(axi_addrgen_q.addr, eff_axi_dw_log_q);

      //   set_end_addr (
      //     next_2page_msb_q,
      //     axi_addrgen_q.len,
      //     axi_addrgen_q.addr,
      //     eff_axi_dw_q,
      //     eff_axi_dw_log_q,
      //     aligned_start_addr_d,
      //     aligned_end_addr_d,
      //     aligned_next_start_addr_d
      //   );
      // end
      // AXI_ADDRGEN_MISALIGNED: begin
      //   axi_addrgen_state_d = core_st_pending_i ? AXI_ADDRGEN_WAITING : AXI_ADDRGEN_REQUESTING;

      //   // The start address is found by aligning the original request address by the width of
      //   // the memory interface.
      //   aligned_start_addr_d = aligned_addr(axi_addrgen_q.addr, eff_axi_dw_log_q);
      //   // The final address can be found similarly...
      //   if (axi_addrgen_q.len << int'(axi_addrgen_q.vew) >= (256 << eff_axi_dw_log_q)) begin
      //     aligned_next_start_addr_d =
      //       aligned_addr(axi_addrgen_q.addr + (256 << eff_axi_dw_log_q), eff_axi_dw_log_q);
      //     aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //   end else begin
      //     aligned_next_start_addr_d =
      //       aligned_addr(axi_addrgen_q.addr + (axi_addrgen_q.len << int'(axi_addrgen_q.vew)) - 1,
      //       eff_axi_dw_log_q) + eff_axi_dw_q;
      //     aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //   end
      //   // But since AXI requests are aligned in 4 KiB pages, aligned_end_addr must be in the
      //   // same page as aligned_start_addr
      //   if (aligned_start_addr_d[AxiAddrWidth-1:12] != aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      //     aligned_end_addr_d = {aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      //     aligned_next_start_addr_d = {                next_2page_msb_q, 12'h000};
      //   end
      // end
      AXI_ADDRGEN_WAITING: begin
        if (!core_st_pending_i)
          axi_addrgen_state_d = AXI_ADDRGEN_REQUESTING;
      end
      AXI_ADDRGEN_REQUESTING : begin
        automatic logic axi_ax_ready = (axi_addrgen_q.is_load && axi_ar_ready_i) || (!axi_addrgen_q.is_load && axi_aw_ready_i);
        automatic logic [12:0] num_bytes; // Cannot consume more than 4 KiB
        automatic vlen_t remaining_bytes;

        // Pre-calculate the next_2page_msb. This should not require much energy if the addr
        // has zeroes in the upper positions.
        next_2page_msb_d = aligned_next_start_addr_q[AxiAddrWidth-1:12] + 1;
        // Pre-calculate the bytes used in a unit-strided access and the remaining bytes to ask.
        // The proper way to do so would be aligned_end_addr_q[11:0] + 1 - axi_addrgen_q.addr[11:0].
        // Avoid explicit computation of aligned_next_start_addr + 1 with a trick that works if
        // aligned_next_start_addr <= 12'hFFF.
        if (aligned_end_addr_q[11:0] != 12'hFFF) begin
          num_bytes = aligned_next_start_addr_q[11:0] - axi_addrgen_q.addr[11:0];
        end else begin
        // Special case: aligned_next_start_addr > 12'hFFF.
          num_bytes = 13'h1000 - axi_addrgen_q.addr[11:0];
        end
        // remaining_bytes = axi_addrgen_q.len - num_bytes;
        // if (axi_addrgen_q.len < num_bytes) begin
        //   remaining_bytes = 0;
        // end
        remaining_bytes = 0;

        // Before starting a transaction on a different channel, wait the formers to complete
        // Otherwise, the ordering of the responses is not guaranteed, and with the current
        // implementation we can incur in deadlocks
        // NOTE: this might be referring to an obsolete axi_cut implementation
        if (axi_addrgen_queue_empty || (axi_addrgen_req_o.is_load && axi_addrgen_q.is_load) ||
             (~axi_addrgen_req_o.is_load && ~axi_addrgen_q.is_load)) begin : axi_ax_idle
          if (!axi_addrgen_queue_full && axi_ax_ready) begin : start_req
            automatic axi_addr_t paddr;

            // Mux target address
            paddr = axi_addrgen_q.addr;

            // Prepare data in advance
            if (axi_addrgen_q.is_burst) begin : unit_stride_data
              /////////////////////////
              //  Unit-Stride access //
              /////////////////////////

              // NOTE: all these variables could be narrowed to the minimum number of bits
              automatic int unsigned num_beats;

              // AXI burst length
              automatic int unsigned burst_length;

              // 1 - AXI bursts are at most 256 beats long.
              burst_length = 256;
              // 2 - The AXI burst length cannot be longer than the number of beats required
              //     to access the memory regions between aligned_start_addr and
              //     aligned_end_addr
              num_beats = ((aligned_end_addr_q[11:0] - aligned_start_addr_q[11:0]) >> eff_axi_dw_log_q) + 1;
              if (burst_length > num_beats) begin
                burst_length = num_beats;
              end

              burst_length = ((axi_addrgen_q.len-1) >> eff_axi_dw_log_q) + 1;

              // AR Channel
              if (axi_addrgen_q.is_load) begin
                axi_ar_o = '{
                  addr   : paddr,
                  len    : burst_length - 1,
                  size   : eff_axi_dw_log_q,
                  cache  : CACHE_MODIFIABLE,
                  burst  : BURST_INCR,
                  default: '0
                };
              end
              // AW Channel
              else begin
                axi_aw_o = '{
                  addr   : paddr,
                  len    : burst_length - 1,
                  // If misaligned store access, reduce the effective AXI width
                  // This hurts performance
                  size   : eff_axi_dw_log_q,
                  cache  : CACHE_MODIFIABLE,
                  burst  : BURST_INCR,
                  default: '0
                };
              end

              // Send this request to the load/store units
              axi_addrgen_queue = '{
                addr         : paddr,
                len          : burst_length - 1,
                size         : eff_axi_dw_log_q,
                is_load      : axi_addrgen_q.is_load,
                vl_ldst      : axi_addrgen_q.vl_ldst
              };

              // Calculate the addresses for the next iteration
              // TODO: test this for SEW!=64, otherwise this computation is never used
              // The start address is found by aligning the original request address by the width of
              // the memory interface. In our case, we have it already.
              set_end_addr (
                next_2page_msb_d,
                axi_addrgen_q.len - num_bytes,
                aligned_next_start_addr_q,
                eff_axi_dw_q,
                eff_axi_dw_log_q,
                aligned_next_start_addr_q,
                aligned_end_addr_temp,
                aligned_next_start_addr_temp
              );
            end : unit_stride_data
            else if (state_q != ADDRGEN_IDX_OP) begin : strided_data
              /////////////////////
              //  Strided access //
              /////////////////////
              // AR Channel
              if (axi_addrgen_q.is_load) begin
                axi_ar_o = '{
                  addr   : paddr,
                  len    : 0,
                  size   : axi_addrgen_q.vew,
                  cache  : CACHE_MODIFIABLE,
                  burst  : BURST_INCR,
                  default: '0
                };
              end
              // AW Channel
              else begin
                axi_aw_o = '{
                  addr   : paddr,
                  len    : 0,
                  size   : axi_addrgen_q.vew,
                  cache  : CACHE_MODIFIABLE,
                  burst  : BURST_INCR,
                  default: '0
                };
              end

              // Send this request to the load/store units
              axi_addrgen_queue = '{
                addr         : paddr,
                size         : axi_addrgen_q.vew,
                len          : 0,
                is_load      : axi_addrgen_q.is_load,
                vl_ldst      : axi_addrgen_q.vl_ldst
              };

              // Account for the requested operands
              // This should never overflow
              len_temp = axi_addrgen_q.len - (1 << axi_addrgen_q.vew);
              // Calculate the addresses for the next iteration, adding the correct stride
              next_addr_strided_temp = axi_addrgen_q.addr + axi_addrgen_q.stride;
            end : strided_data
            else begin : indexed_data
              // NOTE: address translation is not yet been implemented/tested for indexed

              automatic axi_addr_t idx_final_paddr;
              //////////////////////
              //  Indexed access  //
              //////////////////////

              // TODO: check if idx_addr_valid_q is stable
              if (idx_addr_valid_q) begin : if_idx_addr_valid_q
                // Check if the virtual address generates an exception
                // NOTE: we can do this even before address translation, since the
                //       page offset (2^12) is the same for both physical and virtual addresses
                if (is_addr_error(idx_final_addr_q, axi_addrgen_q.vew[1:0])) begin : eew_misaligned_error
                  // Generate an error
                  idx_op_error_d          = 1'b1;
                  // Forward next vstart info to the dispatcher
                  addrgen_error_vl_d      = addrgen_req.len - axi_addrgen_q.len - 1;
                  addrgen_req_ready       = 1'b1;
                  axi_addrgen_state_d     = AXI_ADDRGEN_IDLE;
                end : eew_misaligned_error
                else begin : aligned_vaddress
                  // Mux target address
                  idx_final_paddr = idx_final_addr_q;

                  // AR Channel
                  if (axi_addrgen_q.is_load) begin
                    axi_ar_o = '{
                      addr   : idx_final_paddr,
                      len    : 0,
                      size   : axi_addrgen_q.vew,
                      cache  : CACHE_MODIFIABLE,
                      burst  : BURST_INCR,
                      default: '0
                    };
                  end
                  // AW Channel
                  else begin
                    axi_aw_o = '{
                      addr   : idx_final_paddr,
                      len    : 0,
                      size   : axi_addrgen_q.vew,
                      cache  : CACHE_MODIFIABLE,
                      burst  : BURST_INCR,
                      default: '0
                    };
                  end

                  // Prepare the request for the load or store unit
                  axi_addrgen_queue = '{
                    addr         : idx_final_paddr,
                    size         : axi_addrgen_q.vew,
                    len          : 0,
                    is_load      : axi_addrgen_q.is_load,
                    vl_ldst      : axi_addrgen_q.vl_ldst
                  };

                  // Account for the requested operands
                  // This should never overflow
                  len_temp = axi_addrgen_q.len - (1 << axi_addrgen_q.vew);
                end : aligned_vaddress
              end : if_idx_addr_valid_q
            end : indexed_data

            if (axi_addrgen_q.is_burst) begin : unit_stride // UNIT-STRIDED ACCESS
              // AR Channel
              axi_ar_valid_o = axi_addrgen_q.is_load;
              // AW Channel
              axi_aw_valid_o = ~axi_addrgen_q.is_load;

              // Send this request to the load/store units
              axi_addrgen_queue_push = 1'b1;

              // We pre-calculated the values already
              axi_addrgen_d.len  = remaining_bytes;
              axi_addrgen_d.addr = aligned_next_start_addr_q;

              aligned_start_addr_d      = axi_addrgen_d.addr;
              aligned_end_addr_d        = aligned_end_addr_temp;
              aligned_next_start_addr_d = aligned_next_start_addr_temp;
            end : unit_stride
            else if (state_q != ADDRGEN_IDX_OP) begin : strided // STRIDED ACCESS
              // AR Channel
              axi_ar_valid_o = axi_addrgen_q.is_load;
              // AW Channel
              axi_aw_valid_o = ~axi_addrgen_q.is_load;

              // Send this request to the load/store units
              axi_addrgen_queue_push = 1'b1;

              // We pre-calculated the values already
              axi_addrgen_d.len = len_temp;
              axi_addrgen_d.addr = next_addr_strided_temp;
            end : strided
            else begin : indexed // INDEXED ACCESS
              automatic axi_addr_t idx_final_paddr;
              // TODO: check if idx_addr_valid_q is stable
              if (idx_addr_valid_q) begin : if_idx_addr_valid_q

                // Check if the virtual address generates an exception
                // NOTE: we can do this even before address translation, since the
                //       page offset (2^12) is the same for both physical and virtual addresses
                if (!is_addr_error(idx_final_addr_q, axi_addrgen_q.vew[1:0])) begin : aligned_vaddress
                  // We consumed a word
                  idx_addr_ready_d = 1'b1;

                  // AR Channel
                  axi_ar_valid_o = axi_addrgen_q.is_load;
                  // AW Channel
                  axi_aw_valid_o = ~axi_addrgen_q.is_load;

                  // Send this request to the load/store units
                  axi_addrgen_queue_push = 1'b1;

                  // We pre-calculated the values already
                  axi_addrgen_d.len = len_temp;
                end : aligned_vaddress
              end : if_idx_addr_valid_q
            end : indexed
            // Finished generating AXI requests
            if (axi_addrgen_d.len == 0) begin : finished
              addrgen_req_ready   = 1'b1;
              axi_addrgen_state_d = AXI_ADDRGEN_IDLE;
            end : finished
          end
        end
      end
      // AXI_ADDRGEN_REQUESTING : begin
      //   automatic logic axi_ax_ready = (axi_addrgen_q.is_load && axi_ar_ready_i) || (!
      //     axi_addrgen_q.is_load && axi_aw_ready_i);

      //   // Pre-calculate the next_2page_msb. This should not require much energy if the addr
      //   // has zeroes in the upper positions.
      //   next_2page_msb_d = aligned_next_start_addr_q[AxiAddrWidth-1:12] + 1;

      //   // Before starting a transaction on a different channel, wait the formers to complete
      //   // Otherwise, the ordering of the responses is not guaranteed, and with the current
      //   // implementation we can incur in deadlocks
      //   if (axi_addrgen_queue_empty || (axi_addrgen_req_o.is_load && axi_addrgen_q.is_load) ||
      //       (~axi_addrgen_req_o.is_load && ~axi_addrgen_q.is_load)) begin
      //     if (!axi_addrgen_queue_full && axi_ax_ready) begin
      //       if (axi_addrgen_q.is_burst) begin

      //         /////////////////////////
      //         //  Unit-Stride access //
      //         /////////////////////////

      //         // AXI burst length
      //         automatic int unsigned burst_length;

      //         // 1 - AXI bursts are at most 256 beats long.
      //         burst_length = 256;
      //         // 2 - The AXI burst length cannot be longer than the number of beats required
      //         //     to access the memory regions between aligned_start_addr and
      //         //     aligned_end_addr
      //         // if (burst_length > ((aligned_end_addr_q - aligned_start_addr_q) >>
      //         //       eff_axi_dw_log_q) + 1)
      //         // burst_length = ((aligned_end_addr_q - aligned_start_addr_q) >>
      //         //   eff_axi_dw_log_q) + 1;

      //         burst_length = (((axi_addrgen_q.len << int'(axi_addrgen_q.vew))-1) >> eff_axi_dw_log_q) + 1;

      //         // if (burst_length > ((aligned_end_addr_q[11:0] - aligned_start_addr_q[11:0]) >>
      //         //       eff_axi_dw_log_q) + 1)
      //         //   burst_length = ((aligned_end_addr_q[11:0] - aligned_start_addr_q[11:0]) >>
      //         //     eff_axi_dw_log_q) + 1;
              
      //         // The above burst length is only used for vldu and vstu to know how many packets to receive.
      //         // The AXI request to Memory in Global Ld-St unit takes care of the maximum burst limit.
      //         // AR Channel
      //         if (axi_addrgen_q.is_load) begin
      //           axi_ar_o = '{
      //             addr   : axi_addrgen_q.addr,
      //             len    : burst_length - 1,
      //             size   : eff_axi_dw_log_q,
      //             cache  : CACHE_MODIFIABLE,
      //             burst  : BURST_INCR,
      //             default: '0
      //           };
      //           vew_ar_o = axi_addrgen_q.vew;
      //           axi_ar_valid_o = 1'b1;
      //         end
      //         // AW Channel
      //         else begin
      //           axi_aw_o = '{
      //             addr   : axi_addrgen_q.addr,
      //             len    : burst_length - 1,
      //             // If misaligned store access, reduce the effective AXI width
      //             // This hurts performance
      //             size   : eff_axi_dw_log_q,
      //             cache  : CACHE_MODIFIABLE,
      //             burst  : BURST_INCR,
      //             default: '0
      //           };
      //           vew_aw_o = axi_addrgen_q.vew; 
      //           axi_aw_valid_o = 1'b1;
      //         end

      //         // Send this request to the load/store units
      //         axi_addrgen_queue = '{
      //           addr   : axi_addrgen_q.addr,
      //           len    : burst_length - 1,
      //           size   : eff_axi_dw_log_q,
      //           is_load: axi_addrgen_q.is_load
      //         };
      //         axi_addrgen_queue_push = 1'b1;

      //         // Account for the requested operands
      //         /*axi_addrgen_d.len = axi_addrgen_q.len - ((aligned_end_addr_q[11:0] - axi_addrgen_q.addr[11:0] + 1)
      //             >> int'(axi_addrgen_q.vew));
      //         if (axi_addrgen_q.len < ((aligned_end_addr_q[11:0] - axi_addrgen_q.addr[11:0] + 1)
      //             >> int'(axi_addrgen_q.vew)))
      //           axi_addrgen_d.len = 0;
      //         axi_addrgen_d.addr = aligned_next_start_addr_q;
      //         */
      //         // Finished generating AXI requests
      //         // if (axi_addrgen_d.len == 0) begin
      //           addrgen_req_ready   = 1'b1;
      //           axi_addrgen_state_d = AXI_ADDRGEN_IDLE;
      //         // end
      //         /*
      //         // Calculate the addresses for the next iteration
      //         // The start address is found by aligning the original request address by the width of
      //         // the memory interface. In our case, we have it already.
      //         aligned_start_addr_d = axi_addrgen_d.addr;
      //         // The final address can be found similarly.
      //         // How many B we requested? No more than (256 << burst_size)
      //         if (axi_addrgen_d.len << int'(axi_addrgen_q.vew) >= (256 << eff_axi_dw_log_q)) begin
      //           aligned_next_start_addr_d =
      //             aligned_addr(aligned_start_addr_d + (256 << eff_axi_dw_log_q), eff_axi_dw_log_q);
      //           aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //         end else begin
      //           aligned_next_start_addr_d =
      //             aligned_addr(aligned_start_addr_d + (axi_addrgen_d.len << int'(axi_addrgen_q.vew))
      //             - 1, eff_axi_dw_log_q) + eff_axi_dw_q;
      //           aligned_end_addr_d = aligned_next_start_addr_d - 1;
      //         end
      //         // But since AXI requests are aligned in 4 KiB pages, aligned_end_addr must be in the
      //         // same page as aligned_start_addr
      //         if (aligned_start_addr_d[AxiAddrWidth-1:12] != aligned_end_addr_d[AxiAddrWidth-1:12]) begin
      //           aligned_end_addr_d        = {aligned_start_addr_d[AxiAddrWidth-1:12], 12'hFFF};
      //           aligned_next_start_addr_d = {                       next_2page_msb_d, 12'h000};
      //         end*/
              
      //       end else if (state_q != ADDRGEN_IDX_OP) begin

      //         /////////////////////
      //         //  Strided access //
      //         /////////////////////

      //         // AR Channel
      //         if (axi_addrgen_q.is_load) begin
      //           axi_ar_o = '{
      //             addr   : axi_addrgen_q.addr,
      //             len    : 0,
      //             size   : axi_addrgen_q.vew,
      //             cache  : CACHE_MODIFIABLE,
      //             burst  : BURST_INCR,
      //             default: '0
      //           };
      //           axi_ar_valid_o = 1'b1;
      //         end
      //         // AW Channel
      //         else begin
      //           axi_aw_o = '{
      //             addr   : axi_addrgen_q.addr,
      //             len    : 0,
      //             size   : axi_addrgen_q.vew,
      //             cache  : CACHE_MODIFIABLE,
      //             burst  : BURST_INCR,
      //             default: '0
      //           };
      //           axi_aw_valid_o = 1'b1;
      //         end

      //         // Send this request to the load/store units
      //         axi_addrgen_queue = '{
      //           addr   : axi_addrgen_q.addr,
      //           size   : axi_addrgen_q.vew,
      //           len    : 0,
      //           is_load: axi_addrgen_q.is_load
      //         };
      //         axi_addrgen_queue_push = 1'b1;

      //         // Account for the requested operands
      //         axi_addrgen_d.len  = axi_addrgen_q.len - 1;
      //         // Calculate the addresses for the next iteration, adding the correct stride
      //         axi_addrgen_d.addr = axi_addrgen_q.addr + axi_addrgen_q.stride;

      //         // Finished generating AXI requests
      //         if (axi_addrgen_d.len == 0) begin
      //           addrgen_req_ready   = 1'b1;
      //           axi_addrgen_state_d = AXI_ADDRGEN_IDLE;
      //         end
      //       end else begin

      //         //////////////////////
      //         //  Indexed access  //
      //         //////////////////////

      //         if (idx_addr_valid_q) begin
      //           // We consumed a word
      //           idx_addr_ready_d = 1'b1;

      //           // AR Channel
      //           if (axi_addrgen_q.is_load) begin
      //             axi_ar_o = '{
      //               addr   : idx_final_addr_q,
      //               len    : 0,
      //               size   : axi_addrgen_q.vew,
      //               cache  : CACHE_MODIFIABLE,
      //               burst  : BURST_INCR,
      //               default: '0
      //             };
      //             axi_ar_valid_o = 1'b1;
      //           end
      //           // AW Channel
      //           else begin
      //             axi_aw_o = '{
      //               addr   : idx_final_addr_q,
      //               len    : 0,
      //               size   : axi_addrgen_q.vew,
      //               cache  : CACHE_MODIFIABLE,
      //               burst  : BURST_INCR,
      //               default: '0
      //             };
      //             axi_aw_valid_o = 1'b1;
      //           end

      //           // Send this request to the load/store units
      //           axi_addrgen_queue = '{
      //             addr   : idx_final_addr_q,
      //             size   : axi_addrgen_q.vew,
      //             len    : 0,
      //             is_load: axi_addrgen_q.is_load
      //           };
      //           axi_addrgen_queue_push = 1'b1;

      //           // Account for the requested operands
      //           axi_addrgen_d.len = axi_addrgen_q.len - 1;

      //           // Check if the address does generate an exception
      //           if (is_addr_error(idx_final_addr_q, axi_addrgen_q.vew)) begin
      //             // Generate an error
      //             idx_op_error_d          = 1'b1;
      //             // Forward next vstart info to the dispatcher
      //             addrgen_error_vl_d      = addrgen_req.len - axi_addrgen_q.len - 1;
      //             addrgen_req_ready       = 1'b1;
      //             axi_addrgen_state_d     = AXI_ADDRGEN_IDLE;
      //           end

      //           // Finished generating AXI requests
      //           if (axi_addrgen_d.len == 0) begin
      //             addrgen_req_ready   = 1'b1;
      //             axi_addrgen_state_d = AXI_ADDRGEN_IDLE;
      //           end
      //         end
      //       end
      //     end
      //   end
      // end
    endcase
  end: axi_addrgen

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      axi_addrgen_state_q       <= AXI_ADDRGEN_IDLE;
      axi_addrgen_q             <= '0;
      aligned_start_addr_q      <= '0;
      aligned_next_start_addr_q <= '0;
      aligned_end_addr_q        <= '0;
      eff_axi_dw_q              <= '0;
      eff_axi_dw_log_q          <= '0;
      next_2page_msb_q          <= '0;
    end else begin
      axi_addrgen_state_q       <= axi_addrgen_state_d;
      axi_addrgen_q             <= axi_addrgen_d;
      aligned_start_addr_q      <= aligned_start_addr_d;
      aligned_next_start_addr_q <= aligned_next_start_addr_d;
      aligned_end_addr_q        <= aligned_end_addr_d;
      eff_axi_dw_q              <= eff_axi_dw_d;
      eff_axi_dw_log_q          <= eff_axi_dw_log_d;
      next_2page_msb_q          <= next_2page_msb_d;
    end
  end

endmodule : addrgen
