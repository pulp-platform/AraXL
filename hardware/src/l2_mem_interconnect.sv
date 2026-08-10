// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Hong Pang <hopang@iis.ee.ethz.ch>
//
// Description:
// Word-interleaved memory interconnect between NoSlvPorts requesters (each an
// axi_to_mem memory port) and NoMstPorts single-port SRAM banks.

module l2_mem_interconnect #(
  /// Number of requester ports (cores plus the shared atomics path).
  parameter int unsigned NoSlvPorts = 32'd2,
  /// Number of memory banks. Must be a power of two.
  parameter int unsigned NoMstPorts = 32'd2,
  /// Address width of the byte-addressed requester ports.
  parameter int unsigned AddrWidth  = 32'd64,
  /// Data width of one memory word. Must be a power of two number of bytes.
  parameter int unsigned DataWidth  = 32'd256,
  /// Bank read latency in cycles. Must match tc_sram's Latency.
  parameter int unsigned Latency    = 32'd1,
  /// Dependent parameter, do not override! Write strobe width.
  localparam int unsigned StrbWidth = DataWidth/8
) (
  /// Clock input.
  input  logic clk_i,
  /// Asynchronous reset, active low.
  input  logic rst_ni,
  /// Slave ports, request is valid.
  input  logic [NoSlvPorts-1:0]                slv_ports_req_i,
  /// Slave ports, request can be granted.
  output logic [NoSlvPorts-1:0]                slv_ports_gnt_o,
  /// Slave ports, request write enable, active high.
  input  logic [NoSlvPorts-1:0]                slv_ports_we_i,
  /// Slave ports, request address, byte-wise.
  input  logic [NoSlvPorts-1:0][AddrWidth-1:0] slv_ports_addr_i,
  /// Slave ports, request write strobe.
  input  logic [NoSlvPorts-1:0][StrbWidth-1:0] slv_ports_strb_i,
  /// Slave ports, request write data.
  input  logic [NoSlvPorts-1:0][DataWidth-1:0] slv_ports_wdata_i,
  /// Slave ports, response read data.
  output logic [NoSlvPorts-1:0][DataWidth-1:0] slv_ports_rdata_o,
  /// Slave ports, response is valid. Asserted for reads and writes alike.
  output logic [NoSlvPorts-1:0]                slv_ports_rvalid_o,
  /// Master ports, request is valid. No gnt: the SRAM always accepts.
  output logic [NoMstPorts-1:0]                mst_ports_req_o,
  /// Master ports, request write enable, active high.
  output logic [NoMstPorts-1:0]                mst_ports_we_o,
  /// Master ports, request address as a word index, bank bits removed.
  output logic [NoMstPorts-1:0][AddrWidth-1:0] mst_ports_addr_o,
  /// Master ports, request write strobe.
  output logic [NoMstPorts-1:0][StrbWidth-1:0] mst_ports_strb_o,
  /// Master ports, request write data.
  output logic [NoMstPorts-1:0][DataWidth-1:0] mst_ports_wdata_o,
  /// Master ports, response read data, valid Latency cycles after the request.
  input  logic [NoMstPorts-1:0][DataWidth-1:0] mst_ports_rdata_i
);

  `include "common_cells/registers.svh"

  // Bits of the byte offset inside one memory word: the bank-select field
  // starts immediately above them.
  localparam int unsigned WordOffLog2  = $clog2(StrbWidth);
  // 0 when there is a single bank, in which case the mapping is the identity.
  localparam int unsigned BankIdxWidth = $clog2(NoMstPorts);
  localparam int unsigned IndexLow     = WordOffLog2 + BankIdxWidth;
  // Widths only so signal declarations stay legal in the degenerate cases.
  localparam int unsigned BankSelW     = (BankIdxWidth == 0) ? 32'd1 : BankIdxWidth;
  localparam int unsigned SlvIdxWidth  = (NoSlvPorts > 32'd1) ? $clog2(NoSlvPorts) : 32'd1;

  // A requester's memory request, as presented on the slave port.
  typedef struct packed {
    logic                 we;
    logic [AddrWidth-1:0] addr;    // byte address, undecoded
    logic [StrbWidth-1:0] strb;
    logic [DataWidth-1:0] wdata;
  } req_t;

  // Payload carried through a bank's arbiter.
  typedef struct packed {
    logic                 we;
    logic [AddrWidth-1:0] addr;    // word index, bank bits removed
    logic [StrbWidth-1:0] strb;
    logic [DataWidth-1:0] wdata;
  } payload_t;

  ///////////////////////////////
  //  Requester input buffer   //
  ///////////////////////////////

  req_t [NoSlvPorts-1:0] slv_req_in, slv_req_buf;
  logic [NoSlvPorts-1:0] slv_buf_valid, slv_buf_ready;

  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_slv_buf
    assign slv_req_in[s] = '{
      we   : slv_ports_we_i   [s],
      addr : slv_ports_addr_i [s],
      strb : slv_ports_strb_i [s],
      wdata: slv_ports_wdata_i[s]
    };

    spill_register #(
      .T     (req_t),
      .Bypass(1'b0 )   // MUST stay 0: Bypass=1 is `ready_o = ready_i`, the loop
    ) i_slv_spill (
      .clk_i  (clk_i             ),
      .rst_ni (rst_ni            ),
      .valid_i(slv_ports_req_i[s]),
      .ready_o(slv_ports_gnt_o[s]),
      .data_i (slv_req_in[s]     ),
      .valid_o(slv_buf_valid[s]  ),
      .ready_i(slv_buf_ready[s]  ),
      .data_o (slv_req_buf[s]    )
    );
  end

  ////////////////////////
  //  Address decoding  //
  ////////////////////////

  logic     [NoSlvPorts-1:0][BankSelW-1:0] bank_sel;
  payload_t [NoSlvPorts-1:0]               slv_payload;

  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_slv_decode
    if (BankIdxWidth == 0) begin : gen_single_bank
      assign bank_sel[s] = '0;
    end else begin : gen_multi_bank
      assign bank_sel[s] = slv_req_buf[s].addr[WordOffLog2 +: BankIdxWidth];
    end

    assign slv_payload[s] = '{
      we   : slv_req_buf[s].we,
      addr : AddrWidth'(slv_req_buf[s].addr >> IndexLow),
      strb : slv_req_buf[s].strb,
      wdata: slv_req_buf[s].wdata
    };
  end

  /////////////////////
  //  Arbitration    //
  /////////////////////

  // req_matrix[m][s] is set when requester s is asking for bank m.
  logic [NoMstPorts-1:0][NoSlvPorts-1:0] req_matrix;
  logic [NoMstPorts-1:0][NoSlvPorts-1:0] gnt_matrix;

  for (genvar m = 0; m < NoMstPorts; m++) begin : gen_req_matrix
    for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_req_matrix_slv
      assign req_matrix[m][s] = slv_buf_valid[s] & (bank_sel[s] == BankSelW'(m));
    end
  end

  // The grant MUST be qualified with the request for that same bank. An idle
  // bank's arbiter can assert gnt_o for a requester that is not asking it:
  // rr_arb_tree only masks gnt_o with req_i when AxiVldRdy == 0
  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_slv_gnt
    logic [NoMstPorts-1:0] gnt_per_bank;
    for (genvar m = 0; m < NoMstPorts; m++) begin : gen_gnt_per_bank
      assign gnt_per_bank[m] = gnt_matrix[m][s] & req_matrix[m][s];
    end
    assign slv_buf_ready[s] = |gnt_per_bank;
  end

  logic [NoMstPorts-1:0][SlvIdxWidth-1:0] bank_win_idx;

  for (genvar m = 0; m < NoMstPorts; m++) begin : gen_bank_arb
    payload_t bank_payload;

    // AxiVldRdy MUST stay 0: with 1, rr_arb_tree drops the `& req_i` term and
    // gnt_o can be asserted for an input that is not requesting. The grant
    // fan-in above qualifies it anyway, but leaving this at 0 keeps gnt_matrix
    // itself meaningful (and the "granted an absent request" assertion below a
    // real invariant).
    rr_arb_tree #(
      .NumIn    (NoSlvPorts),
      .DataType (payload_t ),
      .ExtPrio  (1'b0      ),
      .AxiVldRdy(1'b0      ),
      .LockIn   (1'b0      ),
      .FairArb  (1'b1      )
    ) i_bank_arb (
      .clk_i  (clk_i           ),
      .rst_ni (rst_ni          ),
      .flush_i(1'b0            ),
      .rr_i   ('0              ),
      .req_i  (req_matrix[m]   ),
      .gnt_o  (gnt_matrix[m]   ),
      .data_i (slv_payload     ),
      .req_o  (mst_ports_req_o[m]),
      .gnt_i  (1'b1            ), // the SRAM always accepts
      .data_o (bank_payload    ),
      .idx_o  (bank_win_idx[m] )
    );

    assign mst_ports_we_o   [m] = bank_payload.we;
    assign mst_ports_addr_o [m] = bank_payload.addr;
    assign mst_ports_strb_o [m] = bank_payload.strb;
    assign mst_ports_wdata_o[m] = bank_payload.wdata;
  end

  //////////////////////////
  //  Response steering   //
  //////////////////////////

  // Per bank, remember which requester won, for Latency cycles.
  logic [Latency-1:0][NoMstPorts-1:0]                  tag_vld_q;
  logic [Latency-1:0][NoMstPorts-1:0][SlvIdxWidth-1:0] tag_idx_q;

  `FF(tag_vld_q[0], mst_ports_req_o, '0)
  `FF(tag_idx_q[0], bank_win_idx,    '0)

  for (genvar l = 1; l < Latency; l++) begin : gen_tag_pipe
    `FF(tag_vld_q[l], tag_vld_q[l-1], '0)
    `FF(tag_idx_q[l], tag_idx_q[l-1], '0)
  end

  logic [NoSlvPorts-1:0][NoMstPorts-1:0] resp_hit;

  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_slv_resp
    for (genvar m = 0; m < NoMstPorts; m++) begin : gen_resp_hit
      assign resp_hit[s][m] = tag_vld_q[Latency-1][m] &
                              (tag_idx_q[Latency-1][m] == SlvIdxWidth'(s));
    end

    assign slv_ports_rvalid_o[s] = |resp_hit[s];

    always_comb begin
      slv_ports_rdata_o[s] = '0;
      for (int unsigned m = 0; m < NoMstPorts; m++)
        if (resp_hit[s][m]) slv_ports_rdata_o[s] = mst_ports_rdata_i[m];
    end
  end

  //////////////////
  //  Assertions  //
  //////////////////

  if (NoMstPorts == 0 || (NoMstPorts & (NoMstPorts - 1)) != 0)
    $error("[l2_mem_interconnect] NoMstPorts must be a power of two.");

  if (NoSlvPorts == 0)
    $error("[l2_mem_interconnect] NoSlvPorts must be greater than zero.");

  if (Latency == 0)
    $error("[l2_mem_interconnect] Latency must be at least one.");

  if (StrbWidth * 8 != DataWidth || (StrbWidth & (StrbWidth - 1)) != 0)
    $error("[l2_mem_interconnect] DataWidth must be a power-of-two number of bytes.");

  // pragma translate_off
  `ifndef VERILATOR
  `ifndef XSIM
  // The whole no-reorder-buffer argument rests on this: a requester issues in
  // order and all banks share one latency, so two banks can never return to
  // the same requester in the same cycle.
  // --- Decode: a valid request must reach exactly one arbiter --------------
  // If this fires, requester s's request never becomes visible to any bank
  // (or to more than one), i.e. the bank_sel decode is wrong.
  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_decode_assert
    logic [NoMstPorts-1:0] req_col;
    for (genvar m = 0; m < NoMstPorts; m++) begin : gen_req_col
      assign req_col[m] = req_matrix[m][s];
    end

    assert property (@(posedge clk_i) disable iff (~rst_ni)
                     slv_buf_valid[s] |-> $onehot(req_col))
      else $fatal(1, "[l2_mem_interconnect] requester %0d valid but req_col=%b (bank_sel=%0d, addr=%h)",
                  s, req_col, bank_sel[s], slv_req_buf[s].addr);
  end

  // --- Arbitration: gnt_i is tied high, so a bank with any pending request
  //     MUST grant someone every cycle. If this fires, rr_arb_tree is the bug.
  for (genvar m = 0; m < NoMstPorts; m++) begin : gen_arb_assert
    assert property (@(posedge clk_i) disable iff (~rst_ni)
                     (|req_matrix[m]) |-> (|gnt_matrix[m]))
      else $fatal(1, "[l2_mem_interconnect] bank %0d has requests %b but granted nobody",
                  m, req_matrix[m]);

    // A grant must correspond to an actual request on that bank.
    assert property (@(posedge clk_i) disable iff (~rst_ni)
                     (gnt_matrix[m] & ~req_matrix[m]) == '0)
      else $fatal(1, "[l2_mem_interconnect] bank %0d granted an absent request: req=%b gnt=%b",
                  m, req_matrix[m], gnt_matrix[m]);
  end

  // --- Liveness: name the stuck requester instead of hanging silently ------
  localparam int unsigned StallLimit = 32'd2000;

  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_liveness
    int unsigned stall_cnt;
    int unsigned outstanding;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        stall_cnt   <= 32'd0;
        outstanding <= 32'd0;
      end else begin
        // (a) request pending but never granted
        if (slv_buf_valid[s] && !slv_buf_ready[s]) begin
          stall_cnt <= stall_cnt + 32'd1;
          if (stall_cnt == StallLimit) begin
            $display("[l2_mem_interconnect] requester %0d STARVED %0d cycles: bank=%0d addr=%h we=%b",
                     s, StallLimit, bank_sel[s], slv_req_buf[s].addr, slv_req_buf[s].we);
            $display("    slv_buf_valid=%b slv_buf_ready=%b", slv_buf_valid, slv_buf_ready);
            $display("    req_matrix[%0d]=%b gnt_matrix[%0d]=%b",
                     bank_sel[s], req_matrix[bank_sel[s]], bank_sel[s], gnt_matrix[bank_sel[s]]);
            $fatal(1, "[l2_mem_interconnect] starvation");
          end
        end else begin
          stall_cnt <= 32'd0;
        end

        // (b) granted but the response never came back
        outstanding <= outstanding
                     + ((slv_buf_valid[s] && slv_buf_ready[s]) ? 32'd1 : 32'd0)
                     - (slv_ports_rvalid_o[s] ? 32'd1 : 32'd0);
        if (outstanding > (Latency + 32'd8))
          $fatal(1, "[l2_mem_interconnect] requester %0d: %0d responses outstanding (Latency=%0d) -- steering lost one",
                 s, outstanding, Latency);
      end
    end
  end

  for (genvar s = 0; s < NoSlvPorts; s++) begin : gen_resp_assert
    assert property (@(posedge clk_i) disable iff (~rst_ni) $onehot0(resp_hit[s]))
      else $fatal(1, "[l2_mem_interconnect] requester %0d received responses from more than one bank in the same cycle", s);
  end
  `endif
  `endif
  // pragma translate_on

endmodule : l2_mem_interconnect
