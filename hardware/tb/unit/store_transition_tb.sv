// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Queued stores must retain their output route until their pipeline tail drains.
`include "axi/typedef.svh"
module store_transition_case #(parameter int C = 2)(output logic done);
  import ara_pkg::*;
  import rvv_pkg::*;
  localparam int Lanes = 4;
  localparam int Beats = 32;
  typedef logic [63:0] addr_t;
  typedef logic id_t;
  typedef logic user_t;
  typedef logic [127:0] data_t;
  typedef logic [15:0] strb_t;
  `AXI_TYPEDEF_AW_CHAN_T(aw_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_t, data_t, strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_t, data_t, id_t, user_t)
  `AXI_TYPEDEF_REQ_T(req_t, aw_t, w_t, ar_t)
  `AXI_TYPEDEF_RESP_T(resp_t, b_t, r_t)
  logic clk = 0, rst_n;
  always #5 clk = ~clk;
  req_t [C-1:0] req, out_req;
  resp_t [C-1:0] resp, out_resp;
  cluster_metadata_t [C-1:0] ar_meta, aw_meta;
  int received[C];
  bit [C-1:0] stalled;
  w_t [C-1:0] held;
  vew_e width;
  bit indexed_phase, checking;
  int tag;

  shuffle_stage #(.NrLanes(Lanes), .NrClusters(C), .ClusterAxiDataWidth(128),
    .AxiAddrWidth(64), .axi_r_t(r_t), .axi_w_t(w_t),
    .axi_req_t(req_t), .axi_resp_t(resp_t)) dut (
    .clk_i(clk), .rst_ni(rst_n), .aw_metadata_i(aw_meta), .ar_metadata_i(ar_meta),
    .idx_completed_o(), .axi_req_i(req), .axi_req_o(out_req),
    .axi_resp_i(out_resp), .axi_resp_o(resp));

  function automatic byte pattern(input int b);
    return 8'((b * 37 + b / 251 + tag * 19) ^ (b >> 8));
  endfunction

  // The reference uses the architectural element-to-cluster mapping, not the
  // shuffle network: four consecutive elements per cluster, then repeat.
  function automatic data_t source(input int c, input int beat);
    data_t data;
    int bytes_per_element, local_byte, element, global_byte;
    bytes_per_element = 1 << width;
    for (int b = 0; b < 16; b++) begin
      local_byte = beat * 16 + b;
      element = local_byte / bytes_per_element;
      global_byte = ((element / Lanes) * Lanes * C + c * Lanes +
                     element % Lanes) * bytes_per_element + local_byte % bytes_per_element;
      data[8*b+:8] = pattern(global_byte);
    end
    return data;
  endfunction

  always @(posedge clk) if (rst_n && checking) begin
    for (int c = 0; c < C; c++) begin
      if (stalled[c] && (!out_req[c].w_valid || out_req[c].w !== held[c]))
        $fatal(1, "C=%0d EW=%0d stalled store changed c=%0d", C, width, c);
      stalled[c] = out_req[c].w_valid && !out_resp[c].w_ready;
      held[c] = out_req[c].w;
      if (out_req[c].w_valid && out_resp[c].w_ready) begin
        if (indexed_phase) begin
          if (c != 0 || received[c] != 0 || out_req[c].w.data !== data_t'(tag) ||
              !out_req[c].w.last || out_req[c].w.strb !== strb_t'('1))
            $fatal(1, "C=%0d wrong indexed store c=%0d", C, c);
        end else begin
          if (received[c] >= Beats || out_req[c].w.strb !== strb_t'('1) ||
              (c == 0 && out_req[c].w.last != (received[c] == Beats-1)))
            $fatal(1, "C=%0d EW=%0d wrong store length/strobes c=%0d beat=%0d",
                   C, width, c, received[c]);
          for (int b = 0; b < 16; b++)
            if (out_req[c].w.data[8*b+:8] !== pattern((received[c]*C+c)*16+b))
              $fatal(1, "C=%0d EW=%0d wrong data c=%0d beat=%0d byte=%0d",
                     C, width, c, received[c], b);
        end
        received[c]++;
      end
    end
    if (dut.wr_cnt_q > 16) $fatal(1, "write tracker underflow/overflow");
  end

  task automatic address(input bit indexed_store);
    bit [C-1:0] accepted;
    accepted = '0;
    @(negedge clk);
    for (int c = 0; c < C; c++) begin
      aw_meta[c] = '{vew:indexed_store ? EW64 : width,
        vl:vlen_cluster_t'(indexed_store ? 1 : Beats*16*C/(1 << width)),
        op:indexed_store ? VSXE : VSE, default:'0};
      req[c].aw.addr = 64'h80001000;
      req[c].aw.len = indexed_store ? 0 : Beats-1;
      req[c].aw.size = indexed_store ? 3 : 4;
      req[c].aw.burst = axi_pkg::BURST_INCR;
      req[c].aw_valid = indexed_store ? c == 0 : c != C-1;
    end
    for (int cycle = 0; cycle < 30; cycle++) begin
      if (cycle == 3 && !indexed_store) req[C-1].aw_valid = 1;
      @(posedge clk);
      for (int c = 0; c < C; c++)
        if (req[c].aw_valid && resp[c].aw_ready) accepted[c] = 1;
      @(negedge clk);
      for (int c = 0; c < C; c++) if (accepted[c]) req[c].aw_valid = 0;
      if (indexed_store ? accepted[0] : (&accepted)) return;
    end
    $fatal(1, "C=%0d store address timeout", C);
  endtask

  task automatic run_case(input vew_e ew, input bit backpressure);
    int sent[C];
    width = ew; tag++; indexed_phase = 0; checking = 1; stalled = '0;
    foreach (received[c]) begin received[c] = 0; sent[c] = 0; end
    address(0);
    // Queue the next operation before the preceding W packet drains.
    address(1);
    for (int cycle = 0; cycle < 300; cycle++) begin
      for (int c = 0; c < C; c++) begin
        req[c].w_valid = sent[c] < Beats;
        req[c].w.data = source(c, sent[c]);
        req[c].w.strb = '1;
        req[c].w.last = sent[c] == Beats-1;
        out_resp[c].w_ready = !backpressure || cycle % 11 >= 5;
      end
      @(posedge clk);
      for (int c = 0; c < C; c++)
        if (req[c].w_valid && resp[c].w_ready) sent[c]++;
      @(negedge clk);
      if (received[0] == Beats) break;
    end
    req = '0;
    for (int c = 0; c < C; c++)
      if (sent[c] != Beats || received[c] != Beats)
        $fatal(1, "C=%0d EW=%0d lost tail c=%0d sent=%0d received=%0d",
               C, width, c, sent[c], received[c]);
    indexed_phase = 1;
    foreach (received[c]) received[c] = 0;
    req[0].w_valid = 1; req[0].w.data = data_t'(tag);
    req[0].w.strb = '1; req[0].w.last = 1;
    for (int c = 0; c < C; c++) out_resp[c].w_ready = 1;
    do @(posedge clk); while (!resp[0].w_ready);
    @(negedge clk); req[0].w_valid = 0;
    repeat (5) @(negedge clk);
    if (received[0] != 1 || dut.wr_cnt_q != 0)
      $fatal(1, "C=%0d indexed store did not complete", C);
    checking = 0;
  endtask

  initial begin
    done = 0; rst_n = 0; req = '0; out_resp = '0; ar_meta = '0; aw_meta = '0;
    checking = 0; stalled = '0; tag = 0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    for (int c = 0; c < C; c++) out_resp[c].aw_ready = 1;
    // Reuse tracker entries as well as changing the route of the queued store.
    repeat (3)
      for (int ew = 0; ew < 3; ew++) begin
        run_case(vew_e'(ew), 0);
        run_case(vew_e'(ew), 1);
      end
    $display("PASS store transitions: C=%0d, 18 queued store pairs", C);
    done = 1;
  end
endmodule

module store_transition_tb;
  wire [3:0] done;
  store_transition_case #(.C(2)) c2(done[0]);
  store_transition_case #(.C(4)) c4(done[1]);
  store_transition_case #(.C(8)) c8(done[2]);
  store_transition_case #(.C(16)) c16(done[3]);
  initial begin wait (&done); $finish; end
  initial begin #200000; $fatal(1, "store transition test timeout"); end
endmodule
