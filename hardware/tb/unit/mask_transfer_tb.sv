// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Mask memory requests must use the original VL to identify participating
// clusters, even when several VL values round to the same memory byte count.
`include "axi/typedef.svh"
module mask_transfer_case #(parameter int C = 2)(output logic done);
  import ara_pkg::*;
  import rvv_pkg::*;
  localparam int Lanes = 4;
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
  int lengths[24] = '{1, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32,
                       33, 63, 64, 65, 127, 128, 129, 191, 192, 193, 254, 255};

  shuffle_stage #(.NrLanes(Lanes), .NrClusters(C), .ClusterAxiDataWidth(128),
    .AxiAddrWidth(64), .axi_r_t(r_t), .axi_w_t(w_t),
    .axi_req_t(req_t), .axi_resp_t(resp_t)) dut (
    .clk_i(clk), .rst_ni(rst_n), .aw_metadata_i(aw_meta), .ar_metadata_i(ar_meta),
    .idx_completed_o(), .axi_req_i(req), .axi_req_o(out_req),
    .axi_resp_i(out_resp), .axi_resp_o(resp));

  // Independent reference: count actual elements assigned to each cluster.
  function automatic int local_bits(input int vl, input int cluster);
    int count = 0;
    for (int i = 0; i < vl; i++)
      if ((i / Lanes) % C == cluster) count++;
    return count;
  endfunction
  function automatic bit pattern(input int index);
    return ((index * 13 + index / 3 + 7) % 11) < 5;
  endfunction

  task automatic address(input int vl, input bit store);
    bit [C-1:0] accepted, active, stalled;
    aw_t [C-1:0] held_aw;
    ar_t [C-1:0] held_ar;
    int forwarded, last;
    accepted = '0; active = '0; stalled = '0;
    held_aw = '0; held_ar = '0; forwarded = 0; last = 0;
    @(negedge clk);
    for (int c = 0; c < C; c++) begin
      active[c] = local_bits(vl, c) != 0;
      if (active[c]) last = c;
      // Inactive clusters deliberately present the next instruction's address.
      // It must remain queued while this shared request is consumed.
      if (store) begin
        aw_meta[c] = '{vew:EW8, vl:vlen_cluster_t'((vl + 7) / 8),
                       mask_vl:vlen_cluster_t'(vl), use_eew1:1, op:VSE, default:'0};
        req[c].aw.addr = active[c] ? 64'h80001000 : 64'h80002000;
        if (!active[c]) aw_meta[c].mask_vl = vlen_cluster_t'(vl + 1);
      end else begin
        ar_meta[c] = '{vew:EW8, vl:vlen_cluster_t'((vl + 7) / 8),
                       mask_vl:vlen_cluster_t'(vl), use_eew1:1, op:VLE, default:'0};
        req[c].ar.addr = active[c] ? 64'h80001000 : 64'h80002000;
        if (!active[c]) ar_meta[c].mask_vl = vlen_cluster_t'(vl + 1);
      end
    end
    for (int cycle = 0; cycle < 30; cycle++) begin
      for (int c = 0; c < C; c++) begin
        if (store) begin
          req[c].aw_valid = !accepted[c] && (c != last || cycle >= 3);
          out_resp[c].aw_ready = cycle >= 6;
        end else begin
          req[c].ar_valid = !accepted[c] && (c != last || cycle >= 3);
          out_resp[c].ar_ready = cycle >= 6;
        end
      end
      @(posedge clk);
      for (int c = 0; c < C; c++) begin
        if (stalled[c] && (store ?
            (!out_req[c].aw_valid || out_req[c].aw !== held_aw[c]) :
            (!out_req[c].ar_valid || out_req[c].ar !== held_ar[c])))
          $fatal(1, "C=%0d VL=%0d stalled address changed c=%0d", C, vl, c);
        stalled[c] = store ? (out_req[c].aw_valid && !out_resp[c].aw_ready) :
                             (out_req[c].ar_valid && !out_resp[c].ar_ready);
        held_aw[c] = out_req[c].aw;
        held_ar[c] = out_req[c].ar;
        if (store ? (req[c].aw_valid && resp[c].aw_ready) :
                    (req[c].ar_valid && resp[c].ar_ready)) begin
          if (!active[c] || accepted[c])
            $fatal(1, "C=%0d VL=%0d consumed unrelated/duplicate address c=%0d", C, vl, c);
          accepted[c] = 1;
        end
        if (store ? (out_req[c].aw_valid && out_resp[c].aw_ready) :
                    (out_req[c].ar_valid && out_resp[c].ar_ready)) begin
          if (c != 0 || cycle < 6)
            $fatal(1, "C=%0d VL=%0d premature/redundant shared address", C, vl);
          forwarded++;
        end
      end
      @(negedge clk);
      if ((accepted & active) == active) break;
    end
    if ((accepted & active) != active || forwarded != 1)
      $fatal(1, "C=%0d VL=%0d store=%0d shared address blocked/count=%0d", C, vl, store, forwarded);
    for (int c = 0; c < C; c++) begin
      req[c].ar_valid = 0;
      req[c].aw_valid = 0;
    end
  endtask

  task automatic load_data(input int vl);
    bit [C-1:0] received, stalled;
    r_t [C-1:0] held;
    int memory_beats;
    received = '0; stalled = '0; held = '0; memory_beats = 0;
    for (int c = 0; c < C; c++) begin
      out_resp[c].r_valid = 1;
      out_resp[c].r.last = 1;
      for (int b = 0; b < 128; b++) out_resp[c].r.data[b] = pattern(c * 128 + b);
    end
    for (int cycle = 0; cycle < 50; cycle++) begin
      for (int c = 0; c < C; c++)
        req[c].r_ready = local_bits(vl, c) != 0 && cycle >= (c == 0 ? 10 : 5);
      @(posedge clk);
      for (int c = 0; c < C; c++) begin
        if ((out_resp[c].r_valid && out_req[c].r_ready) !=
            (out_resp[0].r_valid && out_req[0].r_ready))
          $fatal(1, "C=%0d VL=%0d split memory R handshake c=%0d", C, vl, c);
        if (stalled[c] && (!resp[c].r_valid || resp[c].r !== held[c]))
          $fatal(1, "C=%0d VL=%0d stalled R payload changed c=%0d", C, vl, c);
        if (resp[c].r_valid && local_bits(vl, c) == 0)
          $fatal(1, "C=%0d VL=%0d RVALID for inactive cluster %0d", C, vl, c);
        if (resp[c].r_valid && req[c].r_ready) begin
          if (received[c] || !resp[c].r.last)
            $fatal(1, "C=%0d VL=%0d duplicate/non-final R c=%0d", C, vl, c);
          for (int b = 0; b < local_bits(vl, c); b++) begin
            if (resp[c].r.data[b] !== pattern((b / Lanes) * Lanes * C + c * Lanes + b % Lanes))
              $fatal(1, "C=%0d VL=%0d incorrect mask bit %0d c=%0d", C, vl, b, c);
          end
          received[c] = 1;
        end
        stalled[c] = resp[c].r_valid && !req[c].r_ready;
        held[c] = resp[c].r;
      end
      if (out_resp[0].r_valid && out_req[0].r_ready) memory_beats++;
      @(negedge clk);
      if (memory_beats != 0)
        for (int c = 0; c < C; c++) out_resp[c].r_valid = 0;
      if (dut.rd_cnt_q == 0 && memory_beats == 1) break;
    end
    for (int c = 0; c < C; c++)
      if (received[c] != (local_bits(vl, c) != 0))
        $fatal(1, "C=%0d VL=%0d missing R c=%0d", C, vl, c);
    if (memory_beats != 1 || dut.rd_cnt_q != 0)
      $fatal(1, "C=%0d VL=%0d read conservation/tracker failure", C, vl);
  endtask

  task automatic store_data(input int vl);
    bit [C-1:0] accepted, stalled;
    w_t [C-1:0] held;
    int memory_beats;
    accepted = '0; stalled = '0; held = '0; memory_beats = 0;
    for (int c = 0; c < C; c++) begin
      req[c].w = '0;
      req[c].w_valid = local_bits(vl, c) != 0;
      req[c].w.last = 1;
      req[c].w.strb = strb_t'((1 << ((local_bits(vl, c) + 7) / 8)) - 1);
      for (int b = 0; b < local_bits(vl, c); b++)
        req[c].w.data[b] = pattern((b / Lanes) * Lanes * C + c * Lanes + b % Lanes);
    end
    for (int cycle = 0; cycle < 50; cycle++) begin
      for (int c = 0; c < C; c++) out_resp[c].w_ready = cycle >= 8;
      @(posedge clk);
      for (int c = 0; c < C; c++) begin
        if (stalled[c] && (!out_req[c].w_valid || out_req[c].w !== held[c]))
          $fatal(1, "C=%0d VL=%0d stalled W payload changed c=%0d", C, vl, c);
        stalled[c] = out_req[c].w_valid && !out_resp[c].w_ready;
        held[c] = out_req[c].w;
        if ((out_req[c].w_valid && out_resp[c].w_ready) !=
            (out_req[0].w_valid && out_resp[0].w_ready))
          $fatal(1, "C=%0d VL=%0d split memory W handshake c=%0d", C, vl, c);
        if (req[c].w_valid && resp[c].w_ready) begin
          if (accepted[c]) $fatal(1, "duplicate W c=%0d", c);
          accepted[c] = 1;
        end
      end
      if (out_req[0].w_valid && out_resp[0].w_ready) begin
        memory_beats++;
        for (int b = 0; b < vl; b++)
          if (out_req[b / 128].w.data[b % 128] !== pattern(b))
            $fatal(1, "C=%0d VL=%0d incorrect stored mask bit %0d", C, vl, b);
      end
      @(negedge clk);
      for (int c = 0; c < C; c++) if (accepted[c]) req[c].w_valid = 0;
      if (dut.wr_cnt_q == 0 && memory_beats == 1) break;
    end
    for (int c = 0; c < C; c++)
      if (accepted[c] != (local_bits(vl, c) != 0))
        $fatal(1, "C=%0d VL=%0d missing W c=%0d", C, vl, c);
    if (memory_beats != 1 || dut.wr_cnt_q != 0)
      $fatal(1, "C=%0d VL=%0d write conservation/tracker failure", C, vl);
  endtask

  always @(posedge clk) if (rst_n && (dut.rd_cnt_q > 16 || dut.wr_cnt_q > 16))
    $fatal(1, "C=%0d tracker occupancy out of bounds", C);
  initial begin
    done = 0; rst_n = 0; req = '0; out_resp = '0; ar_meta = '0; aw_meta = '0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    foreach (lengths[i]) begin
      address(lengths[i], 0);
      load_data(lengths[i]);
      address(lengths[i], 1);
      store_data(lengths[i]);
    end
    $display("PASS mask transfers: C=%0d, 24 VLs, loads and stores", C);
    done = 1;
  end
endmodule

module mask_transfer_tb;
  wire [3:0] done;
  mask_transfer_case #(.C(2)) c2(done[0]);
  mask_transfer_case #(.C(4)) c4(done[1]);
  mask_transfer_case #(.C(8)) c8(done[2]);
  mask_transfer_case #(.C(16)) c16(done[3]);
  initial begin wait (&done); $finish; end
  initial begin #100000; $fatal(1, "mask transfer test timeout"); end
endmodule
