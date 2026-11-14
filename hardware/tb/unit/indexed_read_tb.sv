// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Check indexed response conservation and routing while the selected cluster stalls.

`include "axi/typedef.svh"
module indexed_read_case #(parameter int C = 2)(output logic done);
  import ara_pkg::*;
  import rvv_pkg::*;
  localparam int Lanes = 4;
  localparam int Total = Lanes*C + 6;
  typedef logic [63:0] addr_t;
  typedef logic id_t;
  typedef logic user_t;
  typedef logic [127:0] c_data_t;
  typedef logic [15:0] c_strb_t;
  typedef logic [128*C-1:0] m_data_t;
  typedef logic [16*C-1:0] m_strb_t;
  `AXI_TYPEDEF_AW_CHAN_T(aw_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(cw_t, c_data_t, c_strb_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(mw_t, m_data_t, m_strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(cr_t, c_data_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(mr_t, m_data_t, id_t, user_t)
  `AXI_TYPEDEF_REQ_T(c_req_t, aw_t, cw_t, ar_t)
  `AXI_TYPEDEF_REQ_T(m_req_t, aw_t, mw_t, ar_t)
  `AXI_TYPEDEF_RESP_T(c_resp_t, b_t, cr_t)
  `AXI_TYPEDEF_RESP_T(m_resp_t, b_t, mr_t)

  logic clk = 0;
  always #5 clk = ~clk;
  logic rst_n, single_ready;
  c_req_t [C-1:0] req;
  c_resp_t [C-1:0] resp;
  m_req_t mem_req;
  m_resp_t mem_resp;
  cluster_metadata_t [C-1:0] ar_meta, aw_meta;
  cluster_metadata_t mem_ar_meta, mem_aw_meta;
  int addresses, received, sent;
  bit [C-1:0] stalled;
  cr_t [C-1:0] held;

  global_ldst #(
    .NrLanes(Lanes), .NrClusters(C), .AxiDataWidth(128*C),
    .ClusterAxiDataWidth(128), .AxiAddrWidth(64),
    .cluster_axi_req_t(c_req_t), .cluster_axi_resp_t(c_resp_t),
    .axi_req_t(m_req_t), .axi_resp_t(m_resp_t)
  ) dut (
    .clk_i(clk), .rst_ni(rst_n), .ar_metadata_i(ar_meta), .aw_metadata_i(aw_meta),
    .ar_metadata_o(mem_ar_meta), .aw_metadata_o(mem_aw_meta),
    .axi_req_i(req), .axi_resp_o(resp), .axi_req_o(mem_req), .axi_resp_i(mem_resp)
  );

  function automatic c_data_t payload(input int beat);
    return {64'habcdef0123456789, 64'(beat + 100)};
  endfunction

  always @(posedge clk) if (rst_n) begin
    int accepted;
    accepted = 0;
    if (mem_req.ar_valid && mem_resp.ar_ready) addresses++;
    for (int c = 0; c < C; c++) begin
      if (stalled[c] && (!resp[c].r_valid || resp[c].r !== held[c]))
        $fatal(1, "C=%0d response changed while cluster %0d stalled", C, c);
      if (resp[c].r_valid && req[c].r_ready) begin
        if (c != (received / Lanes) % C || resp[c].r.data !== payload(received) ||
            !resp[c].r.last)
          $fatal(1, "C=%0d response %0d routed to cluster %0d with data %h",
                 C, received, c, resp[c].r.data);
        received++;
        accepted++;
      end
      stalled[c] = resp[c].r_valid && !req[c].r_ready;
      held[c] = resp[c].r;
    end
    if (accepted != int'(mem_resp.r_valid && mem_req.r_ready))
      $fatal(1, "C=%0d memory/cluster response handshake mismatch", C);
    if (mem_resp.r_valid && mem_req.r_ready) sent++;
  end

  // Inactive clusters need not be ready for an indexed response to progress.
  always @(negedge clk) if (rst_n && single_ready)
    for (int c = 0; c < C; c++) req[c].r_ready = c == (received / Lanes) % C;

  initial begin
    done = 0; rst_n = 0; single_ready = 0;
    req = '0; mem_resp = '0; ar_meta = '0; aw_meta = '0;
    addresses = 0; received = 0; sent = 0; stalled = '0; held = '0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    mem_resp.ar_ready = 1;
    for (int c = 0; c < C; c++)
      ar_meta[c] = '{vew:EW32, vl:vlen_cluster_t'(Total), use_eew1:0, op:VLXE, default:'0};

    fork
      begin
        for (int i = 0; i < Total; i++) begin
          automatic int c = (i / Lanes) % C;
          @(negedge clk);
          req[c].ar_valid = 1;
          req[c].ar.addr = 64'h80001000 + addr_t'(i*4);
          req[c].ar.size = 2;
          req[c].ar.burst = axi_pkg::BURST_INCR;
          do @(posedge clk); while (!resp[c].ar_ready);
          @(negedge clk);
          req[c].ar_valid = 0;
        end
      end
      begin
        for (int i = 0; i < Total; i++) begin
          wait (addresses > i);
          @(negedge clk);
          mem_resp.r_valid = 1;
          mem_resp.r.data = m_data_t'(payload(i));
          mem_resp.r.last = 1;
          do @(posedge clk); while (!mem_req.r_ready);
          @(negedge clk);
          mem_resp.r_valid = 0;
        end
      end
      begin
        // Reproduce held RVALID at the GLSU's input. The old RTL advances its
        // element and cluster counters during these cycles without a handshake.
        wait (mem_resp.r_valid);
        repeat (4) @(negedge clk);
        for (int c = 0; c < C; c++) req[c].r_ready = 1;
        wait (received >= 6);
        @(negedge clk);
        for (int c = 0; c < C; c++) req[c].r_ready = 0;
        repeat (3) @(negedge clk);
        for (int c = 0; c < C; c++) req[c].r_ready = 1;
        wait (received >= 10);
        @(negedge clk);
        single_ready = 1;
      end
    join
    repeat (5) @(negedge clk);
    if (addresses != Total || sent != Total || received != Total || !dut.fifo_empty_o)
      $fatal(1, "C=%0d incomplete indexed read: AR=%0d memory R=%0d cluster R=%0d",
             C, addresses, sent, received);
    $display("PASS indexed read backpressure: C=%0d responses=%0d", C, received);
    done = 1;
  end
endmodule

module indexed_read_tb;
  wire [3:0] done;
  indexed_read_case #(.C(2)) c2(done[0]);
  indexed_read_case #(.C(4)) c4(done[1]);
  indexed_read_case #(.C(8)) c8(done[2]);
  indexed_read_case #(.C(16)) c16(done[3]);
  initial begin wait (&done); $finish; end
  initial begin #30000; $fatal(1, "indexed read regression timeout"); end
endmodule
