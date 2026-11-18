// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Check every cache-line invalidation, including across independent AXI bursts.

`include "axi/typedef.svh"

module axi_inval_filter_case #(
  parameter int NrClusters = 2,
  parameter int NumSets = 64
) (output logic done);
  localparam int BusBytes = 16 * NrClusters;
  // AxSIZE has three bits; use legal narrow transfers on the 256-byte bus.
  localparam int BeatBytes = BusBytes > 128 ? 128 : BusBytes;
  localparam int LineBytes = 32;
  typedef logic [63:0] addr_t;
  typedef logic [BusBytes*8-1:0] data_t;
  typedef logic [BusBytes-1:0] strb_t;
  typedef logic id_t;
  typedef logic user_t;
  `AXI_TYPEDEF_AW_CHAN_T(aw_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_t, data_t, strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_t, data_t, id_t, user_t)
  `AXI_TYPEDEF_REQ_T(req_t, aw_t, w_t, ar_t)
  `AXI_TYPEDEF_RESP_T(resp_t, b_t, r_t)

  logic clk = 0;
  always #5 clk = ~clk;
  logic rst_n, enable, inval_valid, inval_ready;
  addr_t inval_addr;
  req_t slv_req, mst_req;
  resp_t slv_resp, mst_resp;
  addr_t expected[$];
  int aw_count, inv_count;
  bit stalled;
  addr_t stalled_addr;

  axi_inval_filter #(
    .MaxTxns(4), .AddrWidth(64), .L1LineWidth(LineBytes), .NumSets(NumSets),
    .aw_chan_t(aw_t), .req_t(req_t), .resp_t(resp_t)
  ) dut (
    .clk_i(clk), .rst_ni(rst_n), .en_i(enable),
    .slv_req_i(slv_req), .slv_resp_o(slv_resp),
    .mst_req_o(mst_req), .mst_resp_i(mst_resp),
    .inval_addr_o(inval_addr), .inval_valid_o(inval_valid),
    .inval_ready_i(inval_ready)
  );

  always @(posedge clk) if (rst_n) begin
    if (slv_req.aw_valid && slv_resp.aw_ready) begin
      if (!mst_req.aw_valid || !mst_resp.aw_ready || mst_req.aw !== slv_req.aw)
        $fatal(1, "C=%0d AW was not forwarded exactly once", NrClusters);
      aw_count++;
      if (enable) begin
        automatic int bytes = (int'(slv_req.aw.len) + 1) << slv_req.aw.size;
        automatic int offset = int'(slv_req.aw.addr % LineBytes);
        automatic int lines = (offset + bytes + LineBytes - 1) / LineBytes;
        // Invalidating every set covers the whole cache, but only for this burst.
        if (lines > NumSets) lines = NumSets;
        for (int i = 0; i < lines; i++) begin
          expected.push_back(i == 0 ? slv_req.aw.addr :
              slv_req.aw.addr - addr_t'(offset) + addr_t'(i * LineBytes));
        end
      end
    end
    if (stalled && (!inval_valid || inval_addr !== stalled_addr))
      $fatal(1, "C=%0d invalidation changed while stalled", NrClusters);
    if (inval_valid && inval_ready) begin
      addr_t next_addr;
      if (expected.size() == 0)
        $fatal(1, "C=%0d unexpected invalidation at %h", NrClusters, inval_addr);
      next_addr = expected.pop_front();
      if (inval_addr !== next_addr)
        $fatal(1, "C=%0d sets=%0d invalidation %0d: got %h expected %h",
               NrClusters, NumSets, inv_count, inval_addr, next_addr);
      inv_count++;
    end
    stalled = inval_valid && !inval_ready;
    stalled_addr = inval_addr;
  end

  task automatic address(input addr_t addr, input int beats, input int size);
    @(negedge clk);
    slv_req.aw_valid = 1;
    slv_req.aw = '0;
    slv_req.aw.addr = addr;
    slv_req.aw.len = 8'(beats - 1);
    slv_req.aw.size = 3'(size);
    slv_req.aw.burst = axi_pkg::BURST_INCR;
    do @(posedge clk); while (!slv_resp.aw_ready);
    @(negedge clk);
    slv_req.aw_valid = 0;
  endtask

  task automatic drain;
    // Also detect missing tail invalidations when no later AW exposes the gap.
    repeat (300) @(negedge clk);
    if (expected.size() != 0 || inval_valid)
      $fatal(1, "C=%0d sets=%0d missing %0d invalidations",
             NrClusters, NumSets, expected.size());
  endtask

  initial begin
    done = 0; rst_n = 0; enable = 1; inval_ready = 1;
    slv_req = '0; mst_resp = '0; aw_count = 0; inv_count = 0; stalled = 0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    mst_resp.aw_ready = 1;

    // Reproduce the application: disjoint stores leave the old counter at 62,
    // then VL=65/EW64 needs 17 cache lines on the two-cluster configuration.
    // Reusing one set also proves transaction count is not unique-set coverage.
    for (int i = 0; i < 62; i++) address(64'h80001000, 1, 3);
    address(64'h80007680, (65*8 + BeatBytes-1) / BeatBytes, $clog2(BeatBytes));
    drain();

    // An unaligned short burst can cross a line; a large burst may cover every
    // set. Neither case permits skipping invalidations from the following AW.
    address(64'h80002018, 2, 3);
    address(64'h80004000, 128, $clog2(BeatBytes));
    address(64'h80009000, 3, $clog2(BeatBytes));
    drain();

    // Fill the AW FIFO while invalidations are blocked, then drain queued,
    // unrelated bursts with intermittent stalls in the invalidation interface.
    inval_ready = 0;
    fork
      begin
        for (int i = 0; i < 9; i++)
          address(64'h80010000 + addr_t'(i*4096), 5, $clog2(BeatBytes));
      end
      begin
        repeat (16) @(negedge clk);
        repeat (240) begin
          inval_ready = 1;
          repeat (3) @(negedge clk);
          inval_ready = 0;
          repeat (2) @(negedge clk);
        end
        inval_ready = 1;
      end
    join
    drain();

    // Disabled coherence forwards addresses without generating invalidations.
    enable = 0;
    address(64'h80020000, 2, $clog2(BeatBytes));
    drain();
    if (aw_count != 76) $fatal(1, "wrong accepted AW count: %0d", aw_count);
    $display("PASS AXI invalidation: C=%0d sets=%0d AW=%0d invalidations=%0d",
             NrClusters, NumSets, aw_count, inv_count);
    done = 1;
  end
endmodule

module axi_inval_filter_tb;
  wire [3:0] done;
  axi_inval_filter_case #(.NrClusters(2)) c2(done[0]);
  axi_inval_filter_case #(.NrClusters(4)) c4(done[1]);
  axi_inval_filter_case #(.NrClusters(8)) c8(done[2]);
  axi_inval_filter_case #(.NrClusters(16)) c16(done[3]);
  initial begin wait (&done); $finish; end
  initial begin #50000; $fatal(1, "invalidation regression timeout"); end
endmodule
