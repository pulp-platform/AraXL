// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// All clusters must make the same destination-reshuffle decision when a
// partial cluster round fills one local register but leaves another partial.
module reshuffle_dispatcher_tb;
  import ara_pkg::*;
  import rvv_pkg::*;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;
  num_cluster_t log_clusters;
  id_cluster_t cluster_id;
  accelerator_req_t req;
  accelerator_resp_t resp;
  ara_req_t backend_req;
  logic backend_valid, backend_resp_valid;

  ara_dispatcher #(.NrLanes(4)) dut (
    .clk_i(clk), .rst_ni(rst_n), .num_clusters_i(log_clusters),
    .cluster_id_i(cluster_id), .acc_req_i(req), .acc_resp_o(resp),
    .ara_req_o(backend_req), .ara_req_valid_o(backend_valid),
    .ara_req_ready_i(1'b1), .ara_resp_i('0), .ara_resp_valid_i(backend_resp_valid),
    .ara_idle_i(1'b1), .fflags_ex_i('0), .fflags_ex_valid_i('0),
    .vxsat_flag_i('0), .alu_vxrm_o(), .core_st_pending_o(),
    .load_complete_i(1'b0), .store_complete_i(1'b0),
    .rs_load_complete_i(1'b0), .store_pending_i(1'b0));

  function automatic logic [31:0] vle(input vew_e width);
    logic [31:0] funct3;
    funct3 = width == EW8 ? 0 : 32'(width) + 4;
    // vm=1, rs1=x1, vd=v16, unit-stride load.
    return (32'b1 << 25) | (32'b1 << 15) | (funct3 << 12) | (16 << 7) | 7;
  endfunction

  task automatic configure(input int vl, input vew_e width);
    @(negedge clk);
    // vsetvli x5, x1, e<width>, m8, ta, ma.
    req.insn.instr = ((32'hc3 | (32'(width) << 3)) << 20) |
                     (32'b1 << 15) | (7 << 12) | (5 << 7) | 32'h57;
    req.rs1 = 64'(vl);
    req.req_valid = 1;
    req.resp_ready = 1;
    #1;
    if (!resp.req_ready || resp.error || resp.result != 64'(vl))
      $fatal(1, "Configuration failed VL=%0d EW=%0d", vl, width);
    @(posedge clk);
    @(negedge clk);
    req.req_valid = 0;
  endtask

  task automatic check_case(input int clusters, input int cluster,
                            input vew_e width, input int vl);
    vew_e old_width;
    bit expect_reshuffle;
    old_width = width == EW8 ? EW64 : EW8;
    expect_reshuffle = vl != (VLENB >> width) * clusters;
    rst_n = 0;
    req = '0;
    backend_resp_valid = 0;
    log_clusters = num_cluster_t'($clog2(clusters));
    cluster_id = id_cluster_t'(cluster);
    repeat (2) @(negedge clk);
    rst_n = 1;

    // Seed the register's EEW through real decoded instructions. Every cluster
    // gets four elements; no forced internal dispatcher state is needed.
    configure(4 * clusters, old_width);
    req.insn.instr = vle(old_width);
    req.req_valid = 1;
    #1;
    if (resp.error) $fatal(1, "Seed load rejected");
    @(posedge clk);
    @(negedge clk);
    backend_resp_valid = 1;
    #1;
    if (!resp.req_ready || resp.error) $fatal(1, "Seed load not acknowledged");
    @(posedge clk);
    @(negedge clk);
    backend_resp_valid = 0;
    req.req_valid = 0;

    configure(vl, width);
    req.insn.instr = vle(width);
    req.req_valid = 1;
    #1;
    if (resp.error || dut.reshuffle_req_d[0] != expect_reshuffle)
      $fatal(1, "C=%0d cluster=%0d EW=%0d VL=%0d: expected reshuffle=%0d, got=%0d",
             clusters, cluster, width, vl, expect_reshuffle, dut.reshuffle_req_d[0]);
    // Observe the actual backend operation, including the injected store.
    repeat (2) @(posedge clk);
    @(negedge clk);
    if (!backend_valid || backend_req.is_reshuffle != expect_reshuffle ||
        backend_req.op != (expect_reshuffle ? VSE : VLE))
      $fatal(1, "Incorrect backend operation C=%0d cluster=%0d EW=%0d VL=%0d",
             clusters, cluster, width, vl);
  endtask

  initial begin
    int cases;
    int lengths[13];
    cases = 0;
    req = '0;
    backend_resp_valid = 0;
    log_clusters = '0;
    cluster_id = '0;
    check_case(2, 0, EW64, 127);
    cases++;
    for (int clusters = 2; clusters <= 16; clusters *= 2) begin
      for (int ew = 0; ew < 4; ew++) begin
        lengths = '{1, 3, 4, 5, 4*clusters-1, 4*clusters, 4*clusters+1,
                    (VLENB >> ew)*clusters-1, (VLENB >> ew)*clusters,
                    (VLENB >> ew)*clusters+1, 127, 254, 255};
        foreach (lengths[v]) begin
          for (int c = 0; c < clusters; c++) begin
            check_case(clusters, c, vew_e'(ew), lengths[v]);
            cases++;
          end
        end
      end
      $display("RESHUFFLE DISPATCHER PASS C=%0d", clusters);
    end
    $display("RESHUFFLE DISPATCHER REGRESSION PASS (%0d cases)", cases);
    $finish;
  end

  initial begin
    #1000000;
    $fatal(1, "Reshuffle dispatcher regression timed out");
  end
endmodule
