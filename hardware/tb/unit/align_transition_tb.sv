// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Check response ordering when buffered unit-stride and indexed loads meet.

`include "axi/typedef.svh"
module align_transition_case #(parameter int C=2, parameter bit STALL=0)(output logic done);
  import ara_pkg::*;
  import rvv_pkg::*;
  localparam int Bytes=16*C;
  localparam int Beats=(127*8+Bytes-1)/Bytes;
  localparam int Total=Beats+4;
  typedef logic [63:0] addr_t;
  typedef logic id_t;
  typedef logic user_t;
  typedef logic [Bytes*8-1:0] data_t;
  typedef logic [Bytes-1:0] strb_t;
  `AXI_TYPEDEF_AW_CHAN_T(aw_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(ar_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_W_CHAN_T(w_t, data_t, strb_t, user_t)
  `AXI_TYPEDEF_B_CHAN_T(b_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(r_t, data_t, id_t, user_t)
  `AXI_TYPEDEF_REQ_T(req_t, aw_t, w_t, ar_t)
  `AXI_TYPEDEF_RESP_T(rsp_t, b_t, r_t)
  logic clk=0, rst_n;
  always #5 clk=~clk;
  req_t req, out_req;
  rsp_t rsp, out_rsp;
  cluster_metadata_t ar_meta, aw_meta;
  int received, sent;
  align_stage #(.NrClusters(C), .AxiDataWidth(Bytes*8), .AxiAddrWidth(64),
      .axi_r_t(r_t), .axi_w_t(w_t), .axi_b_t(b_t),
      .axi_req_t(req_t), .axi_resp_t(rsp_t)) dut (
    .clk_i(clk), .rst_ni(rst_n), .ar_metadata_i(ar_meta), .aw_metadata_i(aw_meta),
    .axi_req_i(req), .axi_req_o(out_req), .axi_resp_i(out_rsp), .axi_resp_o(rsp)
  );
  function automatic data_t payload(input int beat);
    data_t result;
    for (int k=0;k<Bytes/8;k++) result[k*64+:64]=64'(beat*1024+k);
    return result;
  endfunction
  always @(posedge clk) if (rst_n) begin
    if (out_rsp.r_valid && out_req.r_ready) sent++;
    if (rsp.r_valid && req.r_ready) begin
      if (rsp.r.data !== payload(received))
        $fatal(1,"C=%0d stall=%0d lost/reordered response %0d: got %h expected %h",
               C, STALL, received, rsp.r.data, payload(received));
      if (rsp.r.last !== (received>=Beats-1))
        $fatal(1,"C=%0d wrong last on response %0d",C,received);
      received++;
    end
  end
  task automatic address(input ara_op_e op, input vew_e width, input int vl, input int beats);
    @(negedge clk);
    ar_meta='{vew:width, vl:vlen_cluster_t'(vl), use_eew1:0, op:op, default:'0};
    req.ar_valid=1;
    req.ar.addr=64'h80001000;
    req.ar.len=8'(beats-1);
    do @(posedge clk); while (!rsp.ar_ready);
    @(negedge clk);
    req.ar_valid=0;
  endtask
  initial begin
    done=0; rst_n=0; req='0; out_rsp='0; ar_meta='0; aw_meta='0; received=0; sent=0;
    repeat(3) @(negedge clk);
    rst_n=1;
    req.r_ready=1;
    out_rsp.ar_ready=1;
    address(VLE, EW64, 127, Beats);
    address(VLXE, EW64, 1, 1);
    address(VLXE, EW16, 1, 1);
    address(VLE, EW32, Bytes/4, 1);
    address(VLXE, EW8, 1, 1);
    fork
      begin
        for(int i=0;i<Total;i++) begin
          @(negedge clk);
          out_rsp.r_valid=1;
          out_rsp.r.data=payload(i);
          out_rsp.r.last=(i>=Beats-1);
          do @(posedge clk); while(!out_req.r_ready);
        end
        @(negedge clk);
        out_rsp.r_valid=0;
      end
      begin
        if(STALL) begin
          wait(received>=Beats-2);
          @(negedge clk); req.r_ready=0;
          repeat(4) @(negedge clk);
          req.r_ready=1;
          wait(received>=Beats+1);
          @(negedge clk); req.r_ready=0;
          repeat(3) @(negedge clk);
          req.r_ready=1;
        end
      end
    join
    wait(received==Total);
    repeat(3) @(negedge clk);
    if(sent!=Total || received!=Total) $fatal(1,"response count mismatch");
    $display("PASS align transition: C=%0d stall=%0d",C,STALL);
    done=1;
  end
endmodule
module align_transition_tb;
  wire [4:0] done;
  align_transition_case #(.C(2)) c2(done[0]);
  align_transition_case #(.C(2),.STALL(1)) c2_stall(done[1]);
  align_transition_case #(.C(4)) c4(done[2]);
  align_transition_case #(.C(8)) c8(done[3]);
  align_transition_case #(.C(16)) c16(done[4]);
  initial begin wait(&done); $finish; end
  initial begin #20000; $fatal(1,"timeout"); end
endmodule
