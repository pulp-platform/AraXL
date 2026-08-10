// Copyright 2021-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// Top level testbench module.

import "DPI-C" function void read_elf (input string filename);
import "DPI-C" function byte get_section (output longint address, output longint len);
import "DPI-C" context function byte read_section(input longint address, inout byte buffer[]);

`define STRINGIFY(x) `"x`"

module ara_tb;

  /*****************
   *  Definitions  *
   *****************/

  `ifndef VERILATOR
  timeunit      1ns;
  timeprecision 1ps;
  `endif

  `ifdef NR_CORES
  localparam NrCores = `NR_CORES;
  `else
  localparam NrCores = 0;
  `endif

  `ifdef NR_LANES
  localparam NrLanes = `NR_LANES;
  `else
  localparam NrLanes = 0;
  `endif

  `ifdef NR_CLUSTERS
  localparam NrClusters = `NR_CLUSTERS;
  `else
  localparam NrClusters = 0;
  `endif

  `ifdef NR_L2_BANKS
  localparam NrL2Banks = `NR_L2_BANKS;
  `else
  localparam NrL2Banks = (NrCores > 0) ? NrCores : 1;
  `endif
  // Must track ara_soc's L2NumWords parameter: the ELF loader below mirrors
  // tc_sram's index truncation, so if the two drift apart the aliasing
  // warning stops matching what the hardware actually does.
  localparam L2NumWords     = 2**20;
  localparam L2BankIdxWidth = $clog2(NrL2Banks);

  localparam ClockPeriod  = 1ns;
  // Axi response delay [ps]
  localparam int unsigned AxiRespDelay = 200;

  localparam AxiAddrWidth          = 64;
  localparam AxiWideDataWidth      = 32 * NrLanes * NrClusters;
  localparam ClusterAxiDataWidth   = 32 * NrLanes;
  localparam AxiWideBeWidth    = AxiWideDataWidth / 8;
  localparam AxiWideByteOffset = $clog2(AxiWideBeWidth);

  localparam DRAMAddrBase = 64'h8000_0000;
  localparam DRAMLength   = 64'h4000_0000; // 1GByte of DDR (split between two chips on Genesys2)

  /********************************
   *  Clock and Reset Generation  *
   ********************************/

  logic clk;
  logic rst_n;

  // Controlling the reset
  initial begin
    clk   = 1'b0;
    rst_n = 1'b0;

    // Synch reset for TB memories
    repeat (10) #(ClockPeriod/2) clk = ~clk;
    clk = 1'b0;

    // Asynch reset for main system
    repeat (5) #(ClockPeriod);
    rst_n = 1'b1;
    repeat (5) #(ClockPeriod);

    // Start the clock
    forever #(ClockPeriod/2) clk = ~clk;
  end

  /*********
   *  DUT  *
   *********/

  logic [63:0] exit;

  // This TB must be implemented in C for integration with Verilator.
  // In order to Verilator to understand that the ara_testharness module is the top-level,
  // we do not instantiate it when Verilating this module.
  `ifndef VERILATOR
  ara_testharness #(
    .NrCores     (NrCores         ),
    .NrL2Banks   (NrL2Banks       ),
    .NrLanes     (NrLanes         ),
    .NrClusters  (NrClusters      ),
    .AxiAddrWidth(AxiAddrWidth    ),
    .AxiDataWidth(AxiWideDataWidth),
    .ClusterAxiDataWidth(ClusterAxiDataWidth),
    .AxiRespDelay(AxiRespDelay    )
  ) dut (
    .clk_i (clk  ),
    .rst_ni(rst_n),
    .exit_o(exit )
  );
  `endif

  /*************************
   *  DRAM Initialization  *
   *************************/

  typedef logic [AxiAddrWidth-1:0] addr_t;
  typedef logic [AxiWideDataWidth-1:0] data_t;

  // The ELF image, keyed by the ABSOLUTE byte address of each AxiWideBeWidth
  // word. Collected here rather than poked directly because the bank a word
  // belongs to is only known at run time, while a generate-block index must be
  // an elaboration-time constant -- see gen_l2_preload below.
  data_t mem_image [longint unsigned];
  // Sequences the ELF reader against the per-bank preloaders: initial blocks
  // have no defined order at time 0, and iterating an empty mem_image would
  // preload nothing without complaining. Level-sensitive `wait`, not an event,
  // so a preloader scheduled after the reader still proceeds instead of hanging.
  bit image_ready = 1'b0;

  initial begin : dram_init
    automatic data_t mem_row;
    byte buffer [];
    addr_t address;
    addr_t length;
    string binary;

    // tc_sram is initialized with zeros. We need to overwrite this value.
    repeat (2)
      #ClockPeriod;

    // Initialize memories
    void'($value$plusargs("PRELOAD=%s", binary));
    if (binary != "") begin
      // Read ELF
      read_elf(binary);
      $display("Loading ELF file %s", binary);
      while (get_section(address, length)) begin
        // Read sections
        automatic int nwords = (length + AxiWideBeWidth - 1)/AxiWideBeWidth;
        $display("Loading section %x of length %x", address, length);
        buffer = new[nwords * AxiWideBeWidth];
        void'(read_section(address, buffer));
        // Collect the section into the image
        for (int w = 0; w < nwords; w++) begin
          automatic longint unsigned byte_addr = address + (w << AxiWideByteOffset);
          mem_row = '0;
          for (int b = 0; b < AxiWideBeWidth; b++) begin
            mem_row[8 * b +: 8] = buffer[w * AxiWideBeWidth + b];
          end
          // This requires the sections to be aligned to AxiWideByteOffset,
          // otherwise, they can be over-written.
          if (byte_addr >= DRAMAddrBase && byte_addr < DRAMAddrBase + DRAMLength)
            mem_image[byte_addr] = mem_row;
          else
            $display("Cannot initialize address %x, which doesn't fall into the L2 region.", byte_addr);
        end
      end
    end else begin
      $error("Expecting a firmware to run, none was provided!");
      $finish;
    end

    image_ready = 1'b1;
  end : dram_init

  // Distribute the image across the word-interleaved banks. This MUST mirror
  // ara_soc.sv's mapping exactly, and on the ABSOLUTE address: axi_to_mem's
  // mem_addr_o is the full AXI address, so that is what the interconnect
  // decodes. Do not subtract DRAMAddrBase here -- today it would make no
  // difference, but only because (DRAMAddrBase >> IndexLow) % L2NumWords == 0.
  //   bank  = addr[AxiWideByteOffset +: L2BankIdxWidth]
  //   index = addr[63 : AxiWideByteOffset + L2BankIdxWidth]
  for (genvar b = 0; b < NrL2Banks; b++) begin : gen_l2_preload
    initial begin
      wait (image_ready);
      foreach (mem_image[a]) begin
        if (((a >> AxiWideByteOffset) & (NrL2Banks - 1)) == b) begin
          automatic longint unsigned idx = a >> (AxiWideByteOffset + L2BankIdxWidth);
          // tc_sram decodes only $clog2(L2NumWords) index bits, so `idx % ...`
          // below mirrors the hardware truncation exactly. Warn only when the
          // address is genuinely past the end of physical memory, since that is
          // the case where two different addresses share a word. Note idx alone
          // cannot be tested against L2NumWords: the absolute address carries
          // DRAMAddrBase, an exact multiple of L2NumWords once shifted, so idx
          // exceeds L2NumWords for every DRAM address while still truncating
          // to the right place.
          if ((a - DRAMAddrBase) >= NrL2Banks * L2NumWords * AxiWideBeWidth)
            $warning("Address %x is past L2 capacity (%0d B); it aliases onto bank %0d index %0d",
                     a, NrL2Banks * L2NumWords * AxiWideBeWidth, b, idx % L2NumWords);
          dut.i_ara_soc.gen_l2_mem[b].i_dram.init_val[idx % L2NumWords] = mem_image[a];
        end
      end
    end
  end

`ifndef TARGET_GATESIM

  /*************************
   *  PRINT STORED VALUES  *
   *************************/

  // This is useful to check that the ideal dispatcher simulation was correct

`ifndef IDEAL_DISPATCHER
  localparam OutResultFile = "../gold_results.txt";
`else
  localparam OutResultFile = "../id_results.txt";
`endif

  int fd;

  data_t                     ara_w;
  logic [AxiWideBeWidth-1:0] ara_w_strb;
  logic                      ara_w_valid;
  logic                      ara_w_ready;

  // Avoid dumping what it's not measured, e.g. cache warming
  logic dump_en_mask;

  initial begin
    fd = $fopen(OutResultFile, "w");
    $display("Dump results on %s", OutResultFile);
  end

`ifndef USE_CLUSTER
  assign ara_w       = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara.i_vlsu.axi_req.w.data;
  assign ara_w_strb  = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara.i_vlsu.axi_req.w.strb;
  assign ara_w_valid = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara.i_vlsu.axi_req.w_valid;
  assign ara_w_ready = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara.i_vlsu.axi_resp.w_ready;
`else
  assign ara_w       = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara_cluster.axi_req_o.w.data;
  assign ara_w_strb  = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara_cluster.axi_req_o.w.strb;
  assign ara_w_valid = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara_cluster.axi_req_o.w_valid;
  assign ara_w_ready = dut.i_ara_soc.gen_ara_system[0].i_system.i_ara_cluster.axi_resp_i.w_ready;
`endif

`ifndef IDEAL_DISPATCHER
  assign dump_en_mask = dut.i_ara_soc.hw_cnt_en_o[0];
`else
  // Ideal-Dispatcher system does not warm the scalar cache
  assign dump_en_mask = 1'b1;
`endif
  always_ff @(posedge clk)
    if (dump_en_mask)
      if (ara_w_valid && ara_w_ready)
        for (int b = 0; b < AxiWideBeWidth; b++)
          if (ara_w_strb[b])
            $fdisplay(fd, "%0x", ara_w[b*8 +: 8]);

`endif

  /*********
   *  EOC  *
   *********/

`ifndef TARGET_GATESIM
  for (genvar core = 0; core < NrCores; core++) begin : gen_fpu_disp_core
    for (genvar gc = 0; gc < NrClusters; gc++) begin : gen_fpu_disp_cluster
      for (genvar gl = 0; gl < NrLanes; gl++) begin : gen_fpu_disp_lane
        always @(posedge clk) begin
          if (exit[0] && !(exit >> 1)) begin
            $display("core-%0d-cluster-%0d-lane-%0d [fpu-cycles] : %d", core, gc, gl, int'(dut.i_ara_soc.gen_ara_system[core].i_system.i_ara_cluster.p_cluster[gc].i_ara_macro.i_ara.gen_lanes[gl].i_lane.i_vfus.i_vmfpu.fpu_gen.vfpu_cnt_q));
          end
        end
      end : gen_fpu_disp_lane
    end : gen_fpu_disp_cluster
  end : gen_fpu_disp_core
`endif

  always @(posedge clk) begin
    if (exit[0]) begin
      if (exit >> 1) begin
        $warning("Core Test ", $sformatf("*** FAILED *** (tohost = %0d)", (exit >> 1)));
      end else begin
`ifndef TARGET_GATESIM
        $display("[hw-cycles]: %d", int'(dut.runtime_buf_q));
        $display("[cva6-d$-stalls]: %d", int'(dut.dcache_stall_buf_q));
        $display("[cva6-i$-stalls]: %d", int'(dut.icache_stall_buf_q));
        $display("[cva6-sb-full]: %d", int'(dut.sb_full_buf_q));
`endif
        $info("Core Test ", $sformatf("*** SUCCESS *** (tohost = %0d)", (exit >> 1)));
      end

`ifndef TARGET_GATESIM
      $fclose(fd);
`endif
      $finish(exit >> 1);
    end
  end

// Dump VCD with a SW trigger
`ifdef VCD_DUMP

  /****************
  *  VCD DUMPING  *
  ****************/

`ifdef VCD_PATH
  string vcd_path = `STRINGIFY(`VCD_PATH);
`else
  string vcd_path = "../vcd/last_sim.vcd";
`endif

  localparam logic [63:0] VCD_TRIGGER_ON  = 64'h0000_0000_0000_0001;
  localparam logic [63:0] VCD_TRIGGER_OFF = 64'hFFFF_FFFF_FFFF_FFFF;

  event start_dump_event;
  event stop_dump_event;

  logic [63:0] event_trigger_reg;
  logic        dumping = 1'b0;

  assign event_trigger_reg =
           dut.i_ara_soc.i_ctrl_registers.event_trigger_o;

  initial begin
    $display("VCD_DUMP successfully defined\n");
  end

  always_ff @(posedge clk) begin
    if(event_trigger_reg == VCD_TRIGGER_ON && !dumping) begin
       $display("[TB - VCD] START DUMPING\n");
       -> start_dump_event;
       dumping = 1'b1;
    end
    if(event_trigger_reg == VCD_TRIGGER_OFF) begin
       -> stop_dump_event;
       $display("[TB - VCD] STOP DUMPING\n");
    end
  end

  initial begin
    @(start_dump_event);
    $dumpfile(vcd_path);
    $dumpvars(0, dut.i_ara_soc.i_system);
    $dumpon;

    #1 $display("[TB - VCD] DUMPING...\n");

    @(stop_dump_event)
    $dumpoff;
    $dumpflush;
    $finish;
  end

`endif

endmodule : ara_tb
