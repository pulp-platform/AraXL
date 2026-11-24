// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Multicore fabric: per-core address decode, shared atomics, and banked L2.
// ara_system instances, SRAMs, and peripherals live in the common ara_soc.

module ara_interconnect_multicore #(
    parameter int unsigned NrCores = 2,
    parameter int unsigned NrL2Banks = NrCores,
    parameter int unsigned AxiAddrWidth = 64,
    parameter int unsigned AxiDataWidth = 512,
    parameter int unsigned AxiUserWidth = 1,
    parameter int unsigned AxiSystemIdWidth = 5,
    parameter logic [63:0] DRAMBase = 64'h8000_0000,
    parameter logic [63:0] DRAMLength = 64'h0200_0000,
    parameter logic [63:0] DRAMAtomLength = 64'h800,
    parameter logic [63:0] UARTBase = 64'hC000_0000,
    parameter logic [63:0] UARTLength = 64'h1000,
    parameter logic [63:0] CTRLBase = 64'hD000_0000,
    parameter logic [63:0] CTRLLength = 64'h1000,
    parameter type system_axi_req_t = logic,
    parameter type system_axi_resp_t = logic,
    parameter type soc_axi_req_t = logic,
    parameter type soc_axi_resp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,
    input system_axi_req_t [NrCores-1:0] system_req_i,
    output system_axi_resp_t [NrCores-1:0] system_resp_o,
    output soc_axi_req_t [1:0] periph_req_o,
    input soc_axi_resp_t [1:0] periph_resp_i,
    output logic [NrL2Banks-1:0] l2_req_o,
    output logic [NrL2Banks-1:0] l2_we_o,
    output logic [NrL2Banks-1:0][AxiAddrWidth-1:0] l2_addr_o,
    output logic [NrL2Banks-1:0][AxiDataWidth/8-1:0] l2_strb_o,
    output logic [NrL2Banks-1:0][AxiDataWidth-1:0] l2_wdata_o,
    input logic [NrL2Banks-1:0][AxiDataWidth-1:0] l2_rdata_i
);

  `include "axi/typedef.svh"

  localparam int unsigned NrAXIMasters = NrCores;
  localparam int unsigned NrAXISlaves = 2;
  localparam int unsigned NONL2 = 0;
  localparam int unsigned L2MEM = 1;
  localparam int unsigned AMO = 2;
  localparam int unsigned NrAXISlavePre = 3;
  localparam int unsigned UART = 0;
  localparam int unsigned CTRL = 1;
  localparam int unsigned AxiSocIdWidth = AxiSystemIdWidth + $clog2(NrCores);
  localparam int unsigned AxiWideDataWidth = AxiDataWidth;
  localparam int unsigned NrL2Req = NrCores + 1;
  localparam int unsigned L2AmoReq = NrCores;
  localparam int unsigned L2Latency = 1;
  localparam logic [63:0] DRAMAtomBase = DRAMBase + DRAMLength - DRAMAtomLength;

  typedef logic [AxiAddrWidth-1:0] axi_addr_t;
  typedef logic [AxiDataWidth-1:0] axi_data_t;
  typedef logic [AxiDataWidth/8-1:0] axi_strb_t;
  typedef logic [AxiUserWidth-1:0] axi_user_t;
  typedef logic [AxiSystemIdWidth-1:0] axi_system_id_t;
  typedef logic [AxiSocIdWidth-1:0] axi_soc_id_t;
  `AXI_TYPEDEF_ALL(system, axi_addr_t, axi_system_id_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_ALL(soc_wide, axi_addr_t, axi_soc_id_t, axi_data_t, axi_strb_t, axi_user_t)

  system_req_t [NrAXIMasters-1:0] system_axi_req;
  system_resp_t [NrAXIMasters-1:0] system_axi_resp;

  // Per-core address decoding immediately outside ara_system.
  system_req_t [NrAXIMasters-1:0][NrAXISlavePre-1:0] system_axi_req_bifur;
  system_resp_t [NrAXIMasters-1:0][NrAXISlavePre-1:0] system_axi_resp_bifur;

  // Atomics path: every core's AMO branch is gathered here and muxed onto one
  // port, so that a single axi_riscv_atomics adapter sees all cores' atomic
  // traffic.
  system_req_t [NrCores-1:0] amo_slv_req;
  system_resp_t [NrCores-1:0] amo_slv_resp;
  soc_wide_req_t amo_axi_req, amo_axi_req_flt;
  soc_wide_resp_t amo_axi_resp, amo_axi_resp_flt;

  soc_wide_req_t  [NrAXISlaves-1:0]                     periph_wide_axi_req;
  soc_wide_resp_t [NrAXISlaves-1:0]                     periph_wide_axi_resp;

  // Bank addresses are word indices with the bank-select bits removed.
  logic           [  NrL2Banks-1:0]                     l2b_req;
  logic           [  NrL2Banks-1:0]                     l2b_we;
  logic           [  NrL2Banks-1:0][  AxiAddrWidth-1:0] l2b_addr;
  logic           [  NrL2Banks-1:0][AxiDataWidth/8-1:0] l2b_strb;
  logic           [  NrL2Banks-1:0][  AxiDataWidth-1:0] l2b_wdata;
  logic           [  NrL2Banks-1:0][  AxiDataWidth-1:0] l2b_rdata;

  assign periph_req_o = periph_wide_axi_req;
  assign periph_wide_axi_resp = periph_resp_i;
  assign l2_req_o = l2b_req;
  assign l2_we_o = l2b_we;
  assign l2_addr_o = l2b_addr;
  assign l2_strb_o = l2b_strb;
  assign l2_wdata_o = l2b_wdata;
  assign l2b_rdata = l2_rdata_i;

  ////////////////
  //  Crossbar  //
  ////////////////

  localparam axi_pkg::xbar_cfg_t XBarCfg = '{
      NoSlvPorts        : NrAXIMasters,
      NoMstPorts        : NrAXISlaves,
      MaxMstTrans       : 4,
      MaxSlvTrans       : 4,
      FallThrough       : 1'b0,
      LatencyMode       : axi_pkg::CUT_MST_PORTS,
      PipelineStages    : 0,
      AxiIdWidthSlvPorts: AxiSystemIdWidth,
      AxiIdUsedSlvPorts : AxiSystemIdWidth,
      UniqueIds         : 1'b0,
      AxiAddrWidth      : AxiAddrWidth,
      AxiDataWidth      : AxiWideDataWidth,
      NoAddrRules       : NrAXISlaves
  };

  axi_pkg::xbar_rule_64_t [NrAXISlaves-1:0] routing_rules;

  assign routing_rules = '{
          '{idx: CTRL, start_addr: CTRLBase, end_addr: CTRLBase + CTRLLength},
          '{idx: UART, start_addr: UARTBase, end_addr: UARTBase + UARTLength}
      };

  axi_xbar #(
      .Cfg          (XBarCfg),
      .slv_aw_chan_t(system_aw_chan_t),
      .mst_aw_chan_t(soc_wide_aw_chan_t),
      .w_chan_t     (system_w_chan_t),
      .slv_b_chan_t (system_b_chan_t),
      .mst_b_chan_t (soc_wide_b_chan_t),
      .slv_ar_chan_t(system_ar_chan_t),
      .mst_ar_chan_t(soc_wide_ar_chan_t),
      .slv_r_chan_t (system_r_chan_t),
      .mst_r_chan_t (soc_wide_r_chan_t),
      .slv_req_t    (system_req_t),
      .slv_resp_t   (system_resp_t),
      .mst_req_t    (soc_wide_req_t),
      .mst_resp_t   (soc_wide_resp_t),
      .rule_t       (axi_pkg::xbar_rule_64_t)
  ) i_soc_xbar (
      .clk_i                (clk_i),
      .rst_ni               (rst_ni),
      .test_i               (1'b0),
      .slv_ports_req_i      (system_axi_req),
      .slv_ports_resp_o     (system_axi_resp),
      .mst_ports_req_o      (periph_wide_axi_req),
      .mst_ports_resp_i     (periph_wide_axi_resp),
      .addr_map_i           (routing_rules),
      .en_default_mst_port_i('0),
      .default_mst_port_i   ('0)
  );

  //////////
  //  L2  //
  //////////

  // Outstanding memory requests per requester. Must cover the interconnect
  // round trip (arbitration + SRAM latency), not just the SRAM latency: the
  // axi_to_mem default of 1 would throttle the bank port.
  localparam int unsigned L2BufDepth = 4;

  // Requester side of the L2 interconnect: one port per core, plus the shared
  // atomics path at index L2AmoReq.
  logic [NrL2Req-1:0]                     l2r_req;
  logic [NrL2Req-1:0]                     l2r_gnt;
  logic [NrL2Req-1:0]                     l2r_we;
  logic [NrL2Req-1:0][  AxiAddrWidth-1:0] l2r_addr;
  logic [NrL2Req-1:0][AxiDataWidth/8-1:0] l2r_strb;
  logic [NrL2Req-1:0][  AxiDataWidth-1:0] l2r_wdata;
  logic [NrL2Req-1:0][  AxiDataWidth-1:0] l2r_rdata;
  logic [NrL2Req-1:0]                     l2r_rvalid;

  // One axi_to_mem per core: turns that core's AXI bursts into one memory
  // request per AxiDataWidth word, which the interconnect then arbitrates word
  // by word. This is what makes per-word QoS fall out for free -- no burst can
  // occupy a bank for longer than a single cycle.
  for (genvar c = 0; c < NrCores; c++) begin : gen_core_axi_to_mem
    axi_to_mem #(
        .AddrWidth (AxiAddrWidth),
        .DataWidth (AxiDataWidth),
        .IdWidth   (AxiSystemIdWidth),
        .NumBanks  (1),
        .BufDepth  (L2BufDepth),
        .axi_req_t (system_req_t),
        .axi_resp_t(system_resp_t)
    ) i_axi_to_mem (
        .clk_i       (clk_i),
        .rst_ni      (rst_ni),
        .axi_req_i   (system_axi_req_bifur[c][L2MEM]),
        .axi_resp_o  (system_axi_resp_bifur[c][L2MEM]),
        .mem_req_o   (l2r_req[c]),
        .mem_gnt_i   (l2r_gnt[c]),
        .mem_we_o    (l2r_we[c]),
        .mem_addr_o  (l2r_addr[c]),
        .mem_strb_o  (l2r_strb[c]),
        .mem_wdata_o (l2r_wdata[c]),
        .mem_rdata_i (l2r_rdata[c]),
        .mem_rvalid_i(l2r_rvalid[c]),
        .mem_atop_o  (  /* ATOPs filtered upstream */),
        .busy_o      (  /* Unused */)
    );
  end

  ////////////////////
  //  Atomics path  //
  ////////////////////

  // All cores' AMO branches converge on one adapter. Putting the adapter per
  // core instead would let two cores each run their own read-modify-write on
  // the same address, silently breaking sync_barrier().
  for (genvar c = 0; c < NrCores; c++) begin : gen_amo_gather
    assign amo_slv_req[c]                = system_axi_req_bifur[c][AMO];
    assign system_axi_resp_bifur[c][AMO] = amo_slv_resp[c];
  end

  axi_mux #(
      .SlvAxiIDWidth(AxiSystemIdWidth),
      .slv_aw_chan_t(system_aw_chan_t),
      .mst_aw_chan_t(soc_wide_aw_chan_t),
      .w_chan_t     (system_w_chan_t),
      .slv_b_chan_t (system_b_chan_t),
      .mst_b_chan_t (soc_wide_b_chan_t),
      .slv_ar_chan_t(system_ar_chan_t),
      .mst_ar_chan_t(soc_wide_ar_chan_t),
      .slv_r_chan_t (system_r_chan_t),
      .mst_r_chan_t (soc_wide_r_chan_t),
      .slv_req_t    (system_req_t),
      .slv_resp_t   (system_resp_t),
      .mst_req_t    (soc_wide_req_t),
      .mst_resp_t   (soc_wide_resp_t),
      .NoSlvPorts   (NrCores),
      .SpillAw      (1'b1),
      .SpillW       (1'b1),
      .SpillB       (1'b1),
      .SpillAr      (1'b1),
      .SpillR       (1'b1)
  ) i_amo_mux (
      .clk_i      (clk_i),
      .rst_ni     (rst_ni),
      .test_i     (1'b0),
      .slv_reqs_i (amo_slv_req),
      .slv_resps_o(amo_slv_resp),
      .mst_req_o  (amo_axi_req),
      .mst_resp_i (amo_axi_resp)
  );

  axi_riscv_atomics_structs #(
      .AxiAddrWidth   (AxiAddrWidth),
      .AxiDataWidth   (AxiDataWidth),
      .AxiIdWidth     (AxiSocIdWidth),
      .AxiUserWidth   (AxiUserWidth),
      .AxiMaxReadTxns (8),
      .AxiMaxWriteTxns(8),
      .RiscvWordWidth (64),
      .axi_req_t      (soc_wide_req_t),
      .axi_rsp_t      (soc_wide_resp_t)
  ) i_axi_riscv_atomics_wrap (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .axi_slv_req_i(amo_axi_req),
      .axi_slv_rsp_o(amo_axi_resp),
      .axi_mst_req_o(amo_axi_req_flt),
      .axi_mst_rsp_i(amo_axi_resp_flt)
  );

  axi_to_mem #(
      .AddrWidth (AxiAddrWidth),
      .DataWidth (AxiDataWidth),
      .IdWidth   (AxiSocIdWidth),
      .NumBanks  (1),
      .BufDepth  (L2BufDepth),
      .axi_req_t (soc_wide_req_t),
      .axi_resp_t(soc_wide_resp_t)
  ) i_amo_axi_to_mem (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .axi_req_i   (amo_axi_req_flt),
      .axi_resp_o  (amo_axi_resp_flt),
      .mem_req_o   (l2r_req[L2AmoReq]),
      .mem_gnt_i   (l2r_gnt[L2AmoReq]),
      .mem_we_o    (l2r_we[L2AmoReq]),
      .mem_addr_o  (l2r_addr[L2AmoReq]),
      .mem_strb_o  (l2r_strb[L2AmoReq]),
      .mem_wdata_o (l2r_wdata[L2AmoReq]),
      .mem_rdata_i (l2r_rdata[L2AmoReq]),
      .mem_rvalid_i(l2r_rvalid[L2AmoReq]),
      .mem_atop_o  (  /* resolved by the adapter above */),
      .busy_o      (  /* Unused */)
  );

  /////////////////////////
  //  L2 interconnect    //
  /////////////////////////

  // Word-interleaved, fixed-latency. Because each requester's mem port is a
  // single in-order req/gnt channel and every bank has the same latency,
  // grant order == issue order == response order: no reorder buffer needed.
  // The mapping is derived inside the module from DataWidth and NoMstPorts.
  // The SRAMs and ELF loader in the parent hierarchy use the same mapping.
  l2_mem_interconnect #(
      .NoSlvPorts(NrL2Req),
      .NoMstPorts(NrL2Banks),
      .AddrWidth (AxiAddrWidth),
      .DataWidth (AxiDataWidth),
      .Latency   (L2Latency)
  ) i_l2_mem_interconnect (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .slv_ports_req_i   (l2r_req),
      .slv_ports_gnt_o   (l2r_gnt),
      .slv_ports_we_i    (l2r_we),
      .slv_ports_addr_i  (l2r_addr),
      .slv_ports_strb_i  (l2r_strb),
      .slv_ports_wdata_i (l2r_wdata),
      .slv_ports_rdata_o (l2r_rdata),
      .slv_ports_rvalid_o(l2r_rvalid),
      .mst_ports_req_o   (l2b_req),
      .mst_ports_we_o    (l2b_we),
      .mst_ports_addr_o  (l2b_addr),
      .mst_ports_strb_o  (l2b_strb),
      .mst_ports_wdata_o (l2b_wdata),
      .mst_ports_rdata_i (l2b_rdata)
  );

  for (genvar hart_id = 0; hart_id < NrCores; hart_id++) begin : gen_core_decode
    ////////////////////////////////
    //  Crossbar (L2 and non-L2)  //
    ////////////////////////////////

    localparam axi_pkg::xbar_cfg_t XBarPreCfg = '{
        NoSlvPorts        : 1,
        NoMstPorts        : NrAXISlavePre,
        MaxMstTrans       : 4,
        MaxSlvTrans       : 4,
        FallThrough       : 1'b0,
        LatencyMode       : axi_pkg::CUT_MST_PORTS,
        PipelineStages    : 0,
        AxiIdWidthSlvPorts: AxiSystemIdWidth,
        AxiIdUsedSlvPorts : AxiSystemIdWidth,
        UniqueIds         : 1'b0,
        AxiAddrWidth      : AxiAddrWidth,
        AxiDataWidth      : AxiWideDataWidth,
        NoAddrRules       : 2
    };

    axi_pkg::xbar_rule_64_t [1:0] routing_rules_pre;

    // Rules must not overlap. Anything matching neither (UART, CTRL) falls through
    // to NONL2 via en_default_mst_port_i below.
    assign routing_rules_pre = '{
            '{idx: AMO, start_addr: DRAMAtomBase, end_addr: DRAMAtomBase + DRAMAtomLength},
            '{idx: L2MEM, start_addr: DRAMBase, end_addr: DRAMAtomBase}
        };

    axi_xbar #(
        .Cfg          (XBarPreCfg),
        .slv_aw_chan_t(system_aw_chan_t),
        .mst_aw_chan_t(system_aw_chan_t),
        .w_chan_t     (system_w_chan_t),
        .slv_b_chan_t (system_b_chan_t),
        .mst_b_chan_t (system_b_chan_t),
        .slv_ar_chan_t(system_ar_chan_t),
        .mst_ar_chan_t(system_ar_chan_t),
        .slv_r_chan_t (system_r_chan_t),
        .mst_r_chan_t (system_r_chan_t),
        .slv_req_t    (system_req_t),
        .slv_resp_t   (system_resp_t),
        .mst_req_t    (system_req_t),
        .mst_resp_t   (system_resp_t),
        .rule_t       (axi_pkg::xbar_rule_64_t)
    ) i_pre_xbar (
        .clk_i                (clk_i),
        .rst_ni               (rst_ni),
        .test_i               (1'b0),
        .slv_ports_req_i      (system_req_i[hart_id]),
        .slv_ports_resp_o     (system_resp_o[hart_id]),
        .mst_ports_req_o      (system_axi_req_bifur[hart_id]),
        .mst_ports_resp_i     (system_axi_resp_bifur[hart_id]),
        .addr_map_i           (routing_rules_pre),
        .en_default_mst_port_i('1),
        .default_mst_port_i   ('0)
    );

    // Wire req/resp to/from non-L2 region
    assign system_axi_req[hart_id]               = system_axi_req_bifur[hart_id][NONL2];
    assign system_axi_resp_bifur[hart_id][NONL2] = system_axi_resp[hart_id];

  end : gen_core_decode

endmodule : ara_interconnect_multicore
