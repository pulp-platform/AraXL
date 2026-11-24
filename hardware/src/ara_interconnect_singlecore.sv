// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Single-core fabric, following commit 105a59a377a85a039afeb2427f724aec2280a19d:
// one AXI crossbar for L2/UART/CTRL and one AXI-to-memory bridge.
// Atomic operations are unsupported and rejected by the L2 ATOP filter.
// With the default single bank, the memory bridge directly drives the SRAM.

module ara_interconnect_singlecore #(
    parameter int unsigned NrL2Banks = 1,
    parameter int unsigned AxiAddrWidth = 64,
    parameter int unsigned AxiDataWidth = 512,
    parameter int unsigned AxiUserWidth = 1,
    parameter int unsigned AxiIdWidth = 5,
    parameter logic [63:0] DRAMBase = 64'h8000_0000,
    parameter logic [63:0] DRAMLength = 64'h0200_0000,
    parameter logic [63:0] UARTBase = 64'hC000_0000,
    parameter logic [63:0] UARTLength = 64'h1000,
    parameter logic [63:0] CTRLBase = 64'hD000_0000,
    parameter logic [63:0] CTRLLength = 64'h1000,
    parameter type axi_req_t = logic,
    parameter type axi_resp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,
    input axi_req_t system_req_i,
    output axi_resp_t system_resp_o,
    // Peripheral order is shared with ara_interconnect_multicore: UART, CTRL.
    output axi_req_t [1:0] periph_req_o,
    input axi_resp_t [1:0] periph_resp_i,
    output logic [NrL2Banks-1:0] l2_req_o,
    output logic [NrL2Banks-1:0] l2_we_o,
    output logic [NrL2Banks-1:0][AxiAddrWidth-1:0] l2_addr_o,
    output logic [NrL2Banks-1:0][AxiDataWidth/8-1:0] l2_strb_o,
    output logic [NrL2Banks-1:0][AxiDataWidth-1:0] l2_wdata_o,
    input logic [NrL2Banks-1:0][AxiDataWidth-1:0] l2_rdata_i
);

  `include "axi/typedef.svh"
  `include "common_cells/registers.svh"

  localparam int unsigned NrAXIMasters = 1;
  localparam int unsigned NrAXISlaves = 3;
  localparam int unsigned UART = 0;
  localparam int unsigned CTRL = 1;
  localparam int unsigned L2MEM = 2;

  typedef logic [AxiAddrWidth-1:0] axi_addr_t;
  typedef logic [AxiDataWidth-1:0] axi_data_t;
  typedef logic [AxiDataWidth/8-1:0] axi_strb_t;
  typedef logic [AxiUserWidth-1:0] axi_user_t;
  typedef logic [AxiIdWidth-1:0] axi_id_t;
  `AXI_TYPEDEF_ALL(system, axi_addr_t, axi_id_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_ALL(soc_wide, axi_addr_t, axi_id_t, axi_data_t, axi_strb_t, axi_user_t)

  soc_wide_req_t  [NrAXISlaves-1:0] periph_wide_axi_req;
  soc_wide_resp_t [NrAXISlaves-1:0] periph_wide_axi_resp;
  assign periph_req_o = periph_wide_axi_req[1:0];
  assign periph_wide_axi_resp[1:0] = periph_resp_i;

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
      AxiIdWidthSlvPorts: AxiIdWidth,
      AxiIdUsedSlvPorts : AxiIdWidth,
      UniqueIds         : 1'b0,
      AxiAddrWidth      : AxiAddrWidth,
      AxiDataWidth      : AxiDataWidth,
      NoAddrRules       : NrAXISlaves
  };

  axi_pkg::xbar_rule_64_t [NrAXISlaves-1:0] routing_rules;

  assign routing_rules = '{
          '{idx: L2MEM, start_addr: DRAMBase, end_addr: DRAMBase + DRAMLength},
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
      .slv_ports_req_i      (system_req_i),
      .slv_ports_resp_o     (system_resp_o),
      .mst_ports_req_o      (periph_wide_axi_req),
      .mst_ports_resp_i     (periph_wide_axi_resp),
      .addr_map_i           (routing_rules),
      .en_default_mst_port_i('0),
      .default_mst_port_i   ('0)
  );

  // One memory requester; no per-core demultiplexer, AMO mux, or arbiter.
  soc_wide_req_t  l2_axi_req;
  soc_wide_resp_t l2_axi_resp;
  axi_atop_filter #(
      .AxiIdWidth     (AxiIdWidth),
      .AxiMaxWriteTxns(4),
      .axi_req_t      (soc_wide_req_t),
      .axi_resp_t     (soc_wide_resp_t)
  ) i_l2_atop_filter (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .slv_req_i (periph_wide_axi_req[L2MEM]),
      .slv_resp_o(periph_wide_axi_resp[L2MEM]),
      .mst_req_o (l2_axi_req),
      .mst_resp_i(l2_axi_resp)
  );

  logic l2_req, l2_we, l2_rvalid;
  logic [  AxiAddrWidth-1:0] l2_addr;
  logic [AxiDataWidth/8-1:0] l2_strb;
  logic [AxiDataWidth-1:0] l2_wdata, l2_rdata;

  axi_to_mem #(
      .AddrWidth (AxiAddrWidth),
      .DataWidth (AxiDataWidth),
      .IdWidth   (AxiIdWidth),
      .NumBanks  (1),
      .axi_req_t (axi_req_t),
      .axi_resp_t(axi_resp_t)
  ) i_axi_to_mem (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .axi_req_i   (l2_axi_req),
      .axi_resp_o  (l2_axi_resp),
      .mem_req_o   (l2_req),
      .mem_gnt_i   (l2_req),
      .mem_we_o    (l2_we),
      .mem_addr_o  (l2_addr),
      .mem_strb_o  (l2_strb),
      .mem_wdata_o (l2_wdata),
      .mem_rdata_i (l2_rdata),
      .mem_rvalid_i(l2_rvalid),
      .mem_atop_o  (  /* Unused */),
      .busy_o      (  /* Unused */)
  );

  // Match the common SRAM's one-cycle read latency for reads and writes.
  `FF(l2_rvalid, l2_req, 1'b0)

  localparam int unsigned WordOffset   = $clog2(AxiDataWidth / 8);
  localparam int unsigned BankIdxWidth = $clog2(NrL2Banks);
  if (NrL2Banks == 1) begin : gen_single_bank
    assign l2_req_o[0] = l2_req;
    assign l2_we_o[0] = l2_we;
    assign l2_addr_o[0] = l2_addr >> WordOffset;
    assign l2_strb_o[0] = l2_strb;
    assign l2_wdata_o[0] = l2_wdata;
    assign l2_rdata = l2_rdata_i[0];
  end else begin : gen_multiple_banks
    // Optional banking needs only address decode and a delayed read selector:
    // there is still a single requester, so no arbitration is necessary.
    logic [BankIdxWidth-1:0] bank_sel, bank_sel_q;
    assign bank_sel = l2_addr[WordOffset+:BankIdxWidth];
    `FF(bank_sel_q, bank_sel, '0)
    assign l2_rdata = l2_rdata_i[bank_sel_q];
    for (genvar b = 0; b < NrL2Banks; b++) begin : gen_bank
      assign l2_req_o[b] = l2_req && bank_sel == BankIdxWidth'(b);
      assign l2_we_o[b] = l2_we;
      assign l2_addr_o[b] = l2_addr >> (WordOffset + BankIdxWidth);
      assign l2_strb_o[b] = l2_strb;
      assign l2_wdata_o[b] = l2_wdata;
    end
  end

endmodule : ara_interconnect_singlecore
