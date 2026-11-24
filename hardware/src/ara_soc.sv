// Copyright 2021-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>
// Description:
// Ara's SoC, containing Ariane, Ara, and a L2 cache.
//
// NrCores selects the single-core or multicore fabric and replicates the same
// ara_system. AxiIdWidth is the per-system output width (5 by default): the
// internal CVA6/Ara IDs use one fewer bit, and the multicore fabric appends
// $clog2(NrCores) routing bits outside the system boundary. Keep NrLanes,
// NrClusters, AXI widths, feature options, and build defines the same to reuse
// a synthesized ara_system across SoCs. NrL2Banks affects only the fabric/SRAMs.

module ara_soc
  import axi_pkg::*;
  import ara_pkg::*;
#(
    parameter int unsigned NrCores = 1,  // Number of Ariane cores in the system.
    // RVV Parameters
    parameter int unsigned NrLanes = 0,  // Number of parallel vector lanes.
    parameter int unsigned NrClusters = 0,  // Number of Ara instances
    // Support for floating-point data types
    parameter fpu_support_e FPUSupport = FPUSupportHalfSingleDouble,
    // External support for vfrec7, vfrsqrt7
    parameter fpext_support_e FPExtSupport = FPExtSupportEnable,
    // Support for fixed-point data types
    parameter fixpt_support_e FixPtSupport = FixedPointEnable,
    // AXI Interface
    parameter int unsigned AxiDataWidth = 32 * NrLanes * NrClusters,
    parameter int unsigned ClusterAxiDataWidth = 32 * NrLanes,
    parameter int unsigned AxiAddrWidth = 64,
    parameter int unsigned AxiUserWidth = 1,
    // ID width at each ara_system output, independent of NrCores.
    parameter int unsigned AxiIdWidth = 5,
    // AXI Resp Delay [ps] for gate-level simulation
    parameter int unsigned AxiRespDelay = 200,
    parameter int unsigned NrL2Banks = NrCores,
    // Dependant parameters. DO NOT CHANGE!
    localparam type axi_data_t = logic [AxiDataWidth-1:0],
    localparam type axi_strb_t = logic [AxiDataWidth/8-1:0],
    localparam type axi_addr_t = logic [AxiAddrWidth-1:0],
    localparam type axi_user_t = logic [AxiUserWidth-1:0],
    localparam type axi_id_t = logic [AxiIdWidth-1:0],

    localparam type cluster_axi_data_t = logic [  ClusterAxiDataWidth-1:0],
    localparam type cluster_axi_strb_t = logic [ClusterAxiDataWidth/8-1:0],
    localparam type cluster_axi_addr_t = logic [         AxiAddrWidth-1:0],
    localparam type cluster_axi_user_t = logic [         AxiUserWidth-1:0],
    localparam type cluster_axi_id_t   = logic [           AxiIdWidth-1:0]
) (
    input  logic        clk_i,
    input  logic        rst_ni,
    output logic [63:0] exit_o,
    output logic [63:0] hw_cnt_en_o,
    // Scan chain
    input  logic        scan_enable_i,
    input  logic        scan_data_i,
    output logic        scan_data_o,
    // UART APB interface
    output logic        uart_penable_o,
    output logic        uart_pwrite_o,
    output logic [31:0] uart_paddr_o,
    output logic        uart_psel_o,
    output logic [31:0] uart_pwdata_o,
    input  logic [31:0] uart_prdata_i,
    input  logic        uart_pready_i,
    input  logic        uart_pslverr_i
);

  `include "axi/typedef.svh"
  `include "apb/typedef.svh"
  `include "ara/intf_typedef.svh"

  localparam int UART = 0;
  localparam int CTRL = 1;

  localparam NrAXISlaves = CTRL + 1;

  // Memory Map
  // Physical L2 size, spread over NrL2Banks banks
  localparam logic [63:0] DRAMLength = 64'h0200_0000;
  localparam int unsigned L2NumWords = DRAMLength / (NrL2Banks * (AxiDataWidth / 8));
  localparam logic [63:0] UARTLength = 64'h1000;
  localparam logic [63:0] CTRLLength = 64'h1000;

  typedef enum logic [63:0] {
    DRAMBase = 64'h8000_0000,
    UARTBase = 64'hC000_0000,
    CTRLBase = 64'hD000_0000
  } soc_bus_start_e;

  ////////////////////////////
  //  L2 word interleaving  //
  ////////////////////////////

  // Non-cacheable window at the top of DRAM (see apps/common/arch.link.ld).
  localparam logic [63:0] DRAMAtomLength = 64'h800;
  localparam logic [63:0] DRAMAtomBase = DRAMBase + DRAMLength - DRAMAtomLength;

  // SRAM latency is shared by both interconnect implementations.
  localparam int unsigned L2Latency = 1;

  ///////////
  //  AXI  //
  ///////////

  // Ariane's AXI port data width
  localparam AxiNarrowDataWidth = 64;
  localparam AxiNarrowStrbWidth = AxiNarrowDataWidth / 8;
  // Ara's AXI port data width
  localparam AxiWideDataWidth = AxiDataWidth;
  localparam AXiWideStrbWidth = AxiWideDataWidth / 8;

  // Only the fabric appends the core index to transaction IDs.
  localparam AxiSystemIdWidth = AxiIdWidth;
  localparam AxiSocIdWidth = AxiSystemIdWidth + $clog2(NrCores);
  localparam AxiCoreIdWidth = AxiSystemIdWidth - 1;

  // Internal types
  typedef logic [AxiNarrowDataWidth-1:0] axi_narrow_data_t;
  typedef logic [AxiNarrowStrbWidth-1:0] axi_narrow_strb_t;
  typedef logic [AxiSocIdWidth-1:0] axi_soc_id_t;
  typedef logic [AxiSystemIdWidth-1:0] axi_system_id_t;
  typedef logic [AxiCoreIdWidth-1:0] axi_core_id_t;

  // AXI Typedefs
  `AXI_TYPEDEF_ALL(system, axi_addr_t, axi_system_id_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_ALL(ara_axi, axi_addr_t, axi_core_id_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_TYPEDEF_ALL(ariane_axi, axi_addr_t, axi_core_id_t, axi_narrow_data_t, axi_narrow_strb_t,
                   axi_user_t)
  `AXI_TYPEDEF_ALL(soc_narrow, axi_addr_t, axi_soc_id_t, axi_narrow_data_t, axi_narrow_strb_t,
                   axi_user_t)
  `AXI_TYPEDEF_ALL(soc_wide, axi_addr_t, axi_soc_id_t, axi_data_t, axi_strb_t, axi_user_t)
  `AXI_LITE_TYPEDEF_ALL(soc_narrow_lite, axi_addr_t, axi_narrow_data_t, axi_narrow_strb_t)

  `AXI_TYPEDEF_ALL(ara_cluster_axi, cluster_axi_addr_t, axi_core_id_t, cluster_axi_data_t,
                   cluster_axi_strb_t, cluster_axi_user_t)

  // Fixed-width system ports preserve the ara_system synthesis boundary.
  system_req_t  [NrCores-1:0] system_axi_req_pre;
  system_resp_t [NrCores-1:0] system_axi_resp_pre;
`ifdef TARGET_GATESIM
  system_req_t  [NrCores-1:0] system_axi_req_spill;
  system_resp_t [NrCores-1:0] system_axi_resp_spill;
  system_resp_t [NrCores-1:0] system_axi_resp_spill_del;
`endif

  soc_wide_req_t    [NrAXISlaves-1:0]                     periph_wide_axi_req;
  soc_wide_resp_t   [NrAXISlaves-1:0]                     periph_wide_axi_resp;
  soc_narrow_req_t  [NrAXISlaves-1:0]                     periph_narrow_axi_req;
  soc_narrow_resp_t [NrAXISlaves-1:0]                     periph_narrow_axi_resp;

  // Bank side of the L2 interconnect. bank_addr is already a word index with
  // the bank-select bits removed.
  logic             [  NrL2Banks-1:0]                     l2b_req;
  logic             [  NrL2Banks-1:0]                     l2b_we;
  logic             [  NrL2Banks-1:0][  AxiAddrWidth-1:0] l2b_addr;
  logic             [  NrL2Banks-1:0][AxiDataWidth/8-1:0] l2b_strb;
  logic             [  NrL2Banks-1:0][  AxiDataWidth-1:0] l2b_wdata;
  logic             [  NrL2Banks-1:0][  AxiDataWidth-1:0] l2b_rdata;

  // NrCores selects only the fabric. All ara_system parameters below are
  // identical for a given lane/cluster configuration in either topology.
  if (NrCores == 1) begin : gen_interconnect_singlecore
    ara_interconnect_singlecore #(
        .NrL2Banks   (NrL2Banks),
        .AxiAddrWidth(AxiAddrWidth),
        .AxiDataWidth(AxiDataWidth),
        .AxiUserWidth(AxiUserWidth),
        .AxiIdWidth  (AxiSystemIdWidth),
        .DRAMBase    (DRAMBase),
        .DRAMLength  (DRAMLength),
        .UARTBase    (UARTBase),
        .UARTLength  (UARTLength),
        .CTRLBase    (CTRLBase),
        .CTRLLength  (CTRLLength),
        .axi_req_t   (system_req_t),
        .axi_resp_t  (system_resp_t)
    ) i_interconnect (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .system_req_i (system_axi_req_pre[0]),
        .system_resp_o(system_axi_resp_pre[0]),
        .periph_req_o (periph_wide_axi_req),
        .periph_resp_i(periph_wide_axi_resp),
        .l2_req_o     (l2b_req),
        .l2_we_o      (l2b_we),
        .l2_addr_o    (l2b_addr),
        .l2_strb_o    (l2b_strb),
        .l2_wdata_o   (l2b_wdata),
        .l2_rdata_i   (l2b_rdata)
    );
  end else begin : gen_interconnect_multicore
    ara_interconnect_multicore #(
        .NrCores          (NrCores),
        .NrL2Banks        (NrL2Banks),
        .AxiAddrWidth     (AxiAddrWidth),
        .AxiDataWidth     (AxiDataWidth),
        .AxiUserWidth     (AxiUserWidth),
        .AxiSystemIdWidth (AxiSystemIdWidth),
        .DRAMBase         (DRAMBase),
        .DRAMLength       (DRAMLength),
        .DRAMAtomLength   (DRAMAtomLength),
        .UARTBase         (UARTBase),
        .UARTLength       (UARTLength),
        .CTRLBase         (CTRLBase),
        .CTRLLength       (CTRLLength),
        .system_axi_req_t (system_req_t),
        .system_axi_resp_t(system_resp_t),
        .soc_axi_req_t    (soc_wide_req_t),
        .soc_axi_resp_t   (soc_wide_resp_t)
    ) i_interconnect (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .system_req_i (system_axi_req_pre),
        .system_resp_o(system_axi_resp_pre),
        .periph_req_o (periph_wide_axi_req),
        .periph_resp_i(periph_wide_axi_resp),
        .l2_req_o     (l2b_req),
        .l2_we_o      (l2b_we),
        .l2_addr_o    (l2b_addr),
        .l2_strb_o    (l2b_strb),
        .l2_wdata_o   (l2b_wdata),
        .l2_rdata_i   (l2b_rdata)
    );
  end

`ifndef SPYGLASS
  for (genvar b = 0; b < NrL2Banks; b++) begin : gen_l2_mem
    tc_sram #(
        .NumWords (L2NumWords),
        .NumPorts (1),
        .DataWidth(AxiDataWidth),
        .SimInit  ("zeros"),
        .Latency  (L2Latency)
    ) i_dram (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .req_i  (l2b_req[b]),
        .we_i   (l2b_we[b]),
        .addr_i (l2b_addr[b][$clog2(L2NumWords)-1:0]),
        .wdata_i(l2b_wdata[b]),
        .be_i   (l2b_strb[b]),
        .rdata_o(l2b_rdata[b])
    );
  end
`else
  assign l2b_rdata = '{default: '0};
`endif

  ////////////
  //  UART  //
  ////////////

  `AXI_TYPEDEF_ALL(uart_axi, axi_addr_t, axi_soc_id_t, logic [31:0], logic [3:0], axi_user_t)
  `AXI_LITE_TYPEDEF_ALL(uart_lite, axi_addr_t, logic [31:0], logic [3:0])
  `APB_TYPEDEF_ALL(uart_apb, axi_addr_t, logic [31:0], logic [3:0])

  uart_axi_req_t   uart_axi_req;
  uart_axi_resp_t  uart_axi_resp;
  uart_lite_req_t  uart_lite_req;
  uart_lite_resp_t uart_lite_resp;
  uart_apb_req_t   uart_apb_req;
  uart_apb_resp_t  uart_apb_resp;

  assign uart_penable_o = uart_apb_req.penable;
  assign uart_pwrite_o  = uart_apb_req.pwrite;
  assign uart_paddr_o   = uart_apb_req.paddr;
  assign uart_psel_o    = uart_apb_req.psel;
  assign uart_pwdata_o  = uart_apb_req.pwdata;
  assign uart_apb_resp.prdata  = uart_prdata_i;
  assign uart_apb_resp.pready  = uart_pready_i;
  assign uart_apb_resp.pslverr = uart_pslverr_i;

  typedef struct packed {
    int unsigned idx;
    axi_addr_t   start_addr;
    axi_addr_t   end_addr;
  } uart_apb_rule_t;

  uart_apb_rule_t uart_apb_map;
  assign uart_apb_map = '{idx: 0, start_addr: '0, end_addr: '1};

  axi_lite_to_apb #(
      .NoApbSlaves     (32'd1),
      .NoRules         (32'd1),
      .AddrWidth       (AxiAddrWidth),
      .DataWidth       (32'd32),
      .PipelineRequest (1'b0),
      .PipelineResponse(1'b0),
      .axi_lite_req_t  (uart_lite_req_t),
      .axi_lite_resp_t (uart_lite_resp_t),
      .apb_req_t       (uart_apb_req_t),
      .apb_resp_t      (uart_apb_resp_t),
      .rule_t          (uart_apb_rule_t)
  ) i_axi_lite_to_apb_uart (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .axi_lite_req_i (uart_lite_req),
      .axi_lite_resp_o(uart_lite_resp),
      .apb_req_o      (uart_apb_req),
      .apb_resp_i     (uart_apb_resp),
      .addr_map_i     (uart_apb_map)
  );

  axi_to_axi_lite #(
      .AxiAddrWidth   (AxiAddrWidth),
      .AxiDataWidth   (32'd32),
      .AxiIdWidth     (AxiSocIdWidth),
      .AxiUserWidth   (AxiUserWidth),
      .AxiMaxWriteTxns(32'd1),
      .AxiMaxReadTxns (32'd1),
      .FallThrough    (1'b1),
      .full_req_t     (uart_axi_req_t),
      .full_resp_t    (uart_axi_resp_t),
      .lite_req_t     (uart_lite_req_t),
      .lite_resp_t    (uart_lite_resp_t)
  ) i_axi_to_axi_lite_uart (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .test_i    (1'b0),
      .slv_req_i (uart_axi_req),
      .slv_resp_o(uart_axi_resp),
      .mst_req_o (uart_lite_req),
      .mst_resp_i(uart_lite_resp)
  );

  axi_dw_converter #(
      .AxiSlvPortDataWidth(AxiWideDataWidth),
      .AxiMstPortDataWidth(32),
      .AxiAddrWidth       (AxiAddrWidth),
      .AxiIdWidth         (AxiSocIdWidth),
      .AxiMaxReads        (1),
      .ar_chan_t          (soc_wide_ar_chan_t),
      .mst_r_chan_t       (uart_axi_r_chan_t),
      .slv_r_chan_t       (soc_wide_r_chan_t),
      .aw_chan_t          (uart_axi_aw_chan_t),
      .b_chan_t           (soc_wide_b_chan_t),
      .mst_w_chan_t       (uart_axi_w_chan_t),
      .slv_w_chan_t       (soc_wide_w_chan_t),
      .axi_mst_req_t      (uart_axi_req_t),
      .axi_mst_resp_t     (uart_axi_resp_t),
      .axi_slv_req_t      (soc_wide_req_t),
      .axi_slv_resp_t     (soc_wide_resp_t)
  ) i_axi_slave_uart_dwc (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .slv_req_i (periph_wide_axi_req[UART]),
      .slv_resp_o(periph_wide_axi_resp[UART]),
      .mst_req_o (uart_axi_req),
      .mst_resp_i(uart_axi_resp)
  );

  /////////////////////////
  //  Control registers  //
  /////////////////////////

  soc_narrow_lite_req_t axi_lite_ctrl_registers_req;
  soc_narrow_lite_resp_t axi_lite_ctrl_registers_resp;

  logic [63:0] event_trigger;

  axi_to_axi_lite #(
      .AxiAddrWidth   (AxiAddrWidth),
      .AxiDataWidth   (AxiNarrowDataWidth),
      .AxiIdWidth     (AxiSocIdWidth),
      .AxiUserWidth   (AxiUserWidth),
      .AxiMaxReadTxns (1),
      .AxiMaxWriteTxns(1),
      .FallThrough    (1'b0),
      .full_req_t     (soc_narrow_req_t),
      .full_resp_t    (soc_narrow_resp_t),
      .lite_req_t     (soc_narrow_lite_req_t),
      .lite_resp_t    (soc_narrow_lite_resp_t)
  ) i_axi_to_axi_lite (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .test_i    (1'b0),
      .slv_req_i (periph_narrow_axi_req[CTRL]),
      .slv_resp_o(periph_narrow_axi_resp[CTRL]),
      .mst_req_o (axi_lite_ctrl_registers_req),
      .mst_resp_i(axi_lite_ctrl_registers_resp)
  );

  ctrl_registers #(
      .DRAMBaseAddr(DRAMBase),
      .DRAMLength     (DRAMLength -DRAMAtomLength), // stack starts before the mem allocated for non-cacheable region
      .DataWidth(AxiNarrowDataWidth),
      .AddrWidth(AxiAddrWidth),
      .axi_lite_req_t(soc_narrow_lite_req_t),
      .axi_lite_resp_t(soc_narrow_lite_resp_t)
  ) i_ctrl_registers (
      .clk_i                (clk_i),
      .rst_ni               (rst_ni),
      .axi_lite_slave_req_i (axi_lite_ctrl_registers_req),
      .axi_lite_slave_resp_o(axi_lite_ctrl_registers_resp),
      .hw_cnt_en_o          (hw_cnt_en_o),
      .dram_base_addr_o     (  /* Unused */),
      .dram_end_addr_o      (  /* Unused */),
      .exit_o               (exit_o),
      .event_trigger_o      (event_trigger)
  );

  axi_dw_converter #(
      .AxiSlvPortDataWidth(AxiWideDataWidth),
      .AxiMstPortDataWidth(AxiNarrowDataWidth),
      .AxiAddrWidth       (AxiAddrWidth),
      .AxiIdWidth         (AxiSocIdWidth),
      .AxiMaxReads        (2),
      .ar_chan_t          (soc_wide_ar_chan_t),
      .mst_r_chan_t       (soc_narrow_r_chan_t),
      .slv_r_chan_t       (soc_wide_r_chan_t),
      .aw_chan_t          (soc_narrow_aw_chan_t),
      .b_chan_t           (soc_narrow_b_chan_t),
      .mst_w_chan_t       (soc_narrow_w_chan_t),
      .slv_w_chan_t       (soc_wide_w_chan_t),
      .axi_mst_req_t      (soc_narrow_req_t),
      .axi_mst_resp_t     (soc_narrow_resp_t),
      .axi_slv_req_t      (soc_wide_req_t),
      .axi_slv_resp_t     (soc_wide_resp_t)
  ) i_axi_slave_ctrl_dwc (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .slv_req_i (periph_wide_axi_req[CTRL]),
      .slv_resp_o(periph_wide_axi_resp[CTRL]),
      .mst_req_o (periph_narrow_axi_req[CTRL]),
      .mst_resp_i(periph_narrow_axi_resp[CTRL])
  );

  //////////////
  //  System  //
  //////////////

  // Modify configuration parameters
  function automatic config_pkg::cva6_user_cfg_t gen_usr_cva6_config(
      config_pkg::cva6_user_cfg_t cfg);
    cfg.AxiAddrWidth            = AxiAddrWidth;
    cfg.AxiDataWidth            = AxiNarrowDataWidth;
    cfg.AxiIdWidth              = AxiCoreIdWidth;
    cfg.AxiUserWidth            = AxiUserWidth;
    cfg.XF16                    = FPUSupport[3];
    cfg.RVF                     = FPUSupport[4];
    cfg.RVD                     = FPUSupport[5];
    cfg.XF16ALT                 = FPUSupport[2];
    cfg.XF8                     = FPUSupport[1];
    // cfg.XF8ALT                = FPUSupport[0]; // Not supported by OpenHW Group's CVFPU
    cfg.NrPMPEntries            = 0;
    // idempotent region
    cfg.NrNonIdempotentRules    = 2;
    cfg.NonIdempotentAddrBase   = {UARTBase, CTRLBase};
    cfg.NonIdempotentLength     = {UARTLength, CTRLLength};
    cfg.NrExecuteRegionRules    = 3;
    //                          DRAM;       Boot ROM;   Debug Module
    cfg.ExecuteRegionAddrBase   = {DRAMBase, 64'h1_0000, 64'h0};
    cfg.ExecuteRegionLength     = {DRAMLength, 64'h10000, 64'h1000};
    // cached region
    cfg.NrCachedRegionRules     = 1;
    cfg.CachedRegionAddrBase    = {DRAMBase};
    cfg.CachedRegionLength      = {DRAMLength - DRAMAtomLength};
    // Shared-memory software uses FENCE for cache maintenance between the
    // non-coherent cores. Keep this hardware in the single-core system too,
    // so the same synthesized system can be reused in either topology.
    cfg.DcacheFlushOnFence      = 1'b1;
    cfg.DcacheInvalidateOnFlush = 1'b1;
    // Return modified config
    return cfg;
  endfunction

  // Generate the user defined package, starting from the template one for RVV
  localparam config_pkg::cva6_user_cfg_t CVA6AraConfig_user = gen_usr_cva6_config(
      cva6_config_pkg::cva6_cfg
  );
  // Build the package
  localparam config_pkg::cva6_cfg_t CVA6AraConfig = build_config_pkg::build_config(
      CVA6AraConfig_user
  );

  `CVA6_TYPEDEF_EXCEPTION(exception_t, CVA6AraConfig)

  // Standard interface
  `CVA6_INTF_TYPEDEF_ACC_REQ(accelerator_req_t, CVA6AraConfig, fpnew_pkg::roundmode_e)
  `CVA6_INTF_TYPEDEF_ACC_RESP(accelerator_resp_t, CVA6AraConfig, exception_t)
  // MMU interface
  `CVA6_INTF_TYPEDEF_MMU_REQ(acc_mmu_req_t, CVA6AraConfig)
  `CVA6_INTF_TYPEDEF_MMU_RESP(acc_mmu_resp_t, CVA6AraConfig, exception_t)
  // Accelerator - CVA6's top-level interface
  `CVA6_INTF_TYPEDEF_CVA6_TO_ACC(cva6_to_acc_t, accelerator_req_t, acc_mmu_resp_t)
  `CVA6_INTF_TYPEDEF_ACC_TO_CVA6(acc_to_cva6_t, accelerator_resp_t, acc_mmu_req_t)

  // Core count affects replication only, never the system configuration.
  for (genvar hart_id = 0; hart_id < NrCores; hart_id++) begin : gen_ara_system
`ifndef TARGET_GATESIM
    ara_system #(
        .NrLanes            (NrLanes),
        .NrClusters         (NrClusters),
        .FPUSupport         (FPUSupport),
        .FPExtSupport       (FPExtSupport),
        .FixPtSupport       (FixPtSupport),
        .CVA6Cfg            (CVA6AraConfig),
        .exception_t        (exception_t),
        .accelerator_req_t  (accelerator_req_t),
        .accelerator_resp_t (accelerator_resp_t),
        .acc_mmu_req_t      (acc_mmu_req_t),
        .acc_mmu_resp_t     (acc_mmu_resp_t),
        .cva6_to_acc_t      (cva6_to_acc_t),
        .acc_to_cva6_t      (acc_to_cva6_t),
        .AxiAddrWidth       (AxiAddrWidth),
        .AxiIdWidth         (AxiCoreIdWidth),
        .AxiNarrowDataWidth (AxiNarrowDataWidth),
        .AxiWideDataWidth   (AxiDataWidth),
        .ClusterAxiDataWidth(ClusterAxiDataWidth),
        .ara_axi_ar_t       (ara_axi_ar_chan_t),
        .ara_axi_aw_t       (ara_axi_aw_chan_t),
        .ara_axi_b_t        (ara_axi_b_chan_t),
        .ara_axi_r_t        (ara_axi_r_chan_t),
        .ara_axi_w_t        (ara_axi_w_chan_t),
        .ara_axi_req_t      (ara_axi_req_t),
        .ara_axi_resp_t     (ara_axi_resp_t),

        .cluster_axi_ar_t  (ara_cluster_axi_ar_chan_t),
        .cluster_axi_aw_t  (ara_cluster_axi_aw_chan_t),
        .cluster_axi_b_t   (ara_cluster_axi_b_chan_t),
        .cluster_axi_r_t   (ara_cluster_axi_r_chan_t),
        .cluster_axi_w_t   (ara_cluster_axi_w_chan_t),
        .cluster_axi_req_t (ara_cluster_axi_req_t),
        .cluster_axi_resp_t(ara_cluster_axi_resp_t),

        .ariane_axi_ar_t  (ariane_axi_ar_chan_t),
        .ariane_axi_aw_t  (ariane_axi_aw_chan_t),
        .ariane_axi_b_t   (ariane_axi_b_chan_t),
        .ariane_axi_r_t   (ariane_axi_r_chan_t),
        .ariane_axi_w_t   (ariane_axi_w_chan_t),
        .ariane_axi_req_t (ariane_axi_req_t),
        .ariane_axi_resp_t(ariane_axi_resp_t),
        .system_axi_ar_t  (system_ar_chan_t),
        .system_axi_aw_t  (system_aw_chan_t),
        .system_axi_b_t   (system_b_chan_t),
        .system_axi_r_t   (system_r_chan_t),
        .system_axi_w_t   (system_w_chan_t),
        .system_axi_req_t (system_req_t),
        .system_axi_resp_t(system_resp_t)
    ) i_system (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .boot_addr_i  (DRAMBase),                     // start fetching from DRAM
        .hart_id_i    (3'(hart_id)),
        .scan_enable_i(1'b0),
        .scan_data_i  (1'b0),
        .scan_data_o  (  /* Unconnected */),
        .axi_req_o    (system_axi_req_pre[hart_id]),
        .axi_resp_i   (system_axi_resp_pre[hart_id])
    );
`else
    // Reuse the same system netlist for every core count.
    ara_system i_system (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .boot_addr_i  (DRAMBase),
        .hart_id_i    (3'(hart_id)),
        .scan_enable_i(1'b0),
        .scan_data_i  (1'b0),
        .scan_data_o  (  /* Unconnected */),
        .axi_req_o    (system_axi_req_spill[hart_id]),
        .axi_resp_i   (system_axi_resp_spill_del[hart_id])
    );
    assign #(AxiRespDelay*1ps) system_axi_resp_spill_del[hart_id] = system_axi_resp_spill[hart_id];

    axi_cut #(
        .ar_chan_t (system_ar_chan_t),
        .aw_chan_t (system_aw_chan_t),
        .b_chan_t  (system_b_chan_t),
        .r_chan_t  (system_r_chan_t),
        .w_chan_t  (system_w_chan_t),
        .axi_req_t (system_req_t),
        .axi_resp_t(system_resp_t)
    ) i_system_cut (
        .clk_i     (clk_i),
        .rst_ni    (rst_ni),
        .slv_req_i (system_axi_req_spill[hart_id]),
        .slv_resp_o(system_axi_resp_spill[hart_id]),
        .mst_req_o (system_axi_req_pre[hart_id]),
        .mst_resp_i(system_axi_resp_pre[hart_id])
    );
`endif

  end : gen_ara_system

  //////////////////
  //  Assertions  //
  //////////////////

  if (NrCores == 0 || NrCores > 8 || (NrCores & (NrCores - 1)) != 0)
    $error("[ara_soc] NrCores must be a power of two between 1 and 8 (hart_id_i is 3 bits).");

  if (NrL2Banks == 0 || (NrL2Banks & (NrL2Banks - 1)) != 0)
    $error("[ara_soc] NrL2Banks must be a power of two.");

  if (NrLanes == 0) $error("[ara_soc] Ara needs to have at least one lane.");

  if (NrClusters == 0) $error("[ara_soc] Ara needs to have atleast one group");

  if (AxiDataWidth == 0) $error("[ara_soc] The AXI data width must be greater than zero.");

  if (AxiAddrWidth == 0) $error("[ara_soc] The AXI address width must be greater than zero.");

  if (AxiUserWidth == 0) $error("[ara_soc] The AXI user width must be greater than zero.");

  if (AxiIdWidth < 5)
    $error("[ara_soc] AxiIdWidth must include at least 4 CVA6 ID bits and 1 system mux bit.");

endmodule : ara_soc
