// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Global Dispatcher unit that receives request from CVA6 to decode the 
// vl and vtype of the current configuration for LdSt requests.

module global_dispatcher import ara_pkg::*; import rvv_pkg::*; #(
    parameter int unsigned NrLanes    = 0,  // Number of parallel vector lanes.
    parameter int unsigned NrClusters = 0,
    parameter type         vlen_cl_t  = logic
)
(
    // Clock and reset
    input  logic                                 clk_i,
    input  logic                                 rst_ni,
    // Interfaces with Ariane
    input  accelerator_req_t                     acc_req_i,
    output logic                                 acc_req_ready_o,

    // Interface with GLSU
    // To prevent vl_ld/vl_st from changing before the undergoing ld/st finishes
    input  logic                                 ar_addrgen_ack_i,
    input  logic                                 aw_addrgen_ack_i,

    output vlen_cl_t vl_ld_o,
    output vlen_cl_t vl_st_o,
    output vtype_t   vtype_o
);
  `include "common_cells/registers.svh"

  vlen_cl_t  vl_st_d, vl_st_q;
  vlen_cl_t  vl_ld_d, vl_ld_q;
  vtype_t    vtype_d, vtype_q;
  logic      acc_req_ready_d, acc_req_ready_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
        vl_ld_q <= 0;
        vl_st_q <= 0;
        vtype_q <= '{vill: 1'b1, default: '0};
        acc_req_ready_q <= 1'b1;
    end else begin
        vl_ld_q <= vl_ld_d;
        vl_st_q <= vl_st_d;
        vtype_q <= vtype_d;
        acc_req_ready_q <= acc_req_ready_d;
    end
  end

  assign vl_ld_o = vl_ld_q;
  assign vl_st_o = vl_st_q;
  assign vtype_o = vtype_q;
//   assign acc_req_ready_o = acc_req_ready_d;
  assign acc_req_ready_o = 1'b1;
  
  // Converts between the XLEN-bit vtype CSR and its internal representation
  function automatic vtype_t vtype_xlen(riscv::xlen_t xlen);
    vtype_xlen = '{
      vill  : xlen[riscv::XLEN-1],
      vma   : xlen[7],
      vta   : xlen[6],
      vsew  : vew_e'(xlen[5:3]),
      vlmul : vlmul_e'(xlen[2:0])
    };
  endfunction : vtype_xlen

  always_comb begin
    vl_ld_d = vl_ld_q;
    vl_st_d = vl_st_q;
    vtype_d = vtype_q;
    acc_req_ready_d = acc_req_ready_q;

    if ((!acc_req_ready_q) && (ar_addrgen_ack_i || aw_addrgen_ack_i)) begin
        acc_req_ready_d = 1'b1;
    end

    if (acc_req_i.req_valid && acc_req_ready_q) begin
        automatic rvv_instruction_t insn = rvv_instruction_t'(acc_req_i.insn.instr);
        // Decode the instructions based on their opcode
        unique case (acc_req_i.insn.itype.opcode)
            riscv::OpcodeLoadFp: begin
                acc_req_ready_d = 1'b0;
            end
            riscv::OpcodeStoreFp: begin
                acc_req_ready_d = 1'b0;
            end
            riscv::OpcodeVec: begin
                acc_req_ready_d = 1'b1;
                unique case (insn.varith_type.func3)
                    OPCFG: begin
                        // Update vtype
                        if (insn.vsetvli_type.func1 == 1'b0) begin // vsetvli
                            vtype_d = vtype_xlen(riscv::xlen_t'(insn.vsetvli_type.zimm11));
                        end else if (insn.vsetivli_type.func2 == 2'b11) begin // vsetivli
                            vtype_d = vtype_xlen(riscv::xlen_t'(insn.vsetivli_type.zimm10));
                        end else if (insn.vsetvl_type.func7 == 7'b100_0000) begin // vsetvl
                            vtype_d = vtype_xlen(riscv::xlen_t'(acc_req_i.rs2[7:0]));
                        end

                        // Check whether the updated vtype makes sense
                        if ((vtype_d.vsew > rvv_pkg::vew_e'($clog2(ELENB))) || // SEW <= ELEN
                            (vtype_d.vlmul == LMUL_RSVD) ||                    // reserved value
                            // LMUL >= SEW/ELEN
                            (signed'($clog2(ELENB)) + signed'(vtype_d.vlmul) < signed'(vtype_d.vsew))) begin
                            vtype_d = '{vill: 1'b1, default: '0};
                            vl_ld_d    = '0;
                            vl_st_d    = '0;
                        end

                        // Update the vector length
                        else begin
                            // Maximum vector length. VLMAX = LMUL * VLEN / SEW.
                            automatic int unsigned vlmax = (VLENB << $clog2(NrClusters)) >> vtype_d.vsew;
                            unique case (vtype_d.vlmul)
                                LMUL_1  : vlmax <<= 0;
                                LMUL_2  : vlmax <<= 1;
                                LMUL_4  : vlmax <<= 2;
                                LMUL_8  : vlmax <<= 3;
                                // Fractional LMUL
                                LMUL_1_2: vlmax >>= 1;
                                LMUL_1_4: vlmax >>= 2;
                                LMUL_1_8: vlmax >>= 3;
                                default:;
                            endcase

                            if (insn.vsetivli_type.func2 == 2'b11) begin // vsetivli
                                vl_ld_d = vlen_t'(insn.vsetivli_type.uimm5);
                                vl_st_d = vlen_t'(insn.vsetivli_type.uimm5);
                            end else begin // vsetvl || vsetvli
                                if (insn.vsetvl_type.rs1 == '0 && insn.vsetvl_type.rd == '0) begin
                                    // Do not update the vector length
                                    vl_ld_d = vl_ld_q;
                                    vl_st_d = vl_st_q;
                                end else if (insn.vsetvl_type.rs1 == '0 && insn.vsetvl_type.rd != '0) begin
                                    // Set the vector length to vlmax
                                    vl_ld_d = vlmax;
                                    vl_st_d = vlmax;
                                end else begin
                                    // vl_st_d = ((|acc_req_i.rs1[$bits(acc_req_i.rs1)-1:$bits(vl_st_d)]) ||
                                    //   (vlen_cl_t'(acc_req_i.rs1) > vlmax)) ? vlmax : vlen_cl_t'(acc_req_i.rs1);

                                    // Normal stripmining
                                    // vl_d = ((|acc_req_i.rs1[$bits(acc_req_i.rs1)-1:$bits(vl_d)]) ||
                                    //     (vlen_t'(acc_req_i.rs1) > vlmax)) ? vlmax : vlen_t'(acc_req_i.rs1);
                                    if (((|acc_req_i.rs1[$bits(acc_req_i.rs1)-1:$bits(vl_ld_d)]) ||
                                        (vlen_cl_t'(acc_req_i.rs1) > vlmax))) begin
                                        vl_ld_d = vlmax;
                                    end else if (|vlen_cl_t'(acc_req_i.rs1[$clog2(NrClusters * NrLanes)-1:0])) begin
                                        vl_ld_d = vlen_cl_t'(((acc_req_i.rs1 >> $clog2(NrClusters * NrLanes)) + 1) << $clog2(NrClusters * NrLanes));
                                    end else begin
                                        vl_ld_d = vlen_cl_t'(acc_req_i.rs1);
                                    end

                                    vl_st_d = vl_ld_d;
                                end
                            end
                        end
                    end
                    default: ;
                endcase
            end
            default: acc_req_ready_d = 1'b1;
        endcase
    end
  end


endmodule