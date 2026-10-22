// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Global load store unit that receives LD-ST AXI request from Ara instances
// and generates/receives an AXI request/response to/from the System XBAR.

module reshuffle_unit import ara_pkg::*; import rvv_pkg::*;  #(
  parameter  int unsigned NrLanes             = 0,
  parameter  int unsigned NrClusters          = 0,
  parameter  int unsigned AxiDataWidth        = 0,
  parameter  int unsigned ClusterAxiDataWidth = 0,
  parameter  int unsigned AxiAddrWidth        = 0, 
  parameter type cluster_axi_req_t            = logic,
  parameter type cluster_axi_resp_t           = logic,
  parameter type axi_req_t                    = logic,
  parameter type axi_resp_t                   = logic,
  parameter type axi_addr_t                   = logic [AxiAddrWidth-1:0],

  localparam int size_axi                 = $clog2(AxiDataWidth/8),
  localparam int numShuffleStages         = $clog2(AxiDataWidth/(8*NrLanes))-1,
  localparam logic is_full_bw             = (NrClusters == (AxiDataWidth/ClusterAxiDataWidth)) ? 1'b1 : 1'b0,
  localparam int MaxAxiBurst              = 256
  ) (
  input  logic                           clk_i,
  input  logic                           rst_ni,

  input  cluster_metadata_t             cluster_metadata_i,
  output  cluster_metadata_t             cluster_metadata_o,
  
  // To ARA
  input  cluster_axi_req_t   [NrClusters-1:0] axi_req_i,
  output cluster_axi_resp_t  [NrClusters-1:0] axi_resp_o,
  
  // To GLSU
  input  cluster_axi_resp_t   [NrClusters-1:0] axi_resp_i,
  output cluster_axi_req_t  [NrClusters-1:0] axi_req_o
);

// Reshuffle buffer
logic[ClusterAxiDataWidth-1:0] rs_buffer [(NrLanes*1024)/ClusterAxiDataWidth] [NrClusters-1:0];

logic [NrClusters-1:0] rs_r_clusters_valid, rs_w_clusters_valid;

logic [5:0] rs_w_count_q, rs_w_count_d;
logic [5:0] rs_r_count_q, rs_r_count_d;

logic rs_ar_valid_toggle_d, rs_ar_valid_toggle_q;

cluster_metadata_t cluster_metadata_d;



cluster_axi_resp_t  [NrClusters-1:0] axi_resp_d;
cluster_axi_req_t  [NrClusters-1:0] axi_req_d;

assign axi_resp_o = axi_resp_d;
assign axi_req_o = axi_req_d;
assign cluster_metadata_o = cluster_metadata_d;

always_ff @(posedge clk_i or negedge rst_ni) begin
  if(~rst_ni) begin
    rs_w_count_q    <= '0;
    rs_r_count_q    <= '0;
    rs_ar_valid_toggle_q <= '0;
  end else begin
    rs_w_count_q    <= rs_w_count_d;
    rs_r_count_q    <= rs_r_count_d;
    rs_ar_valid_toggle_q <= rs_ar_valid_toggle_d;
  end
end



always_comb begin

  // deffault assignments
  axi_resp_d = axi_resp_i;
  axi_req_d = axi_req_i;
  rs_w_count_d = rs_w_count_q;
  rs_r_count_d = rs_r_count_q;
  rs_ar_valid_toggle_d = rs_ar_valid_toggle_q;
  cluster_metadata_d = cluster_metadata_i;

  if (cluster_metadata_i.is_reshuffle) begin

    axi_req_d='0;
    cluster_metadata_d ='0;

  
    case(cluster_metadata_i.op)

      VSE : begin
        // SEM PROJ :
        // RESHUFFLE STORE MANAGEMENT
        for (int j=0; j<NrClusters; j++) begin
          rs_w_clusters_valid[j]= axi_req_i[j].w_valid;
        end
        // TODO : Maybe set w ready in response to 0 unitl all are valid
        if (rs_w_clusters_valid=='1) begin

          for (int j=0; j<NrClusters; j++) begin
            axi_resp_d[j].r.id   = axi_req_i[j].ar.id;
            rs_buffer[rs_w_count_q][j] = axi_req_i[j].w.data;
            axi_resp_d[j].r.resp = 2'b00;
            axi_resp_d[j].r.user = '0;
            axi_resp_d[j].ar_ready = 1'b1;
            axi_resp_d[j].aw_ready = 1'b1;
            axi_resp_d[j].w_ready = 1'b1;
            axi_resp_d[j].r_valid = 1'b0;
            axi_resp_d[j].r.last = 1'b0;
            axi_resp_d[j].b = '0;
            axi_resp_d[j].w_ready ='1;
            axi_resp_d[j].r.data ='0;

            // Request doesn't propagate to GLSU
            axi_req_d[j] = '0;
          end

          if (rs_w_count_d == (((NrLanes*1024)/ClusterAxiDataWidth)-1))begin
            rs_w_count_d ='0;
            for (int j=0; j<NrClusters; j++) begin
              axi_resp_d[j].b_valid = '1;
            end
          end else begin
            for (int j=0; j<NrClusters; j++) begin
              axi_resp_d[j].b_valid = '0;
            end
            rs_w_count_d = rs_w_count_q + 1;
          end
        end

      end

      VLE: begin

        // RESHUFFLE LOAD MANAGEMENT

        for (int j=0; j<NrClusters; j++) begin
          rs_r_clusters_valid[j]= axi_req_i[j].ar_valid;
        end

        // TODO : Maybe set r ready in response to 0 unitl all are valid
        if (rs_r_clusters_valid == '1) begin
          rs_ar_valid_toggle_d = 1'b1;
        end

        if (rs_ar_valid_toggle_q) begin
          for (int j=0; j<NrClusters; j++) begin
            axi_resp_d[j].r_valid = 1'b1;
            axi_resp_d[j].r.data = rs_buffer[rs_r_count_q][j];
            axi_resp_d[j].r.id   = axi_req_i[j].ar.id;
            axi_resp_d[j].r.resp = 2'b00;
            axi_resp_d[j].r.user = '0;
            axi_resp_d[j].ar_ready = 1'b1;
            axi_resp_d[j].aw_ready = 1'b1;
            axi_resp_d[j].w_ready = 1'b1;
            axi_resp_d[j].b_valid = 1'b0;
            axi_resp_d[j].b = '0;

            // Request doesn't propagate to GLSU
            axi_req_d[j] = '0;

            // Fixed number of transactions since we reshuffle entire vector
            if (rs_r_count_q>=(((NrLanes*1024)/ClusterAxiDataWidth)-1)) begin
              axi_resp_d[j].r.last = 1'b1;
              rs_r_count_d ='0;
              rs_ar_valid_toggle_d = 1'b0;
            end else begin 
              axi_resp_d[j].r.last = 1'b0;
              rs_r_count_d = rs_r_count_q + 1;
            end
          end
        end else begin
          for (int j=0; j<NrClusters; j++) begin
              axi_resp_d[j] = '0;
          end
        end

      end

    endcase

    

  end

  // TODO : REMOVE THIS
  // axi_resp_d = axi_resp_i;
  // axi_req_d = axi_req_i;

end



endmodule