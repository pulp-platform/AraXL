// Copyright 2024-2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Navaneeth Kunhi Purayil <nkunhi@iis.ee.ethz.ch>
//
// Description:
// Router to move data between different ARA clusters for
// sliding and reduction operations.
// 
// Data received from Cluster on the left is send to the cluster on the right for bypass mode. 
// Otherwise data is send to the Slide Unit of the Cluster.

module ring_router import ara_pkg::*; #(
	localparam int  unsigned DataWidth = $bits(elen_t)
	) (

	input logic clk_i, 
	input logic rst_ni,

	input id_cluster_t cluster_id_i,
	input num_cluster_t num_clusters_i,
	
	// From SLDU in ARA
	input remote_data_t sldu_i,
	input logic sldu_valid_i,
	output logic sldu_ready_o,  

	output remote_data_t sldu_o,
	output logic sldu_valid_o,
	input logic sldu_ready_i,

	// From other ring routers
	input remote_data_t ring_right_i,
	input logic         ring_right_valid_i,
	output logic        ring_right_ready_o,
	
	input remote_data_t ring_left_i,
	input logic         ring_left_valid_i,
	output logic        ring_left_ready_o,

	output remote_data_t ring_right_o,
	output logic         ring_right_valid_o,
	input logic          ring_right_ready_i,

	output remote_data_t ring_left_o,
	output logic         ring_left_valid_o,
	input logic          ring_left_ready_i

	);

	remote_data_t ring_right_inp, ring_left_inp; 
	logic ring_right_ready_out, ring_left_ready_out;
	logic ring_right_valid_inp, ring_left_valid_inp;

	remote_data_t ring_right_out, ring_left_out; 
	logic ring_right_ready_inp, ring_left_ready_inp;
	logic ring_right_valid_out, ring_left_valid_out;

	logic sldu_ready_o_1, sldu_ready_o_2;
	logic ring_right_ready_out_1, ring_right_ready_out_2;
	logic ring_left_ready_out_1, ring_left_ready_out_2;

	spill_register #(
		.T(remote_data_t)
	) i_ring_right_spill_register (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_right_valid_i           ),
		.ready_o(ring_right_ready_o           ),
		.data_i (ring_right_i                 ),

		.valid_o(ring_right_valid_inp         ),
		.ready_i(ring_right_ready_out         ),
		.data_o (ring_right_inp               )
	);

	spill_register #(
		.T(remote_data_t)
	) i_ring_left_spill_register (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_left_valid_i           ),
		.ready_o(ring_left_ready_o           ),
		.data_i (ring_left_i                 ),
		
		.valid_o(ring_left_valid_inp         ),
		.ready_i(ring_left_ready_out         ),
		.data_o (ring_left_inp               )
	);

	spill_register #(
		.T(remote_data_t)
	) i_ring_right_spill_register_o (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_right_valid_out           ),
		.ready_o(ring_right_ready_inp           ),
		.data_i (ring_right_out                 ),
		
		.valid_o(ring_right_valid_o         ),
		.ready_i(ring_right_ready_i         ),
		.data_o (ring_right_o               )
	);

	spill_register #(
		.T(remote_data_t)
	) i_ring_left_spill_register_o (
		.clk_i  (clk_i                        ),
		.rst_ni (rst_ni                       ),
		
		.valid_i(ring_left_valid_out          ),
		.ready_o(ring_left_ready_inp          ),
		.data_i (ring_left_out                ),
		
		.valid_o(ring_left_valid_o            ),
		.ready_i(ring_left_ready_i            ),
		.data_o (ring_left_o                  )
	);
	logic dir_sldu_inp;

	stream_arbiter #(
		.DATA_T(remote_data_t), 
		.N_INP(2),
		.ARBITER("rr")
	) i_ring_left (
		.clk_i(clk_i), 
		.rst_ni(rst_ni),

		.inp_data_i({ring_right_inp, sldu_i}), 
		.inp_valid_i({ring_right_valid_inp & (ring_right_inp.dst_cluster != cluster_id_i), sldu_valid_i & (dir_sldu_inp == 1'b0)}),
		.inp_ready_o({ring_right_ready_out_1, sldu_ready_o_1}),
		
		.oup_data_o (ring_left_out), 
		.oup_valid_o (ring_left_valid_out),
		.oup_ready_i (ring_left_ready_inp)
	);

	stream_arbiter #(
		.DATA_T(remote_data_t), 
		.N_INP(2),
		.ARBITER("rr")
	) i_ring_right (
		.clk_i(clk_i), 
		.rst_ni(rst_ni),

		.inp_data_i({ring_left_inp, sldu_i}), 
		.inp_valid_i({ring_left_valid_inp & (ring_left_inp.dst_cluster!=cluster_id_i), sldu_valid_i & (dir_sldu_inp == 1'b1)}),
		.inp_ready_o({ring_left_ready_out_1, sldu_ready_o_2}),
		
		.oup_data_o (ring_right_out), 
		.oup_valid_o (ring_right_valid_out),
		.oup_ready_i (ring_right_ready_inp)
	);

	stream_arbiter #(
		.DATA_T(remote_data_t), 
		.N_INP(2),
		.ARBITER("rr")
	) i_sldu_o (
		.clk_i(clk_i), 
		.rst_ni(rst_ni),

		.inp_data_i({ring_right_inp, ring_left_inp}), 
		.inp_valid_i({ring_right_valid_inp & (ring_right_inp.dst_cluster==cluster_id_i), ring_left_valid_inp & (ring_left_inp.dst_cluster==cluster_id_i)}),
		.inp_ready_o({ring_right_ready_out_2, ring_left_ready_out_2}),
		
		.oup_data_o (sldu_o),
		.oup_valid_o (sldu_valid_o),
		.oup_ready_i (sldu_ready_i)
	);
    
	// Because a spill register is used ring_right_ready_out_1 and ring_left_ready_out_1 are 1 by default
	// So we also and if the data is valid to get the final ready_out signal
	assign ring_right_ready_out = (ring_right_ready_out_1 & (ring_right_inp.dst_cluster != cluster_id_i)) | 
									ring_right_ready_out_2;
	assign ring_left_ready_out  = (ring_left_ready_out_1 & (ring_left_inp.dst_cluster != cluster_id_i)) | 
									ring_left_ready_out_2;
	assign sldu_ready_o = sldu_ready_o_1 | sldu_ready_o_2;
	assign dir_sldu_inp = find_ring_dir(sldu_i.src_cluster, sldu_i.dst_cluster, (1 << num_clusters_i));

endmodule
