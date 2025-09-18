# Copyright 2021-2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>

add wave -noupdate -group Ara[$1] -group core /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/*

add wave -noupdate -group Ara[$1] -group dispatcher /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_dispatcher/*
add wave -noupdate -group Ara[$1] -group sequencer /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_sequencer/*

# Add waves from all the lanes
for {set lane 0}  {$lane < [examine -radix dec ara_tb.NrLanes]} {incr lane} {
    do ../scripts/wave_lane_ideal.tcl $1 $lane
}

add wave -noupdate -group Ara[$1] -group masku /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_masku/*

add wave -noupdate -group Ara[$1] -group sldu /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_sldu/*

add wave -noupdate -group Ara[$1] -group vlsu -group addrgen /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_vlsu/i_addrgen/*
add wave -noupdate -group Ara[$1] -group vlsu -group vldu /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_vlsu/i_vldu/*
add wave -noupdate -group Ara[$1] -group vlsu -group vstu /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_vlsu/i_vstu/*
add wave -noupdate -group Ara[$1] -group vlsu /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/p_cluster[$1]/i_ara_macro/i_ara/i_vlsu/*
