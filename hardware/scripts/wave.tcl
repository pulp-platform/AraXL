# Copyright 2021-2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Author: Matheus Cavalcante <matheusd@iis.ee.ethz.ch>

onerror {resume}
quietly WaveActivateNextPane {} 0

# Add Ariane's waveforms
do ../scripts/wave_core.tcl

# Add Ara's waveforms
for {set cluster 0}  {$cluster < [examine -radix dec ara_tb.NrClusters]} {incr cluster} {
    do ../scripts/wave_ara.tcl $cluster
}

add wave -noupdate -group Global -group ShuffleStage /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/i_shuffle_stage/*
add wave -noupdate -group Global -group LdSt /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/i_global_ldst/*
add wave -noupdate -group Global -group AlignStage /ara_tb/dut/i_ara_soc/i_system/i_ara_cluster/i_align_stage/*

