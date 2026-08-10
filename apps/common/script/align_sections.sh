#!/usr/bin/env bash

# Takes as input the number of lanes ($1), the number of clusters ($2), the
# number of word-interleaved L2 banks ($3) and the linker script to process ($4)
# Align the sections by AxiWideBeWidth
# NB: this script modify ALL the ALIGN directives
# let ALIGNMENT=4096 # 4*$1*$2 # TODO: Should correct this once addrgen is moved outside of clusters.
let ALIGNMENT=4*$1*$2 # TODO: Should correct this once addrgen is moved outside of clusters.
sed -i "s/ALIGNMENT/$ALIGNMENT/g" $4

# Usable L2 spans every bank: the MEMORY LENGTH expression in arch.link.ld is
# NRL2BANKS * L2NumWords * ALIGNMENT - 2048.
sed -i "s/NRL2BANKS/$3/g" $4
