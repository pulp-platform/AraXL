#!/bin/sh

## To run RiVEC benchmarks
# Usage: ./run_rivec.sh <app> <nr_clusters> <path> <latency>
# Example: ./run_rivec.sh _pathfinder 4 mem 0
set -e

app=$1
nr_clusters=$2
path=$3 # mem / cva6 / ring
latency=$4

logdir=logs
nr_lanes=4

# set parameters
if [[ $app == "_axpy" ]]
then
args_app=4096
app_size=4096
elif [[ $app == "_blackscholes" ]]
then
args_app="input/in_16K.input"
app_size=16K
elif [[ $app == "_pathfinder" ]]
then
args_app="input/data_medium.in"
app_size="medium"
elif [[ $app == "_streamcluster" ]]
then
args_app="3 3 128 8 8 10"
app_size=3_3_128_8_8_10
elif [[ $app == "_spmv" ]]
then
args_app="input/lhr07.mtx"
app_size=lhr07
elif [[ $app == "_canneal" ]]
then
args_app="input/100.nets"
app_size=100
elif [[ $app == "_swaptions" ]]
then
args_app="128 1"
app_size=128_1
elif [[ $app == "_lavaMD" ]]
then
args_app="1 1 32"
app_size=1_1_32
else
  echo "RiVEC app not supported!!"
fi

mkdir -p ${logdir}/$app

# Compile apps
make bin/${app} nr_clusters=${nr_clusters} def_args_$app="$args_app" -B

# Run simulation
cd ../hardware/
logfile=../apps/${logdir}/${app}/${nr_lanes}L_${nr_clusters}C_${app_size}_${latency}${path}.log
make sim nr_clusters=${nr_clusters} app=${app} ${path}_latency=$latency -B > $logfile &

cd ../apps/