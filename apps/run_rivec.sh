#!/bin/sh

## To run RiVEC benchmarks
# Usage: ./run_rivec.sh <app> <path> <latency>
# Example: ./run_rivec.sh _pathfinder mem 0
set -e

app=$1
path=$2 # mem / cva6 / ring
latency=$3

logdir=logs
nr_lanes=4

for nr_cores in 4
do
for nr_clusters in 4
do

# for nr_cores in 1
# do
# for nr_clusters in 16
# do

#Build hw
cd ../hardware/
make compile nr_cores=${nr_cores} nr_clusters=${nr_clusters} config=${nr_lanes}_lanes ${path}_latency=$latency -B
cd ../apps/

for bytes_lane in 512 256 128 64 32 16 8
do

len=$((bytes_lane * nr_lanes * nr_clusters * nr_cores / 8))

# set parameters
if [[ $app == "_axpy" ]]
then
args_app="$len"
app_size="$len"
elif [[ $app == "_blackscholes" ]]
then
len=$((len * 2))
args_app="input/in_${len}.input"
app_size="$len"
elif [[ $app == "_pathfinder" ]]
then
len=$((len * 2))
args_app="input/data_medium.in $len"
app_size="medium_${len}"
elif [[ $app == "_streamcluster" ]]
then
len=$((len * 2))
args_app="3 10 ${len} 128 128 10"
app_size=3_10_${len}_128_128_10
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
nr_swaptions=${nr_cores}
args_app="${len} ${nr_swaptions}"
app_size=${len}_${nr_swaptions}
elif [[ $app == "_lavaMD" ]]
then
len=$((len * 2))
boxes_1d=2
args_app="${nr_cores} ${boxes_1d} ${len}"
app_size=${nr_cores}_${boxes_1d}_${len}
else
  echo "RiVEC app not supported!!"
fi

mkdir -p ${logdir}/$app

# Compile apps
make bin/${app} nr_cores=${nr_cores} nr_clusters=${nr_clusters} def_args_$app="$args_app" -B
appname=${app}_${nr_cores}_${nr_clusters}_${nr_lanes}_${bytes_lane}
cp bin/${app} bin/${appname}
cp bin/${app}.dump bin/${appname}.dump

# Run simulation
cd ../hardware/
logfile=../apps/${logdir}/${app}/${nr_lanes}L_${nr_clusters}C_${nr_cores}core_${bytes_lane}B_${latency}${path}.log
# logfile=../apps/${logdir}/${app}/${nr_lanes}L_${nr_clusters}C_${bytes_lane}B_${latency}${path}.log
make simc app=${appname} nr_cores=${nr_cores} nr_clusters=$nr_clusters config=${nr_lanes}_lanes > $logfile &
# make sim nr_cores=${nr_cores} nr_clusters=${nr_clusters} app=${appname}

cd ../apps/
done
done
done
