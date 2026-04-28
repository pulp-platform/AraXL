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

#Build hw
cd ../hardware/
make compile nr_clusters=${nr_clusters} config=${nr_lanes}_lanes ${path}_latency=$latency -B
cd ../apps/

for bytes_lane in 512 #256 128 64 32 16 8
do

len=$((bytes_lane * nr_lanes * nr_clusters/ 8))

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
args_app="3 3 ${len} 8 8 10"
app_size=3_3_${len}_8_8_10
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
args_app="${len} 1"
app_size=${len}_1
elif [[ $app == "_lavaMD" ]]
then
len=$((len * 2))
args_app="1 1 ${len}"
app_size=1_1_${len}
else
  echo "RiVEC app not supported!!"
fi

mkdir -p ${logdir}/$app

# Compile apps
make bin/${app} nr_clusters=${nr_clusters} def_args_$app="$args_app" -B
appname=${app}_${nr_clusters}_${nr_lanes}_${bytes_lane}
cp bin/${app} bin/${appname}
cp bin/${app}.dump bin/${appname}.dump

# Run simulation
cd ../hardware/
logfile=../apps/${logdir}/${app}/${nr_lanes}L_${nr_clusters}C_${bytes_lane}B_${latency}${path}.log
make simc nr_clusters=${nr_clusters} app=${appname} > $logfile &

cd ../apps/
done