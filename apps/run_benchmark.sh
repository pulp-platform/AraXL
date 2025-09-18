#!/bin/sh
set -e

app=$1
dtype=$2
latency=$3
path=$4 # mem / cva6 / ring 

logdir=logs

make clean
mkdir -p ${logdir}/$app

for nr_clusters in 16 #2 4 8
do
for nr_lanes in 4
do

#Build hw
cd ../hardware/
make clean && make compile nr_clusters=${nr_clusters} config=${nr_lanes}_lanes ${path}_latency=$latency
cd ../apps/

for bytes_lane in 128 64 32 16 8
do

len=$((bytes_lane * nr_lanes * nr_clusters/ 8))
echo "C=$nr_clusters L=$nr_lanes LEN=$len"

# Benchmark parameters
if [[ $app == "fmatmul" ]]
then
  #args_app="256 256 $len"
  args_app="16 16 $len"
  str_app=FMATMUL
elif [[ $app == "fconv2d" ]]
then
  #args_app="256 $len 7"
  args_app="16 $len 7"
  str_app=FCONV2D
elif [[ $app == "fdotproduct" ]]
then
  args_app="$len"
  str_app=FDOTPRODUCT
elif [[ $app == "jacobi2d" ]]
then
  r=$((len+2))
  #args_app="256 $r"
  args_app="16 $r"
  str_app=JACOBI2D
elif [[ $app == "softmax" ]]
then
  #args_app="64 $len"
  args_app="16 $len"
  str_app=SOFTMAX
elif [[ $app == "exp" ]]
then
  args_app="$len"
  str_app=EXP
else
  echo "SPECIFY app and dtype as 2 arguments to script!"
fi

# Build app
echo "$app"
make $app/data.S def_args_$app="$args_app" nr_clusters=$nr_clusters config=${nr_lanes}_lanes
cp $app/data.S benchmarks/
make bin/benchmarks ENV_DEFINES="-D$str_app -Ddtype=$dtype" nr_clusters=$nr_clusters config=${nr_lanes}_lanes old_data=1

# Simulate
appname=${app}_${nr_clusters}_${nr_lanes}_${bytes_lane}
cp bin/benchmarks bin/${appname}
cp bin/benchmarks.dump bin/${appname}.dump

cd ../hardware/
logfile=../apps/${logdir}/${app}/${nr_lanes}L_${nr_clusters}C_${bytes_lane}B_${latency}${path}.log
make simc app=${appname} nr_clusters=$nr_clusters config=${nr_lanes}_lanes > $logfile &
cd ../apps

done
done
done
