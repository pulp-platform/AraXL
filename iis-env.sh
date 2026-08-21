[ -d /usr/pack/riscv-1.0-kgf/efclschool-llvm-18.1.6 ] || /usr/sepp/bin/riscv-1.0-kgf true
[ -d /usr/pack/verilator-5.020-zr ] || /usr/sepp/bin/verilator-5.020 verilator --version >/dev/null 2>&1

_riscv_pack=/usr/pack/riscv-1.0-kgf

export GCC_INSTALL_DIR=$_riscv_pack/riscv64-2023.03.14
export LLVM_INSTALL_DIR=$_riscv_pack/efclschool-llvm-18.1.6
export LLVM_FLAGS="--target=riscv64-unknown-elf --sysroot=$GCC_INSTALL_DIR/riscv64-unknown-elf --gcc-toolchain=$GCC_INSTALL_DIR -march=rv64gcv_zfh_zvfh -mabi=lp64d -mno-relax -fuse-ld=lld"
export RISCV_SIM="$_riscv_pack/riscv-isa-sim-for-cva6/spike"

export ISA_SIM_INCLUDE=$GCC_INSTALL_DIR/include
export VERILATOR_INCLUDE=/usr/pack/verilator-5.020-zr/verilator-5.020/include/vltstd
export veril_path=/usr/pack/verilator-5.020-zr/verilator-5.020/bin
export CXX=/usr/bin/g++

unset _riscv_pack
