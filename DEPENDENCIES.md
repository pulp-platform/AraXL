# Dependencies

AraXL has strong and weak dependencies on several packages. Such dependencies and their licenses are listed in this document.

## Hardware dependencies

AraXL needs the following hardware packages to work.

- [apb](https://github.com/pulp-platform/apb)
- [cva6](https://github.com/pulp-platform/cva6)
- [axi](https://github.com/pulp-platform/axi)
- [common\_cells](https://github.com/pulp-platform/common_cells)
- [common\_verification](https://github.com/pulp-platform/common_verification)
- [fpnew](https://github.com/openhwgroup/cvfpu)
- [fpu_div_sqrt_mvp](https://github.com/pulp-platform/fpu_div_sqrt_mvp)
- [tech\_cells\_generic](https://github.com/pulp-platform/tech_cells_generic)

All of them are licensed under the Solderpad 0.51 license.

## Software

In order to compile the benchmarks, you will need the RISC-V GCC toolchain with support for the Vector Extension, version 0.9.
This is included as a submodule (`toolchain/riscv-gnu-toolchain`).

Unit tests for the vector instructions are given in a patched version of the [riscv\_tests](https://github.com/riscv/riscv-tests/) repository (`apps/riscv-tests`).
The riscv-tests repository is licensed under the BSD license.

The unit tests can also run on Spike, the RISC-V ISA Simulator, which is also included as a submodule (`toolchain/riscv-isa-sim`).
This version of Spike is patched to align the behavior of the `vcsr` CSR with the toolchain and with RVV v0.9.

We provide a Python script to run `clang-format` and format the C and C++ files of this repository (`scripts/run-clang-format.py`).
This file is licensed under the MIT license.

`conjugate_gradient` needs python package `sklearn` for generating symmetric and positivie-defined matrix
