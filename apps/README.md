# Apps

This folder contains the benchmarks, programs, and tests ready to be run on AraXL.
All software sources are licensed under [Apache 2.0](../LICENSE.sw).

SW utilities and benchmarks for Ara using external contributions,
* `RiVEC` - Use of the RISC-V VECTOR intrinsics mapping `common/rivec/vector_defines.h` and benchmarks `apps/cos`, `apps/log`, `apps/exp` by Cristóbal Ramírez Lazo, "Barcelona 2019" under [license](common/rivec/LICENSE)
* `print` - `common/printf.c`, `common/printf.h` Print primitives for embedded systems under the MIT License (MIT)
* `common/encoding.h` - Copyright (c) 2010-2017, The Regents of the University of California

* `dwt` - Scalar version ported to RVV environment for Ara
* `jacobi2d` - Modified vector version from PolyBench/C and then RiVEC benchmark suites
* `pathfinder` - Modified vector version from RODINIA and then RiVEC benchmark suites
* `roi_align` - Porting to RVV environment for Ara from the Original implementation taken from https://github.com/longcw/RoIAlign.pytorch
* `softmax` - Scalar implmentation inspired by OpenCV softmax https://github.com/opencv/opencv/blob/master/modules/dnn/src/layers/softmax_layer.cpp

### Hello World
Run the following command to build an application. E.g., `hello_world`:

```bash
cd apps
make bin/hello_world
```

### Convolutions

Convolutions allow to specify the output matrix size and the size of the filter, with the variables `OUT_MTX_SIZE` up to 112 and `F_SIZE` within {3, 5, 7}. Currently, not all the configurations are supported for all the convolutions. For more information, check the `main.c` file for the convolution of interest.
Example:

```bash
cd apps
make bin/fconv2d OUT_MTX_SIZE=112 F_SIZE=7
```

### Standard RISC-V tests

To compile the standardized riscv tests
```bash
cd apps
make riscv_tests_compile
```
