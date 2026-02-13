# Copyright 2026 ETH Zurich and University of Bologna.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Navaneeth Kunhi Purayil, ETH Zurich (nkunhi@iis.ee.ethz.ch)

RIVEC_PATH := riscv-vectorized-benchmark-suite
RIVEC_DIR := $(APPS_DIR)/$(RIVEC_PATH)
RIVEC_APPS := _blackscholes
RIVEC_BINARIES := $(addprefix bin/, $(RIVEC_APPS))

rivec_binaries: $(RIVEC_BINARIES)

# Adding compile flags for RiVEC

RISCV_CCFLAGS += -I$(RIVEC_DIR) -DUSE_RISCV_VECTOR
RISCV_CXXFLAGS += -I$(RIVEC_DIR) -DUSE_RISCV_VECTOR

define rivec_gen_data_template
.PHONY: $1/data.S
$1/data.S:
	cd $1 && if [ -d script ]; then ${PYTHON} script/gen_data.py $(subst ",,$(def_args_$1)) > data.S ; else touch data.S; fi
endef
$(foreach app,$(RIVEC_APPS),$(eval $(call rivec_gen_data_template, $(RIVEC_PATH)/$(app))))

define rivec_compile_template
bin/$1: $(RIVEC_PATH)/$1/data.S.o $(addsuffix .o, $(shell find $(RIVEC_DIR)/$(1) -name "*.c" -o -name "*.S" -o -name "*.cpp")) $(RUNTIME_LLVM) linker_script
	mkdir -p bin/
	$$(RISCV_CC) $(RISCV_CCFLAGS) -o $$@ $$(addsuffix .o, $$(shell find $(RIVEC_DIR)/$(1) -name "*.c" -o -name "*.S" -o -name "*.cpp")) $(RUNTIME_LLVM) $$(RISCV_LDFLAGS) -T$$(CURDIR)/common/link.ld
	$$(RISCV_OBJDUMP) $$(RISCV_OBJDUMP_FLAGS) -D $$@ > $$@.dump
	$$(RISCV_STRIP) $$@ -S --strip-unneeded
endef
$(foreach app,$(RIVEC_APPS),$(eval $(call rivec_compile_template,$(app))))
