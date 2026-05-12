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

TSVC_PATH := TSVC_2
TSVC_DIR := $(APPS_DIR)/$(TSVC_PATH)
TSVC_APPS := tsvc
TSVC_BINARIES := $(addprefix bin/, $(TSVC_APPS))

tsvc_binaries: $(TSVC_BINARIES)

# TSVC-specific compile flags: enable RVV auto-vectorization.
# The shared RISCV_CCFLAGS has -scalable-vectorization=off (for intrinsic-based apps).
# TSVC is an auto-vectorization benchmark, so we override those flags here.
TSVC_V_FLAGS   := -fvectorize -mllvm -scalable-vectorization=preferred \
                  -mllvm -riscv-v-vector-bits-min=$(vlen) \
				  -Rpass=loop-vectorize -Rpass-missed=loop-vectorize -Rpass-analysis=loop-vectorize
TSVC_CCFLAGS   := $(LLVM_FLAGS) $(TSVC_V_FLAGS) \
                  -mcmodel=medany -I$(CURDIR)/common -I$(TSVC_DIR) \
                  -O3 -ffast-math -fno-common -fno-builtin-printf \
                  $(DEFINES) $(RISCV_WARNINGS) \
                  -ffunction-sections -fdata-sections

define tsvc_gen_data_template
.PHONY: $1/data.S
$1/data.S:
	cd $1 && if [ -d script ]; then ${PYTHON} script/gen_data.py $(strip $(subst ",,$(or $(def_args_$(notdir $1)),$(def_args_$(patsubst _%,%,$(notdir $1)))))) > data.S ; else touch data.S; fi
endef
$(foreach app,$(TSVC_APPS),$(eval $(call tsvc_gen_data_template, $(TSVC_PATH)/$(app))))

define tsvc_compile_template
# Pattern rule to compile TSVC sources with auto-vectorization flags
$(TSVC_DIR)/src/%.c.o: $(TSVC_DIR)/src/%.c
	$$(RISCV_CC) $$(TSVC_CCFLAGS) -c $$< -o $$@

bin/$1: $(addsuffix .o, $(shell find $(TSVC_DIR)/src -name "*.c" -o -name "*.S" -o -name "*.cpp")) $(RUNTIME_LLVM) linker_script
	mkdir -p bin/
	$$(RISCV_CC) $$(TSVC_CCFLAGS) -o $$@ $$(addsuffix .o, $$(shell find $(TSVC_DIR)/src -name "*.c" -o -name "*.S" -o -name "*.cpp")) $(RUNTIME_LLVM) $$(RISCV_LDFLAGS) -T$$(CURDIR)/common/link.ld
	$$(RISCV_OBJDUMP) $$(RISCV_OBJDUMP_FLAGS) -D $$@ > $$@.dump
	$$(RISCV_STRIP) $$@ -S --strip-unneeded
endef
$(foreach app,$(TSVC_APPS),$(eval $(call tsvc_compile_template,$(app))))
