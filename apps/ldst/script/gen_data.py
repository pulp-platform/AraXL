# Copyright 2024-2025 ETH Zurich and University of Bologna.
#
# SPDX-License-Identifier: Apache-2.0
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
#
# Author : Navaneeth Kunhi Purayil (nkunhi@iis.ee.ethz.ch)
#

import numpy as np
import random
from functools import reduce
import sys

def emit(name, array, alignment='8'):
  print(".global %s" % name)
  print(".balign " + alignment)
  print("%s:" % name)
  bs = array.tobytes()
  for i in range(0, len(bs), 4):
    s = ""
    for n in range(4):
      s += "%02x" % bs[i+3-n]
    print("    .word 0x%s" % s)

# Vector length
if len(sys.argv) > 1:
  vsize = int(sys.argv[1])
else:
  # Default: no stripmine
  vsize = 64

avl = int(vsize) + 10  #adding 10 to test unaligned 
dtype = np.float64
# dtype = np.float32
# dtype = np.float16

# Create the vectors
va = np.random.rand(avl).astype(dtype)
vb = np.random.rand(avl).astype(dtype)
vc = np.random.rand(avl).astype(dtype)
vres = np.random.rand(avl).astype(dtype)

# Print information on file
print(".section .data,\"aw\",@progbits")
emit("vsize", np.array(vsize, dtype=np.uint64))

# For mask test
vres = np.minimum(va, vb)

# For reduction test
red = sum(va[0:vsize])

emit("va", va, 'NR_LANES*NR_CLUSTERS*4')
emit("vb", vb, 'NR_LANES*NR_CLUSTERS*4')
emit("vc", vc, 'NR_LANES*NR_CLUSTERS*4')
emit("vres", vres, 'NR_LANES*NR_CLUSTERS*4')

if (dtype == np.float16):
  emit("red", np.array(red, dtype=np.float32))
else:
  emit("red", np.array(red, dtype=dtype))