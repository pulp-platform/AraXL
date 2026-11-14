# Contributor workflow for RTL changes

These guidelines apply to human contributors and automated coding assistants. Use the repository's build and regression targets with a supported simulator and a compatible toolchain.

## Preserve the reproduction

- Read `git status --short` and the relevant diff before making changes. Preserve unrelated work and the configuration used to reproduce the failure.
- Record the source revision and local diff, application binary checksum, lanes per cluster, cluster count, VLEN, simulator/compiler versions, and exact build/run commands. Include core and memory-bank counts where applicable.
- Confirm that the application binary matches the current source and hardware configuration. Make may reuse stale application objects or compiled RTL when configuration variables change; rebuild explicitly or use separate build directories.
- Save failing logs, waveforms, and traces before rebuilding or rerunning. Compilation, simulation, and waveform viewing can overwrite fixed output filenames.
- Keep machine-specific paths, transient artifact locations, and session results in local reproduction notes. Repository guidance should refer to maintained sources and reproducible commands.

## Keep builds and runs isolated

- Use a separate build directory and simulation working directory for each configuration. Keep generated files out of source directories.
- Use compatible simulator versions for compilation, elaboration, simulation, and waveform inspection. A compiled-library version mismatch is an environment problem, not evidence of an RTL syntax error.
- Preserve configuration defines, include directories, package ordering, and library settings during incremental compilation. Recompile all producers and consumers affected by an interface change.
- After rebasing or updating dependencies, check interface references against the current package and type definitions.
- Use explicit library, application, and output paths when running outside the normal build directory. Give waveform inspection a separate log path as well.
- Do not recompile a library while a simulation is using it. Preserve artifacts from existing or active runs.
- Use a compiler and runtime libraries compatible with generated simulation code. Compiler overrides must also account for cached compiler-specific flags. Configure tool caches in writable locations when needed.

## Bound execution and verify completion

- Set both a simulated-time or cycle limit and a wall-clock timeout. A deadlock must produce a failed regression rather than run indefinitely.
- Treat assertion failures, simulator errors, timeouts, and missing completion markers as failures, even if the process returns zero.
- Check the expected completion marker for the test. Application tests commonly report `Core Test *** SUCCESS *** (tohost = 0)`; other testbenches may use their own completion protocol.
- Check expected case coverage as well as the final status. A few printed `PASSED` messages do not prove that the entire application completed.
- Review warnings. Keep suppressions specific to understood, pre-existing issues; do not hide new latches, combinational loops, width errors, or interface mismatches.
- If an environment or license limitation prevents a required run, report the missing validation and use an available supported backend for the checks it can perform. Compilation does not establish full application correctness.

## Run focused regressions, then the application

Regression runners and their Python tests are in `hardware/scripts/regressions/`. Select the simulator through `sim`; for example, run the focused RTL suite with either backend:

```bash
make ci-tb sim=questa
```

The focused testbenches cover four lanes per cluster with 2, 4, 8, and 16 clusters internally. The shared registry and individual-test runner is `hardware/scripts/regressions/run_tb_test.sh`:

```bash
bash hardware/scripts/regressions/run_tb_test.sh --list
```

After focused tests pass, rerun the original failing application in an isolated directory, then the relevant wider regression. The complete CI target runs focused testbenches before application regressions:

```bash
make ci sim=questa cluster_configs="2 4 8"
```

Use the selected backend consistently. The runner's Python tests can be checked separately:

```bash
python3 -m unittest discover -s hardware/scripts/regressions
```

These Python tests validate regression orchestration; they do not replace RTL simulation. A unit test of one module does not validate the complete load/store, shuffle, alignment, interconnect, and scalar-core integration.

### Maintained testbenches

Sources are in `hardware/tb/unit/`. Add useful reproductions to the maintained suite; temporary test sources alone do not provide durable regression coverage.

| Test | Main checks |
| --- | --- |
| `align_transition` | Unit-stride/indexed response transitions, retirement of the previous instruction's final beat, and downstream backpressure. |
| `axi_inval_filter` | Per-burst invalidation coverage, repeated cache sets, line crossings, queued addresses, backpressure, and disabled coherence. |
| `indexed_read` | Response stability, selected-cluster readiness, element order, partial final groups, and handshake conservation. |
| `mask_transfer` | Mask-bit participation, inactive clusters, address barriers, data conservation, and backpressure. |
| `reshuffle_dispatcher` | Reshuffle decisions across cluster counts, element widths, and partial versus complete register overwrites. |
| `store_transition` | Queued unit-stride/indexed stores, width changes, tail preservation, data/strobe correctness, and backpressure. |

## Exercise handshake and metadata boundaries

For handshake or metadata changes, instantiate the modified RTL with its real packages and dependencies. Use assertions or explicit failures and a testbench timeout. Include the relevant scenarios:

- Delay the last participating cluster's address request while earlier clusters wait.
- Stall downstream address and data channels, then release them.
- Change source metadata after an address enters a pipeline cut while the request remains queued downstream.
- Queue stores with different operations and element widths, then drain the older data packet after the newer address has arrived.
- Allow write data before downstream write-address acceptance. Avoid introducing a dependency that requires address readiness before data validity can be asserted.
- Exclude clusters with zero local VL from participation barriers without consuming requests belonging to later instructions.
- Exercise partial final lane groups and consecutive operations without adding fences solely to hide a failure.

Check address/data conservation at the same interface: read and write beat counts must agree with the accepted burst lengths, and valid payloads must remain stable under backpressure. Check tracker bounds and ensure inactive clusters cannot acknowledge data. Use legal AXI transfer sizes, including narrow transfers when a bus is wider than the protocol's maximum encoded beat size.

Retain these invariants when changing the relevant paths:

- Derive transfer mode from the instruction owning buffered data until its final beat retires.
- Advance response selectors and counters only on accepted transfers. Indexed response readiness must follow the selected recipient.
- Establish invalidation coverage per burst; counts from unrelated bursts do not prove unique cache-set coverage.
- Preserve the original global mask-bit count separately from the rounded byte-transfer length when determining participating clusters.
- Distinguish memory ordering from cache invalidation. Shared data between non-coherent agents requires the platform's documented cache-maintenance behavior.

## Cover configuration and vector-length boundaries

Build and run each relevant hardware configuration separately. Changing a software VL list does not change the instantiated hardware.

For four lanes per cluster with 2, 4, 8, or 16 clusters, cover:

- Lane boundaries: VL 1, 3, 4, and 5.
- Full-cluster-round boundaries: one below, equal to, and one above `4 * NrClusters`.
- Repeated rounds and tails: values around 128 and 192, plus VL 127, 254, and 255 where supported.
- Data and index widths of 8, 16, 32, and 64 bits, including unequal widths where relevant.
- Consecutive operations with different widths, masked operations, and nonzero `vstart` where supported.

Ensure the requested VL fits VLMAX for the selected SEW, LMUL, and configuration, or read the actual VL and construct expectations from it. Keep indexed byte offsets representable in the index element width. Use deterministic data, check every active element, and use sentinels to detect unintended writes outside the active range.

## Debug the first incorrect transfer

- Use the scalar instruction trace to identify the active instruction, then correlate per-cluster load/store progress. Local trace VL values may differ from the global instruction metadata.
- Inspect VALID and READY together at each boundary, along with payload metadata, queue occupancy, counters, and selectors.
- Verify timestamp units and precision before correlating text traces with waveforms; raw numeric timestamps may use different units.
- Find the earliest incorrect handshake. A later stuck selector or tracker underflow can be a consequence of an earlier lost or misrouted beat.
- Cross-check diagnostic messages against the actual loads and stores. Separate volatile reads can return different values; inspect outgoing data and cache invalidations before attributing a comparison failure to transfer ordering.

## Report validation precisely

State what compiled, which directed tests and hardware configurations passed, and whether the full failing application reached its expected successful completion. Record the commands and evidence in the run notes. Keep untested configurations and unverified integration behavior explicit, and distinguish environment failures from design failures.
