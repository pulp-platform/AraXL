# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

"""Exercise CI isolation and failure handling without compiling whole models."""

import csv
import fcntl
import io
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


RUNNER = Path(__file__).resolve().with_name("run_ci.py")


class ApplicationCI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / "repo"
        (self.repo / "apps" / "bin").mkdir(parents=True)
        scripts = self.repo / "hardware" / "scripts" / "regressions"
        scripts.mkdir(parents=True)
        self.runner = scripts / "run_ci.py"
        shutil.copyfile(RUNNER, self.runner)
        self.logs = self.repo / "hardware" / "build" / "ci"
        (self.repo / "config").mkdir()
        (self.repo / "apps" / "source").write_text("local changes\n")
        (self.repo / "apps" / "bin" / "ok").write_text("stale binary\n")
        self.simulation = '''#!/usr/bin/env python3
import sys,time
from pathlib import Path
args=sys.argv[1:]
if '-l' in args and args[args.index('-l')+1].startswith('ram,'):
    binary=Path(args[args.index('-l')+1].split(',')[1])
    expected=Path(__file__).resolve().parents[1].joinpath('clusters').read_text()
    print('Verilator cycle limit: '+args[args.index('-c')+1])
else:
    binary=Path(next(a.split('=',1)[1] for a in args if a.startswith('+PRELOAD=')))
    ini=Path(args[args.index('-modelsimini')+1]).read_text()
    work=Path(next(s.split(' = ',1)[1] for s in ini.splitlines() if s.startswith('work = ')))
    expected=(work.parent/'clusters').read_text()
    tcl=Path(args[args.index('-do')+1]).read_text()
    print('Questa time limit: '+next(line[4:] for line in tcl.splitlines() if line.startswith('run ')))
if binary.read_text()!=expected: sys.exit(9)
name=binary.name
if not Path('trace_hart_0.log').is_symlink(): sys.exit(10)
Path('trace_hart_0.log').write_text('discard scalar trace')
Path('unexpected_trace.csv').write_text('temporary trace')
Path('../gold_results.txt').write_text('discard data dump')
if name=='hang': time.sleep(5)
if name=='silent': sys.exit(0)
if name=='limit': print('Simulation timeout of 100 cycles'); sys.exit(0)
print('Core Test *** SUCCESS *** (tohost = 0)')
if name=='fatal': print('** Fatal: expected failure')
sys.exit(3 if name=='bad_exit' else 0)
'''
        self.questa = self.root / "questa"
        self.questa.write_text(self.simulation)
        self.questa.chmod(0o755)
        self.make = self.root / "make"
        self.make.write_text('''#!/usr/bin/env python3
import sys
from pathlib import Path
args=sys.argv[1:]
directory=Path(args[args.index('-C')+1])
values=dict(a.split('=',1) for a in args if '=' in a)
clusters=values['nr_clusters']
if directory.name=='apps':
    (directory/'bin').mkdir(exist_ok=True)
    for target in args:
        if target.startswith('bin/'):
            (directory/target).write_text(clusters)
    (directory/'source').write_text('generated in copied workspace')
else:
    build=Path(values['buildpath']); build.mkdir(parents=True)
    (build/'clusters').write_text(clusters)
    (build/'work').mkdir()
    (build/'modelsim.ini').write_text('[Library]\\n')
    (build/('compile_'+values['config']+'.tcl')).write_text('compiled')
    if 'verilate' in args:
        (build/'verilator').mkdir()
        model=build/'verilator'/'Vara_tb_verilator'
        model.write_text(Path(__file__).with_name('questa').read_text())
        model.chmod(0o755)
''')
        self.make.chmod(0o755)

    def invoke(self, simulator, stage="all", clusters=(2,), rvv=("ok",), benchmarks=("ok",),
               cwd=None, extra=()):
        command = [sys.executable, str(self.runner), "--stage", stage, "--sim", simulator,
                   "--config", "default", "--lanes", "4",
                   "--vlen", "4096", "--make", str(self.make), "--questa-cmd", str(self.questa),
                   "--clusters", *map(str, clusters), *extra,
                   "--rvv", *rvv, "--benchmarks", *benchmarks]
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=20, cwd=cwd or self.repo,
                                env=dict(os.environ, TMPDIR=str(self.root)))
        rows = []
        header = "simulator\tclusters\tsuite\tapplication\tresult\texit_code\n"
        for log in sorted(self.logs.glob(f"{simulator}-*/run.log")):
            records = [line[len("RESULT\t"):] + "\n" for line in log.read_text().splitlines()
                       if line.startswith("RESULT\t")]
            rows.extend(csv.DictReader(io.StringIO(header + "".join(records)), delimiter="\t"))
        return result, rows

    def test_matrix_keeps_models_binaries_and_runs_separate(self):
        for simulator in ("verilator", "questa"):
            with self.subTest(simulator=simulator):
                if simulator == "verilator":
                    limit_args = ("--verilator-max-cycles", "12345")
                    limit_output = "Verilator cycle limit: 12345"
                else:
                    limit_args = ("--questa-sim-time", "77us")
                    limit_output = "Questa time limit: 77us"
                result, rows = self.invoke(simulator, clusters=(2, 4, 8), extra=limit_args)
                self.assertEqual(result.returncode, 0, result.stdout)
                self.assertEqual(len(rows), 6)
                self.assertEqual({r["clusters"] for r in rows}, {"2", "4", "8"})
                self.assertTrue(all(r["result"] == "PASSED" for r in rows))
                expected = {f"{simulator}-default-c{c}-l4-v4096/run.log" for c in (2, 4, 8)}
                files = {str(p.relative_to(self.logs))
                         for p in self.logs.glob(f"{simulator}-*/*") if p.is_file()}
                self.assertEqual(files, expected)
                for clusters in (2, 4, 8):
                    log = self.logs / f"{simulator}-default-c{clusters}-l4-v4096/run.log"
                    self.assertIn(f"nr_clusters={clusters}", log.read_text())
                    self.assertIn("=== rvv: ok ===", log.read_text())
                    self.assertIn("=== benchmarks: ok ===", log.read_text())
                    self.assertIn(limit_output, log.read_text())
                self.assertEqual(list(self.root.glob("araxl-ci-*")), [])
                self.assertEqual((self.repo / "apps/source").read_text(), "local changes\n")
                self.assertEqual((self.repo / "apps/bin/ok").read_text(), "stale binary\n")

    def test_exit_completion_errors_and_timeouts(self):
        for simulator in ("verilator", "questa"):
            with self.subTest(simulator=simulator):
                result, rows = self.invoke(simulator, stage="rvv",
                    rvv=("ok", "bad_exit", "silent", "fatal", "limit", "hang"),
                    extra=("--timeout", "1"))
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual([r["result"] for r in rows],
                                 ["PASSED", "FAILED", "FAILED", "FAILED", "TIMEOUT", "TIMEOUT"])
                self.assertEqual(rows[-1]["exit_code"], "124")
                self.assertEqual([p.name for p in self.logs.glob(f"{simulator}-*/*")
                                  if p.is_file()], ["run.log"])
                self.assertEqual(list(self.root.glob("araxl-ci-*")), [])

    def test_skip_build_requires_matching_configuration(self):
        result, _ = self.invoke("verilator", stage="build")
        self.assertEqual(result.returncode, 0, result.stdout)
        result, _ = self.invoke("verilator", stage="rvv", extra=("--skip-build",))
        self.assertEqual(result.returncode, 0, result.stdout)
        logs = self.logs / "verilator-default-c2-l4-v4096"
        self.assertEqual([p.name for p in logs.iterdir()], ["run.log"])
        result, _ = self.invoke("verilator", stage="rvv", clusters=(4,), extra=("--skip-build",))
        self.assertEqual(result.returncode, 2)
        self.assertIn("no matching compiled configuration", result.stdout)

    def test_generated_data_cannot_follow_symlinks_back_to_the_workspace(self):
        shared = self.repo / "shared-data"
        shared.write_text("preserve this input")
        source = self.repo / "apps/source"
        source.unlink()
        source.symlink_to(shared)
        result, _ = self.invoke("verilator", stage="build")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(shared.read_text(), "preserve this input")

    def test_paths_are_independent_of_the_working_directory(self):
        for simulator in ("verilator", "questa"):
            for count, cwd in enumerate((self.repo, self.repo / "hardware", self.root), 1):
                with self.subTest(simulator=simulator, cwd=cwd):
                    result, rows = self.invoke(simulator, stage="rvv", cwd=cwd)
                    self.assertEqual(result.returncode, 0, result.stdout)
                    self.assertEqual(len(rows), count)
                    self.assertTrue(all(row["result"] == "PASSED" for row in rows))
        self.assertFalse((self.root / "hardware").exists())
        self.assertFalse((self.repo / "hardware" / "hardware").exists())

    def test_build_setup_failure_is_reported_for_each_configuration(self):
        (self.repo / "apps/missing-input").symlink_to(self.repo / "missing")
        result, rows = self.invoke("verilator", clusters=(2, 4))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual([r["clusters"] for r in rows], ["2", "4"])
        self.assertTrue(all(r["suite"] == "build" and r["result"] == "FAILED" for r in rows))

    def test_repeated_runs_append_to_one_log(self):
        result, _ = self.invoke("verilator")
        self.assertEqual(result.returncode, 0, result.stdout)
        folder = self.logs / "verilator-default-c2-l4-v4096"
        previous = (folder / "run.log").read_bytes()
        result, rows = self.invoke("verilator")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(len(rows), 4)
        self.assertTrue((folder / "run.log").read_bytes().startswith(previous))
        self.assertEqual([p.name for p in folder.iterdir()], ["run.log"])

    def test_running_configuration_log_is_untouched(self):
        folder = self.logs / "verilator-default-c2-l4-v4096"
        folder.mkdir(parents=True)
        log = folder / "run.log"
        log.write_text("active run output\n")
        with log.open("a") as active:
            fcntl.flock(active, fcntl.LOCK_EX)
            result, rows = self.invoke("verilator")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already running", result.stdout)
        self.assertEqual(rows, [])
        self.assertEqual(log.read_text(), "active run output\n")


if __name__ == "__main__":
    unittest.main()
