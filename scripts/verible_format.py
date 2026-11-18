#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51

"""Install commit-formatting tools, or invoke Verible with the supplied arguments."""

import hashlib
import io
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import venv


ROOT = Path(__file__).resolve().parents[1]
VERIBLE_VERSION = "v0.0-4298-g085eef05"
PRE_COMMIT_VERSION = "4.3.0"
TOOLS = ROOT / ".tools"
FORMATTER = TOOLS / "verible" / VERIBLE_VERSION / "bin" / "verible-verilog-format"
PRE_COMMIT_PYTHON = TOOLS / "pre-commit" / "bin" / "python"
ASSETS = {
    ("Linux", "x86_64"): ("linux-static-x86_64", "ca83778c7c2b73272277bd670d1851c8caf973fd3283688aeb0d36d4b3598a90"),
    ("Linux", "aarch64"): ("linux-static-arm64", "b0c9367535d61ce8d5f021c6d434bef9c774692bbd751bd86d16602c57afbe9c"),
    ("Darwin", "x86_64"): ("macOS", "260ff58d63188e175d2f63be2823a65dcea21383ee4a83f370fd3e1b05f0e977"),
    ("Darwin", "arm64"): ("macOS", "260ff58d63188e175d2f63be2823a65dcea21383ee4a83f370fd3e1b05f0e977"),
}


def install_tools():
    if not FORMATTER.is_file():
        system = (platform.system(), platform.machine())
        if system not in ASSETS:
            raise RuntimeError(f"No Verible binary configured for {system}")
        asset, checksum = ASSETS[system]
        url = (f"https://github.com/chipsalliance/verible/releases/download/{VERIBLE_VERSION}/"
               f"verible-{VERIBLE_VERSION}-{asset}.tar.gz")
        print(f"Downloading Verible {VERIBLE_VERSION}", flush=True)
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != checksum:
            raise RuntimeError("Verible download checksum does not match the pinned release")
        # Extract only the formatter to a fixed path, ignoring archive paths.
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
            member = next(m for m in archive.getmembers()
                          if m.isfile() and Path(m.name).name == "verible-verilog-format")
            binary = archive.extractfile(member).read()
        FORMATTER.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=FORMATTER.parent, delete=False) as output:
            output.write(binary)
            downloaded = Path(output.name)
        downloaded.chmod(0o755)
        downloaded.replace(FORMATTER)
    if not PRE_COMMIT_PYTHON.is_file():
        venv.EnvBuilder(with_pip=True).create(PRE_COMMIT_PYTHON.parent.parent)
    subprocess.run([str(PRE_COMMIT_PYTHON), "-m", "pip", "install",
                    "--disable-pip-version-check", "--no-cache-dir",
                    f"pre-commit=={PRE_COMMIT_VERSION}"], check=True)


def main():
    arguments = sys.argv[1:]
    if arguments in (["--install"], ["--install-tools"]):
        install_tools()
        if arguments == ["--install"]:
            subprocess.run([str(PRE_COMMIT_PYTHON), "-m", "pre_commit", "install"],
                           cwd=ROOT, check=True)
        return 0
    formatter = os.environ.get("VERIBLE")
    if not formatter:
        formatter = str(FORMATTER) if FORMATTER.is_file() else shutil.which("verible-verilog-format")
    if not formatter:
        print("Verible is missing; run 'make install-hooks' once to install it.", file=sys.stderr)
        return 1
    return subprocess.call([formatter, *arguments])


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"Verible setup/formatting failed: {error}", file=sys.stderr)
        sys.exit(1)
