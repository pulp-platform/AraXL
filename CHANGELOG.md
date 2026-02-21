# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## 0.2 - 21-02-2026

- Added support for mask loads and stores `vlm`, `vsm`
- Added support for indexed load operations `vluxei`, `vloxei`

## 0.1.1 - 30-01-2026

- Removed the dispatcher from Global VLSU and pass metadata from VLSU of Cluster 0
- Fixed vsetvl instructions for vector lengths not divisible by number of clusters
- Fixed synchronization between clusters for load-store instructions
- Fixed support for misaligned AXI reads for unit-stride vector load instructions
- Fixed vmv instructions
- Added benchmarks and instruction-level tests to CI

## 0.1.0 - 01-09-2025

- Hardware support currently for performance critical applications
  - Unit-stride load-stores
  - Slide-by-1 operations
  - Reduction operations
  - Basic masking operations
  - All integer and floating-point arithmetic

- Newly designed interconnects
  - Slide Unit (for sliding, reduction operations)
  - Global Load-store unit (Global controller for vector load-store operations)
  - Request Interface (Request broadcasting from CVA6 core)

- Configurations upto 64 lanes nr_clusters=16 nr_lanes=4 (maximum supported by RISC-V V 1.0 extension)
