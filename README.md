# DBT-RISE
A versatile Dynamic Binary Translation (DBT) based environment to implement instruction set simulators (ISS)

This library contains the core elements of DBT-RISE and as such is intended to be part of a target project like [DBT-RISE-RISCV](https://github.com/Minres/DBT-RISE-RISCV) or [HIFIFE1-VP](https://github.com/Minres/HIFIVE1-VP).

It implements the basic structure to quickly develop ISS using dynamic binary translation using different options for a backend.

The different backends are:
- interp: An interpreter based backend without any dbt
- tcc: c compiler without any optimizations
- llvm: llvm IR with optimizations from llvm
- asmjit: direct host assembly generation