#!/usr/bin/env python3
"""Sanity small_steps experiment; defaults to preview. See BENCH_EXPERIMENTS.txt."""
import sys
from bench_experiments import main

if __name__ == "__main__":
    sys.exit(main(preset="small_steps"))
