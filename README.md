# Tracker Optimization (C++17)

This repository provides a modular tracker demo with pluggable math backends (Eigen/MKL/STD),
config-driven scenario loading, and hot+warm track database experimentation.

## 🧱 Overview

| Backend | Library | Highlights | Docker Build Arg |
|--------|---------|------------|------------------|
| **Eigen** | Eigen3 | Portable, high‑performance default with OpenMP | `--build-arg BACKEND=eigen` |
| **MKL** | Intel oneAPI MKL | Optimized for Intel CPUs, strong for large/batched ops | `--build-arg BACKEND=mkl` |
| **STD** | None | Pure C++ baseline; minimal deps; predictable | `--build-arg BACKEND=std` |



## Docker Build and Run

Build images (choose backend):
```bash
docker build -t la-demo:eigen --build-arg BACKEND=eigen .
docker build -t la-demo:mkl   --build-arg BACKEND=mkl   .
docker build -t la-demo:std   --build-arg BACKEND=std   .
```

Run images:
```bash
docker run --rm la-demo:eigen
docker run --rm la-demo:mkl
docker run --rm la-demo:std
```

Dev build stage (interactive shell):
```bash
docker build --target build -t la-dev:eigen-build --build-arg BACKEND=eigen .
docker run --rm -it -v ${PWD}:/src -w /src la-dev:eigen-build /bin/bash
```

## Commands Inside the Dev Container

Build:
```bash
cmake -S . -B build -G Ninja
cmake --build build -j
```

Run:
```bash
./C_Container/run.sh
```

## Host Build (Non-Container)

Build (defaults to Debug for fast iteration):
```bash
./C_Container/build.sh
```

Backend selection:
```bash
./C_Container/build.sh --backend eigen
./C_Container/build.sh --backend mkl
./C_Container/build.sh --backend std
```

Build types:
```bash
./C_Container/build.sh --build-type Debug
./C_Container/build.sh --build-type Release
```

Notes:
- Default build type: Debug (fast compile, full symbols).
- LTO is OFF by default.
- --build-type Release enables LTO unless --disable-lto is passed.

## Run Script Notes

`./C_Container/run.sh`:
- Uses `TRACKER_LOG_LEVEL` only (no `--verbose` flag).
- Supports `RUN_S` env var (default 10.0).

Log levels (env var):
```
TRACE, DEBUG, INFO, WARN, ERROR
```

## Config Entry Points

Main configs live under `C_Container/config/` and are wired by:
- `C_Container/config/system.xml`
- `C_Container/config/scenarios/scenario_default.xml`
- `C_Container/config/store.xml`

## Performance Logging

Performance CSVs are written to:
- Rolling file: `C_Container/logs/performance.csv`
- Archive per run: `C_Container/logs/archive/YYYYMMDD/performance_YYYYMMDD_HHMMSS.csv`

If the CSV schema changes, the program writes a new header before appending.

## Notes

- Release builds are slower to compile but faster at runtime.
- Debug builds are best for iteration and profiling logic changes.
