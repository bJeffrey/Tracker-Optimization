# Linear Algebra Abstraction Framework (CentOS 9 + C++17 Ready)

This repository provides a modular, offline-ready C++ linear algebra abstraction framework
with interchangeable backends (**Eigen**, **Intel MKL**, **Standard C++**) unified under a
single API. It now targets **CentOS Stream 9 (RHEL 9–compatible)** and **C++17** for modern
toolchains while retaining full offline portability.

---

## 🧱 Overview

| Backend | Library | Highlights | Build Flags |
|----------|----------|-------------|--------------|
| **Eigen** | Eigen3 | Portable, high‑performance default with OpenMP | `-DUSE_MKL=OFF -DUSE_STD=OFF` |
| **MKL** | Intel oneAPI MKL | Optimized for Intel CPUs, fastest for large matrices | `-DUSE_MKL=ON` |
| **STD** | None | Pure C++17 reference; minimal dependencies | `-DUSE_STD=ON` |

Each backend implements the same batch‑covariance API, enabling consistent benchmarking or
deployment on systems with or without external libraries.

---

## 🗂 Directory Layout

```
include/
  la.h                # Common API definitions
src/
  la_eigen.cpp        # Eigen backend (OpenMP supported)
  la_mkl.cpp          # Intel MKL backend (oneAPI EL9 RPMs)
  la_std.cpp          # Standard C++17 baseline backend
  main.cpp            # Demo driver
CMakeLists.txt        # Unified build configuration (C++17)
build.sh              # Smart build selector (Eigen/MKL/STD, auto CPU detect)
app-*.Dockerfile      # CentOS Stream 9 container builds
```

---

## 🚀 Build Options

### 🔧 Using Docker

#### Eigen (default)
```bash
docker build -t la-demo:eigen -f app-eigen.Dockerfile .
docker run --rm la-demo:eigen
```

#### MKL
```bash
docker build -t la-demo:mkl -f app-mkl.Dockerfile .
docker run --rm -e MKL_NUM_THREADS=1 -e OMP_NUM_THREADS=1 la-demo:mkl
```

#### STD (baseline)
```bash
docker build -t la-demo:std -f app-std.Dockerfile .
docker run --rm la-demo:std
```

---

### 🧰 Building Directly (RHEL 9 / CentOS 9)

The helper script automatically detects CPU vendor and selects a backend:

```bash
./build.sh --backend auto        # detect Intel → MKL, else Eigen
./build.sh --backend eigen       # force Eigen
./build.sh --backend mkl         # force MKL
./build.sh --backend std         # force STD backend
./build.sh --clean               # clean build directory
```

To run after build:

```bash
cd build
./demo
```

---

## 🧮 Example Output

```
Batch RW propagation time (dt=0.1): 0.0087 s, checksum 6026.27
```

---

## ⚙️ Thread Control

| Backend | Env Variable | Example |
|----------|--------------|----------|
| **MKL** | `MKL_NUM_THREADS` | `export MKL_NUM_THREADS=4` |
| **Eigen** | `OMP_NUM_THREADS` | `export OMP_NUM_THREADS=4` |
| **STD** | single‑threaded | N/A |

---

## 🧠 Notes

- Use **`-DCMAKE_BUILD_TYPE=Release`** for performance comparisons.
- Small matrices (6×6 typical for radar/ESM) may show similar timings across all backends.
- MKL excels for larger batch sizes or high‑thread counts.
- Eigen benefits from OpenMP and vectorization when enabled in CMake.

---

## 🧩 Planned Enhancements

- Extend Eigen/STD batch loops with OpenMP parallelization.
- Add ESM‑inspired covariance models (frequency, drift, bias).
- Generate performance metrics and plots automatically.
- Add unit testing and continuous integration for each backend.

---

© 2025 Jeffrey Noe — MIT License
