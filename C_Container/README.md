# Linear Algebra Abstraction Framework (Offline-Ready)

This repository demonstrates an extensible C++98 linear algebra abstraction framework that can switch seamlessly between **Eigen**, **Intel MKL**, and **pure standard C++98** (no dependencies).  
It also supports **offline builds** via prebuilt toolchain base images (`devbase` concept).

---

## 🧱 Repository Overview

| Backend | Library Used | Purpose | Build Option |
|----------|---------------|----------|---------------|
| **Eigen** | Eigen3 | Portable high-performance default | `-DUSE_MKL=OFF -DUSE_STD=OFF` |
| **MKL** | Intel MKL | Optimized for Intel CPUs | `-DUSE_MKL=ON` |
| **STD** | None (C++98 only) | Baseline reference; offline safe | `-DUSE_STD=ON` |

All backends share the same API, with batch processing support for updating multiple tracks (e.g., radar track covariances) simultaneously.

---

## 🧩 Directory Layout

```
include/
  la.h                # Common interface
src/
  la_eigen.cpp        # Eigen backend
  la_mkl.cpp          # Intel MKL backend
  la_std.cpp          # Pure C++98 backend
  main.cpp            # Demo driver + timing example
CMakeLists.txt        # Unified build configuration
Dockerfile variants   # Offline + online build targets
```

---

## 🚀 Build Options

### Online Builds

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

## 📴 Offline Workflow

### 1️⃣ Build Base Image Online

**devbase-std.Dockerfile**
```dockerfile
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build pkg-config ca-certificates && \
    rm -rf /var/lib/apt/lists/*
```

Build & export it once (online):
```bash
docker build -t devbase:std -f devbase-std.Dockerfile .
docker save -o devbase-std.tar devbase:std
# Optional: docker save -o ubuntu-22.04.tar ubuntu:22.04
```

Copy the following to your offline system:
- `devbase-std.tar`
- (optional) `ubuntu-22.04.tar`
- This project directory

---

### 2️⃣ Load Base Image Offline

```bash
docker load -i devbase-std.tar
# Optional: docker load -i ubuntu-22.04.tar
```

Verify:
```bash
docker images | grep devbase
```

---

### 3️⃣ Build App Offline

**app-std.Dockerfile**
```dockerfile
FROM devbase:std
WORKDIR /src
COPY . /src
RUN cmake -S . -B /build -G Ninja \
      -DUSE_STD=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=98 && \
    cmake --build /build -j
ENTRYPOINT ["/build/demo"]
```

Build it offline:
```bash
docker build -t la-demo:std -f app-std.Dockerfile .
```

Run:
```bash
docker run --rm la-demo:std
```

---

## 🧮 Example Output

```
Initializing 1000 tracks...
Batch update wall time = 0.008765 s
Checksum: 1.23456e+03
```

---

## ⚙️ Thread Control

| Backend | Variable | Example |
|----------|-----------|----------|
| **MKL** | `MKL_NUM_THREADS` | `docker run -e MKL_NUM_THREADS=4 la-demo:mkl` |
| **Eigen (OpenMP)** | `OMP_NUM_THREADS` | `docker run -e OMP_NUM_THREADS=4 la-demo:eigen` |
| **STD** | single-threaded | N/A |

---

## 🧠 Tips

- Always build with **`-DCMAKE_BUILD_TYPE=Release`** for accurate performance.
- Small matrices (e.g., 6×6) may perform faster in Eigen or STD than MKL (overhead).
- MKL shows its advantage for larger matrices and when multithreaded.
- Add OpenMP later to parallelize Eigen or STD outer loops.

---

## 🧩 Future Extensions

- Add OpenMP or TBB parallelization for the outer batch loop.
- Extend MKL path to handle variable-sized matrices per track.
- Add timing utilities and CSV output for performance sweeps.
- Integrate unit tests (GoogleTest or Catch2).

---

© 2025 Jeffrey Noe — MIT License
