# Linear Algebra Abstraction Framework (CentOS Stream 9 + C++17, Offline-Ready)

This repository provides a modular, offline-ready C++ linear algebra abstraction framework
with interchangeable backends (**Eigen**, **Intel MKL**, **Standard C++**) unified under a
single API. It targets **CentOS Stream 9 (RHEL 9–compatible)** and **C++17** while retaining
full offline portability.

---

## 🧱 Overview

| Backend | Library | Highlights | Build Flags |
|--------|---------|------------|-------------|
| **Eigen** | Eigen3 | Portable, high‑performance default with OpenMP | `-DUSE_MKL=OFF -DUSE_STD=OFF` |
| **MKL** | Intel oneAPI MKL | Optimized for Intel CPUs, strong for large/batched ops | `-DUSE_MKL=ON` |
| **STD** | None | Pure C++ baseline; minimal deps; predictable | `-DUSE_STD=ON` |

All backends implement the same batch‑covariance API (see `include/la.h`) and the demo in
`src/main.cpp` times a Δt‑scaled random‑walk covariance update across many tracks.

---

## 🗂 Directory Layout

```
include/
  la.h                # Common API definitions
src/
  la_eigen.cpp        # Eigen backend (optional OpenMP)
  la_mkl.cpp          # Intel MKL backend (oneAPI EL9 RPMs)
  la_std.cpp          # Standard C++ backend
  main.cpp            # Demo driver (timed batch propagation)
CMakeLists.txt        # Unified CMake (C++17, OpenMP hints for Eigen)
build.sh              # Smart build selector (auto/eigen/mkl/std)
app-eigen.Dockerfile  # CentOS Stream 9 + Eigen
app-mkl.Dockerfile    # CentOS Stream 9 + MKL
app-std.Dockerfile    # CentOS Stream 9 + pure-STD
```

---

## 🚀 Online Build/Run (Docker)

### Eigen (default)
```bash
docker build -t la-demo:eigen -f app-eigen.Dockerfile .
docker run --rm la-demo:eigen
```

### MKL
```bash
docker build -t la-demo:mkl -f app-mkl.Dockerfile .
docker run --rm -e MKL_NUM_THREADS=1 -e OMP_NUM_THREADS=1 la-demo:mkl
```

### STD (baseline)
```bash
docker build -t la-demo:std -f app-std.Dockerfile .
docker run --rm la-demo:std
```

> Open an interactive shell inside an image (useful for debugging):
>
> ```bash
> docker run --rm -it --entrypoint /bin/bash la-demo:eigen
> ```

---

## 🧰 Build Directly on RHEL 9 / CentOS 9 (no Docker)

The helper script auto‑detects CPU vendor (Intel → MKL, else Eigen). You can override it.

```bash
# Make sure the script is LF (not CRLF) on Windows:
#   git config core.autocrlf input   # or convert with: dos2unix build.sh
chmod +x build.sh

./build.sh --backend auto        # detect Intel → MKL, else Eigen
./build.sh --backend eigen       # force Eigen
./build.sh --backend mkl         # force MKL
./build.sh --backend std         # force STD backend
./build.sh --clean               # clean build dir
```

Run the demo:
```bash
cd build && ./demo
```

---

## 📦 Offline Workflow (Containers)

**Goal:** produce images that can be built/used on a fully offline host.
You have two options; choose one based on your constraints.

### Option A — *Simplest*: Export Fully‑Built App Images

1) **Online machine — build the images**
```bash
docker build -t la-demo:eigen -f app-eigen.Dockerfile .
docker build -t la-demo:mkl   -f app-mkl.Dockerfile   .
docker build -t la-demo:std   -f app-std.Dockerfile   .
```

2) **Export images to tarballs**
```bash
docker save -o la-demo-eigen.tar la-demo:eigen
docker save -o la-demo-mkl.tar   la-demo:mkl
docker save -o la-demo-std.tar   la-demo:std
```

3) **Copy to offline machine**
- `la-demo-eigen.tar`, `la-demo-mkl.tar`, `la-demo-std.tar`
- Your project source tree (this repo), if you also want to build there later.

4) **Offline machine — load and run**
```bash
docker load -i la-demo-eigen.tar
docker load -i la-demo-mkl.tar
docker load -i la-demo-std.tar

docker run --rm la-demo:eigen
docker run --rm la-demo:mkl
docker run --rm la-demo:std
```

> **Pros:** fastest, least moving parts.  
> **Cons:** images are larger; re‑build offline requires repeating export.

---

### Option B — *Flexible*: Export Small “Base” Images, Build App Offline

This yields **smaller transfers** and lets you **rebuild app layers offline**.

#### 1) Online — build minimal **base images**

Create two base Dockerfiles (you may already have these names):

**`base-eigen.Dockerfile`** (CentOS 9 + toolchain + Eigen + Doxygen)
```dockerfile
# syntax=docker/dockerfile:1.6
FROM quay.io/centos/centos:stream9

ENV LANG=C.UTF-8
RUN dnf -y install dnf-plugins-core && dnf -y config-manager --set-enabled crb && \
    dnf -y install epel-release epel-next-release && \
    dnf -y install gcc gcc-c++ make cmake ninja-build eigen3-devel doxygen graphviz pkgconf-pkg-config && \
    dnf clean all
```

**`base-mkl.Dockerfile`** (CentOS 9 + toolchain + MKL repo + Doxygen)
```dockerfile
# syntax=docker/dockerfile:1.6
FROM quay.io/centos/centos:stream9

ENV LANG=C.UTF-8
RUN dnf -y install dnf-plugins-core && dnf -y config-manager --set-enabled crb && \
    dnf -y install gcc gcc-c++ make cmake ninja-build doxygen graphviz pkgconf-pkg-config curl-minimal ca-certificates && \
    dnf clean all

# Intel oneAPI MKL repo + key
RUN printf '%s\n' \
      '[intel-oneapi]' \
      'name=Intel oneAPI Repository' \
      'baseurl=https://yum.repos.intel.com/oneapi' \
      'enabled=1' \
      'gpgcheck=1' \
      'repo_gpgcheck=1' \
      'gpgkey=https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB' \
    > /etc/yum.repos.d/intel-oneapi.repo \
 && rpm --import https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB \
 && dnf -y makecache \
 && dnf -y install intel-oneapi-mkl-devel \
 && dnf clean all

# Helpful envs (define first, then reference; avoids UndefinedVar warnings)
ENV MKLROOT=/opt/intel/oneapi/mkl/latest
ENV LD_LIBRARY_PATH=/opt/intel/oneapi/mkl/latest/lib/intel64:/opt/intel/oneapi/compiler/latest/linux/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}
```

Build and save them:
```bash
docker build -t la-base:eigen -f base-eigen.Dockerfile .
docker build -t la-base:mkl   -f base-mkl.Dockerfile   .

docker save -o la-base-eigen.tar la-base:eigen
docker save -o la-base-mkl.tar   la-base:mkl
```

#### 2) Offline — load bases and build app locally

Copy to the offline host:
- `la-base-eigen.tar`, `la-base-mkl.tar`
- The project directory (this repo)

Load the bases:
```bash
docker load -i la-base-eigen.tar
docker load -i la-base-mkl.tar
```

Build app images **reusing the base layers**:
```bash
# Eigen app
docker build -t la-demo:eigen -f app-eigen.Dockerfile .

# MKL app
docker build -t la-demo:mkl   -f app-mkl.Dockerfile   .

# STD app (uses only CentOS toolchain; no external math lib)
docker build -t la-demo:std   -f app-std.Dockerfile   .
```

Run:
```bash
docker run --rm la-demo:eigen
docker run --rm la-demo:mkl
docker run --rm la-demo:std
```

> **Tip (Windows):** If you edit shell scripts on Windows, ensure LF line endings:
> `git config core.autocrlf input` or run `dos2unix` before executing in containers.

---

## 📚 Doxygen Docs (Auto‑Generated in Build)

Each Dockerfile generates Doxygen pages into the image at `/src/docs/html`.  
To copy them to your host **after a build**:

```bash
# Eigen
CID=$(docker create la-demo:eigen) && \
docker cp "$CID":/src/docs ./docs-eigen && \
docker rm "$CID"

# MKL
CID=$(docker create la-demo:mkl) && \
docker cp "$CID":/src/docs ./docs-mkl && \
docker rm "$CID"

# STD
CID=$(docker create la-demo:std) && \
docker cp "$CID":/src/docs ./docs-std && \
docker rm "$CID"
```

On Windows PowerShell:
```powershell
$cid = docker create la-demo:eigen
docker cp "$cid`:/src/docs" ".\docs-eigen"
docker rm $cid
```

The HTML entry point is `docs-*/html/index.html`.

---

## ⚙️ Thread Control

| Backend | Env Var | Example |
|--------|---------|---------|
| MKL    | `MKL_NUM_THREADS` | `docker run -e MKL_NUM_THREADS=4 la-demo:mkl` |
| Eigen  | `OMP_NUM_THREADS` | `docker run -e OMP_NUM_THREADS=4 la-demo:eigen` |
| STD    | single‑threaded   | N/A |

> For reproducible perf comparisons, also consider setting `KMP_AFFINITY` / `OMP_PROC_BIND`.

---

## 🧠 Notes & Tips

- Always use **`-DCMAKE_BUILD_TYPE=Release`** for meaningful timing.  
- Small matrices (e.g., 6×6 typical of many ESM states) can favor Eigen or even STD due to MKL dispatch overhead; MKL shines with larger `n` and large batch counts.  
- The demo avoids `F P Fᵀ` when `F = I` for a realistic stationary ESM update path.  
- If you need a shell in a built image: `docker run --rm -it --entrypoint /bin/bash la-demo:eigen`.

---

## 📜 License

© 2025 Jeffrey Noe — MIT License
