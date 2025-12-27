# Linear Algebra Abstraction Framework (C++17, Offline-Ready)

This repository provides a modular, offline-ready C++ linear algebra abstraction framework
with interchangeable backends (**Eigen**, **Intel MKL**, **Standard C++**) unified under a
single API. The primary container workflow is now based on a **single unified `Dockerfile`**
(ubuntu-based) while keeping the project itself **portable** across Linux environments.

> **Legacy note:** Older *CentOS Stream 9* `base-*` / `app-*` Dockerfiles may still exist in the repo
> for reference/rollback, but the recommended path is the unified root `Dockerfile`.

---

## 🧱 Overview

| Backend | Library | Highlights | Docker Build Arg |
|--------|---------|------------|------------------|
| **Eigen** | Eigen3 | Portable, high‑performance default with OpenMP | `--build-arg BACKEND=eigen` |
| **MKL** | Intel oneAPI MKL | Optimized for Intel CPUs, strong for large/batched ops | `--build-arg BACKEND=mkl` |
| **STD** | None | Pure C++ baseline; minimal deps; predictable | `--build-arg BACKEND=std` |

All backends implement the same batch‑covariance API (see `include/la.h`) and the demo in
`src/main.cpp` runs a Δt‑scaled random‑walk covariance update across many tracks.

---

## 🗂 Directory Layout

```
include/
  la.h                # Common API definitions
  config/             # Config system public headers (loader + parsed types)
src/
  la_eigen.cpp        # Eigen backend (optional OpenMP)
  la_mkl.cpp          # Intel MKL backend (oneAPI)
  la_std.cpp          # Standard C++ backend
  la_batch_rw_*.cpp   # Backend-selected fast RW batch update (full 9x9 for CA9)
  targets/            # Target generation utilities (config-driven)
  main.cpp            # Demo driver (config-driven + timed batch propagation)
  config/             # Config loader implementation (internal helpers + loader)
  config_check.cpp    # Config loader smoke-test executable (load + validate + print summary)
config/
  system.xml          # Root config entry point (refs + active profile selection)
  *.xml               # Other config modules (sensors, store, etc.)
schemas/
  *.xsd               # XSD pack used for validation
CMakeLists.txt        # Unified CMake (C++17, OpenMP hints for Eigen)
build.sh              # Smart build selector (auto/eigen/mkl/std) for direct-on-host builds
Dockerfile            # Unified multi-stage docker build (eigen/std/mkl via BACKEND arg)

# Legacy (optional / may still exist):
# app-eigen.Dockerfile, app-mkl.Dockerfile, app-std.Dockerfile
# base-eigen.Dockerfile, base-mkl.Dockerfile, base-std.Dockerfile
```

---

## 🚀 Online Build/Run (Docker) — Recommended

### Eigen (default)
```bash
docker build -t la-demo:eigen --build-arg BACKEND=eigen .
docker run --rm la-demo:eigen
```

### MKL
```bash
docker build -t la-demo:mkl --build-arg BACKEND=mkl .
docker run --rm -e MKL_NUM_THREADS=1 -e OMP_NUM_THREADS=1 la-demo:mkl
```

### STD (baseline)
```bash
docker build -t la-demo:std --build-arg BACKEND=std .
docker run --rm la-demo:std
```

> Open an interactive shell inside an image (useful for debugging):
>
> ```bash
> docker run --rm -it la-demo:eigen /bin/bash
> ```
>
> Notes:
> - The unified Dockerfile uses `CMD ["demo"]` (not ENTRYPOINT), so passing `/bin/bash`
>   works as expected without needing `--entrypoint`.

---

## 🧪 Run Command Options (Demo + Config Validation)

These are the commands you’ll use most often while iterating on config + performance.

### 1) Demo (config-driven propagation timing)

From repo root (host or inside a dev container), after building:

```bash
./build/demo --config ./config/system.xml --xsd-dir ./schemas
```

Typical output includes:
- resolved config bundle summary
- `dim`, `tracks`, `dt_s`
- generation timing (if generation is enabled in the scenario)
- RW batch propagation timing + checksum

### 2) Config-only smoke test (load + validate + print summary)

```bash
./build/config_check --config ./config/system.xml --xsd-dir ./schemas
```

This is the fastest way to verify:
- XML href resolution
- XSD validation
- which “Active” IDs are being selected
- resolved paths (scenario, ownship, targets, etc.)

> Tip: if you’re unsure what flags exist for a binary, run it with `--help` (if implemented).

---

## 🧰 Build Directly on a Linux Host (no Docker)

The helper script can auto‑detect CPU vendor (Intel → MKL, else Eigen). You can override it.

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
cd build && ./demo --config ../config/system.xml --xsd-dir ../schemas
```

> If you’re not on RHEL/CentOS, you may need to install equivalent packages
> (compiler toolchain, CMake/Ninja, Eigen, OpenMP runtime, etc.).

---

## 📦 Offline Workflow (Containers)

**Goal:** produce images that can be built/used on a fully offline host.

### Option A — *Simplest*: Export Fully‑Built App Images (Recommended)

1) **Online machine — build the images**
```bash
docker build -t la-demo:eigen --build-arg BACKEND=eigen .
docker build -t la-demo:mkl   --build-arg BACKEND=mkl   .
docker build -t la-demo:std   --build-arg BACKEND=std   .
```

2) **Export images to tarballs**
```bash
docker save -o la-demo-eigen.tar la-demo:eigen
docker save -o la-demo-mkl.tar   la-demo:mkl
docker save -o la-demo-std.tar   la-demo:std
```

3) **Copy to offline machine**
- `la-demo-eigen.tar`, `la-demo-mkl.tar`, `la-demo-std.tar`

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
> **Cons:** images are larger; rebuilding offline requires repeating export.

---

### Option B — *Flexible (Legacy)*: Export Small “Base” Images, Build App Offline

This was the older workflow using `base-*` and `app-*` Dockerfiles. It can still be used if those
files remain in the repo, but it is **not the recommended path** now that the unified `Dockerfile`
exists.

If you keep using the legacy workflow, the high-level steps remain:
- build `la-base:*` images on an online machine
- `docker save` them
- `docker load` them on the offline host
- build `la-demo:*` app images offline on top of those bases

> **Tip (Windows):** If you edit shell scripts on Windows, ensure LF line endings:
> `git config core.autocrlf input` or run `dos2unix` before executing in containers.

---

### Option C — Build a dev shell for rapid iteration, then run builds inside the container

This is the workflow you want when you’re making **small code changes** frequently and want to avoid
rebuilding full docker layers every time.

1) **Build a dev image (build stage)**
```bash
docker build --target build -t la-dev:eigen-build --build-arg BACKEND=eigen .
```

2) **Run container and open a bash shell**

Windows PowerShell:
```powershell
docker run --rm -it -v ${PWD}:/src -w /src la-dev:eigen-build /bin/bash
```

Windows CMD:
```bat
docker run --rm -it -v %cd%:/src -w /src la-dev:eigen-build /bin/bash
```

Linux/macOS:
```bash
docker run --rm -it -v "$PWD":/src -w /src la-dev:eigen-build /bin/bash
```

3) **Compile inside the container**
```bash
cmake -S . -B build -G Ninja -DENABLE_CONFIG=ON
cmake --build build -j
```

4) **Run binaries inside the container**
```bash
./build/demo --config ./config/system.xml --xsd-dir ./schemas --verbose
./build/config_check --config ./config/system.xml --xsd-dir ./schemas
```

Notes:
- The **final runtime image** intentionally does **not** include `cmake`.
- If you see `bash: cmake: command not found`, you are in the runtime image (or your dev image tag is wrong).
  Use the `--target build` image/tag for compilation.

---

## 📚 Doxygen Docs (Optional)

Docs are generated **only if enabled** at build time:

```bash
docker build -t la-demo:eigen --build-arg BACKEND=eigen --build-arg GENERATE_DOCS=1 .
```

In the unified Dockerfile, docs are copied into the runtime image at:
- `/usr/local/share/la-demo/docs`

To copy them to your host **after a build**:

```bash
CID=$(docker create la-demo:eigen) && docker cp "$CID":/usr/local/share/la-demo/docs ./docs-eigen && docker rm "$CID"
```

On Windows PowerShell:
```powershell
$cid = docker create la-demo:eigen
docker cp "$cid`:/usr/local/share/la-demo/docs" ".\docs-eigen"
docker rm $cid
```

The HTML entry point (when generated) is typically:
- `docs-eigen/html/index.html`

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

- Always use **`-DCMAKE_BUILD_TYPE=Release`** for meaningful timing (the CMake config defaults to Release if unspecified).  
- Small matrices (e.g., 6×6 typical of many ESM states) can favor Eigen or even STD due to MKL dispatch overhead; MKL shines with larger `n` and large batch counts.  
- The demo avoids `F P Fᵀ` when `F = I` for a realistic stationary ESM update path.  
- If you need a shell in a built image: `docker run --rm -it la-demo:eigen /bin/bash`.

---

## 📜 License

© 2025 Jeffrey Noe — MIT License
