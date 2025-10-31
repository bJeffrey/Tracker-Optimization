# simple-la — Portable C++98 Linear Algebra Starter (Eigen or MKL)

A tiny C++98 starter that multiplies two matrices through a single API, with **either Eigen or Intel MKL** behind the scenes.  
You choose **one backend per image** at build time.

This repo includes an **offline-friendly flow (Option B)**:
- Build two **dev base** images **online** (once), save them to `.tar`.
- On an **offline** machine, load those bases and rebuild the app images locally.

---

## Repo layout

```
.
├─ CMakeLists.txt
├─ include/
│  └─ la.h
├─ src/
│  ├─ la_eigen.cpp
│  ├─ la_mkl.cpp
│  └─ main.cpp
├─ devbase-eigen.Dockerfile   # build online once
├─ devbase-mkl.Dockerfile     # build online once
├─ app-eigen.Dockerfile       # rebuild offline using dev base
└─ app-mkl.Dockerfile         # rebuild offline using dev base
```

- **C++ standard:** C++98 (set in CMake or Docker build lines).
- **Backend selection:** `Eigen` or `MKL` (one per build).

---

## Prerequisites

- Docker Desktop (Windows/macOS) or Docker Engine (Linux)
- Internet access **only** on the machine used to build the **dev base** images (for MKL packages / apt repos)

> On Windows, run commands in **PowerShell**; on macOS/Linux, use a shell (bash/zsh). Replace `\` line continuations with backticks <code>`</code> in PowerShell if needed.

---

## ✅ Quick start (Online only)

If you don’t need offline support, you can build and run directly with the **online** multi-stage flow (Option 1).  
Use MKL variant only if you actually need MKL; otherwise, Eigen is simpler and smaller.

```bash
# Eigen-only image (no MKL)
docker build -t la-demo:eigen .

# With MKL (downloads oneAPI repo & packages)
docker build -t la-demo:mkl --build-arg USE_MKL=1 .

# Run
docker run --rm la-demo:eigen
docker run --rm la-demo:mkl
```

You should see output like:
```
sum(C)=<some number>
```

If you need **offline** usage, follow the steps below to pre-stage dev bases.

---

## Option B — Offline-friendly workflow

### 1) Build **dev base** images online (once)

These include compilers, CMake, Ninja, and either Eigen or MKL.  
Do this **on a machine with internet access**:

```bash
# From the repo root
docker build -t devbase:eigen -f devbase-eigen.Dockerfile .
docker build -t devbase:mkl   -f devbase-mkl.Dockerfile   .

# Save for transfer to offline machine
docker save -o devbase-eigen.tar devbase:eigen
docker save -o devbase-mkl.tar   devbase:mkl
```

> Tip: Also save the **project source** (this repo) and carry it along with the `.tar` files.

---

### 2) Move to the **offline** machine

Copy these files to the offline host (USB, etc.):  
- `devbase-eigen.tar`  
- `devbase-mkl.tar`  
- The project source directory (this repo)

Load the dev bases:

```bash
docker load -i devbase-eigen.tar
docker load -i devbase-mkl.tar
```

---

### 3) Build the **app images** offline

Now, with the dev bases available locally, rebuild the app layers completely **offline**:

```bash
# Eigen app (offline)
docker build -t la-demo:eigen -f app-eigen.Dockerfile .

# MKL app (offline)
docker build -t la-demo:mkl   -f app-mkl.Dockerfile   .
```

> These app Dockerfiles do **not** reach the network. They use the preloaded dev bases and your local source only.

---

### 4) Run (offline)

```bash
docker run --rm la-demo:eigen
docker run --rm la-demo:mkl
```

Expected output:
```
sum(C)=<some number>
```

---

## Notes & tips

- **When to use MKL:** Only if you’re running on x86 and actually want MKL’s tuned BLAS/LAPACK. The MKL dev base is larger and requires the oneAPI apt repo during its **online** build.
- **CMake options:**
  - `-DUSE_MKL=OFF` compiles the Eigen backend (`app-eigen.Dockerfile` already does this).
  - `-DUSE_MKL=ON` compiles the MKL backend (`app-mkl.Dockerfile` already does this).
- **C++98:** Both Docker paths set/expect C++98; the code and CMake are compatible.
- **Windows/macOS:** All Linux commands run inside the Linux containers; you don’t need host toolchains.
- **Rebuilding offline:** Just re-run the `docker build` commands for the app images. No internet needed as long as the dev bases are loaded.

---

## Troubleshooting

- **MKL repo/key errors (online dev base build):**
  - Ensure the key URL in `devbase-mkl.Dockerfile` is reachable:
    `https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB`
  - Corporate proxies may require adding a CA certificate or proxy env vars (`http_proxy`, `https_proxy`) **just for the dev base build**.

- **“file not found” / missing Dockerfiles:**
  - Run builds from the repo root where the Dockerfiles and `CMakeLists.txt` live.

- **No output or unexpected sum:**
  - Rebuild images to ensure the latest source is inside:
    ```bash
    docker build --no-cache -t la-demo:eigen -f app-eigen.Dockerfile .
    ```

---

## What’s next?

- Add more ops to `include/la.h` (e.g., `gemv`, `potrf`, `trsm`) and implement them in both `la_eigen.cpp` and `la_mkl.cpp`.
- Create simple unit tests and a tiny benchmark target to compare Eigen vs MKL.
- If you later want a **host-native** build (no Docker), use the provided `configure.sh` / `configure.ps1` pattern to select MKL or Eigen at build time.

---

Happy building! If you want this README customized for your internal registry (saving/loading to a private Harbor/ECR/GCR, etc.), I can add those commands too.
