#!/usr/bin/env bash
# build.sh — portable builder for RHEL 9 / CentOS Stream 9
# Chooses MKL on Intel if available, else Eigen. Override with --backend.
# Backends: mkl | eigen | std

set -euo pipefail

# ---------------------------
# Defaults & usage
# ---------------------------
BUILD_DIR="build"
GENERATOR="Ninja"
CXX_STANDARD="17"
BUILD_TYPE="Release"
ENABLE_OPENMP="ON"
BACKEND=""     # auto
JOBS="$(nproc || echo 4)"

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --backend {mkl|eigen|std}   Force backend (default: auto-detect)
  --build-dir DIR             Build directory (default: ${BUILD_DIR})
  --no-openmp                 Disable OpenMP (default: enabled)
  --build-type {Release|Debug|RelWithDebInfo|MinSizeRel} (default: ${BUILD_TYPE})
  --jobs N                    Parallel build jobs (default: ${JOBS})
  --clean                     Remove build dir before configuring
  --help                      Show this help

Auto-detect rules:
  1) If --backend given, use it.
  2) Else if CPU vendor is Intel AND MKL is installed (MKLROOT or cmake package found), use 'mkl'.
  3) Else use 'eigen'.

Dependency hints (RHEL9 / CentOS Stream 9):
  sudo dnf -y install gcc gcc-c++ make cmake ninja-build pkgconf-pkg-config
  # For Eigen backend:
  sudo dnf -y install eigen3-devel
  # For MKL backend (after enabling Intel repo):
  sudo dnf -y install intel-oneapi-mkl-devel
EOF
}

# ---------------------------
# Parse args
# ---------------------------
CLEAN=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend) BACKEND="${2:-}"; shift 2 ;;
    --build-dir) BUILD_DIR="${2:-}"; shift 2 ;;
    --no-openmp) ENABLE_OPENMP="OFF"; shift ;;
    --build-type) BUILD_TYPE="${2:-}"; shift 2 ;;
    --jobs) JOBS="${2:-}"; shift 2 ;;
    --clean) CLEAN=1; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

# ---------------------------
# Dependency checks (soft)
# ---------------------------
need_bin() {
  command -v "$1" >/dev/null 2>&1 || { echo "Missing tool: $1"; return 1; }
  return 0
}

MISSING=0
for t in cmake ${GENERATOR,,}; do
  if [[ "$t" == "ninja" ]]; then t="ninja"; fi
  need_bin "$t" || MISSING=1
done

# GCC and G++ (OpenMP uses libgomp)
for t in gcc g++; do
  need_bin "$t" || MISSING=1
done

if [[ $MISSING -eq 1 ]]; then
  echo
  echo "Some required tools are missing. On RHEL9/CentOS9:"
  echo "  sudo dnf -y install gcc gcc-c++ make cmake ninja-build pkgconf-pkg-config"
  echo
fi

# ---------------------------
# Detect Intel CPU & MKL presence
# ---------------------------
is_intel_cpu=0
if [[ -r /proc/cpuinfo ]] && grep -qi "GenuineIntel" /proc/cpuinfo; then
  is_intel_cpu=1
fi

# Try to detect MKL
mkl_available=0
# Heuristic 1: MKLROOT environment
if [[ -n "${MKLROOT:-}" && -d "$MKLROOT" ]]; then
  mkl_available=1
fi
# Heuristic 2: Well-known CMake package path
if [[ $mkl_available -eq 0 ]]; then
  if [[ -d /opt/intel/oneapi/mkl/latest/lib/cmake/mkl ]]; then
    export MKLROOT=/opt/intel/oneapi/mkl/latest
    mkl_available=1
  fi
fi

# ---------------------------
# Decide backend
# ---------------------------
if [[ -n "$BACKEND" ]]; then
  case "$BACKEND" in
    mkl|eigen|std) ;;
    *) echo "Invalid --backend '$BACKEND'"; exit 1 ;;
  esac
else
  if [[ $is_intel_cpu -eq 1 && $mkl_available -eq 1 ]]; then
    BACKEND="mkl"
  else
    BACKEND="eigen"
  fi
fi

echo "[INFO] Backend: $BACKEND"
echo "[INFO] OpenMP:  $ENABLE_OPENMP"
echo "[INFO] Build:    $BUILD_TYPE, C++${CXX_STANDARD}, generator=${GENERATOR}, dir=${BUILD_DIR}"

# Helpful hints if user forced MKL but we don't see it
if [[ "$BACKEND" == "mkl" && $mkl_available -eq 0 ]]; then
  cat <<EOF
[WARN] MKL requested, but MKL not detected.
       Install MKL (RHEL9/CentOS9):
         sudo rpm --import https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB
         sudo tee /etc/yum.repos.d/intel-oneapi.repo >/dev/null <<'REPO'
[intel-oneapi]
name=Intel oneAPI Repository
baseurl=https://yum.repos.intel.com/oneapi
enabled=1
gpgcheck=1
repo_gpgcheck=1
gpgkey=https://yum.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2023.PUB
REPO
         sudo dnf -y makecache
         sudo dnf -y install intel-oneapi-mkl-devel
EOF
  exit 2
fi

# Eigen headers present?
if [[ "$BACKEND" == "eigen" ]]; then
  if ! rpm -q eigen3-devel >/dev/null 2>&1; then
    echo "[WARN] eigen3-devel not detected. Install with:"
    echo "       sudo dnf -y install eigen3-devel"
    # still attempt build if Eigen headers are vendored somewhere else
  fi
fi

# ---------------------------
# Configure environment for MKL threading (avoid oversubscription)
# ---------------------------
CMAKE_PREFIX_PATH_ENV="${CMAKE_PREFIX_PATH:-}"
if [[ "$BACKEND" == "mkl" ]]; then
  # Prefer GCC OpenMP layer with MKL when building with GCC toolchain
  export MKL_THREADING_LAYER=GNU
  : "${MKL_NUM_THREADS:=1}"   # single-thread MKL; let outer OpenMP split batches
  export MKL_NUM_THREADS

  # Help CMake find MKLConfig.cmake
  if [[ -n "${MKLROOT:-}" && -d "$MKLROOT/lib/cmake/mkl" ]]; then
    if [[ -n "$CMAKE_PREFIX_PATH_ENV" ]]; then
      export CMAKE_PREFIX_PATH="$MKLROOT/lib/cmake/mkl:$CMAKE_PREFIX_PATH_ENV"
    else
      export CMAKE_PREFIX_PATH="$MKLROOT/lib/cmake/mkl"
    fi
  fi
fi

# ---------------------------
# Clean build dir (optional)
# ---------------------------
if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then
  echo "[INFO] Removing ${BUILD_DIR}"
  rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

# ---------------------------
# Compose CMake flags
# ---------------------------
USE_MKL_FLAG="OFF"
USE_STD_FLAG="OFF"

case "$BACKEND" in
  mkl)   USE_MKL_FLAG="ON"  ;;
  std)   USE_STD_FLAG="ON"  ;;
  eigen) ;; # default
esac

CMAKE_FLAGS=(
  -G "${GENERATOR}"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DCMAKE_CXX_STANDARD="${CXX_STANDARD}"
  -DENABLE_OPENMP="${ENABLE_OPENMP}"
  -DUSE_MKL="${USE_MKL_FLAG}"
  -DUSE_STD="${USE_STD_FLAG}"
)

echo "[INFO] CMake flags: ${CMAKE_FLAGS[*]}"

# ---------------------------
# Configure & build
# ---------------------------
cmake -S . -B "${BUILD_DIR}" "${CMAKE_FLAGS[@]}"
cmake --build "${BUILD_DIR}" -j "${JOBS}"

echo
echo "[OK] Build complete. Run:"
echo "     ${BUILD_DIR}/demo"
echo
echo "Tips:"
echo "  # Control outer OpenMP threads"
echo "  OMP_NUM_THREADS=8 ${BUILD_DIR}/demo"
if [[ "$BACKEND" == "mkl" ]]; then
  echo "  # MKL is single-threaded by default via MKL_NUM_THREADS=1."
  echo "  # If you change it, consider reducing OMP_NUM_THREADS to avoid oversubscription."
fi
