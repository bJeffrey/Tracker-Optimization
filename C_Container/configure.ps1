param([string]$BuildDir="build")

$ErrorActionPreference = "Stop"
$cpuVendor = "other"
try {
  $m = (Get-CimInstance Win32_Processor | Select-Object -First 1 -ExpandProperty Manufacturer)
  if ($m -match "Intel") { $cpuVendor = "intel" }
} catch {}

$use_mkl = "OFF"
if ($cpuVendor -eq "intel" -and $env:MKLROOT) { $use_mkl = "ON" }

Write-Host "[detect] cpu_vendor=$cpuVendor  MKLROOT=$($env:MKLROOT)  USE_MKL=$use_mkl"
if (-not (Test-Path $BuildDir)) { New-Item -ItemType Directory -Path $BuildDir | Out-Null }

cmake -S . -B $BuildDir -G "Ninja" -DUSE_MKL=$use_mkl
cmake --build $BuildDir -j
Write-Host "[done] run: $BuildDir\demo.exe"
