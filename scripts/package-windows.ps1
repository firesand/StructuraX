# PowerShell Build & Packaging Script for StructuraX on Windows (x64)
# Usage: pwsh -File scripts/package-windows.ps1

$ErrorActionPreference = "Stop"
$Version = "1.0.0"
$Target = "x86_64-pc-windows-msvc"

Write-Host "==> Building StructuraX v$Version for Windows ($Target)..."
Set-Location (Split-Path $PSScriptRoot -Parent)
cargo build --locked --release --target $Target --bin structurax

if ($LASTEXITCODE -ne 0) { throw "Cargo build failed" }

$StageDir = "dist\windows\stage"
if (Test-Path $StageDir) { Remove-Item $StageDir -Recurse -Force }
New-Item -ItemType Directory -Force -Path "$StageDir\sections" | Out-Null

Copy-Item "target\$Target\release\structurax.exe" "$StageDir\structurax.exe" -Force
# Vendor section libraries are not distributed.
Copy-Item "LICENSE" "$StageDir\LICENSE" -Force

@"
StructuraX v$Version — 3D Finite Element Structural Analysis & Code-Based Member Design Suite
=============================================================================================

Run Desktop GUI:
  .\structurax.exe

Run CLI Verification Suite (60 Benchmarks):
  .\structurax.exe verify

Analyze a Structural Model (.s2k or .sdb):
  .\structurax.exe analyze model.s2k --design --output results.json
"@ | Set-Content -Path "$StageDir\README.txt" -Encoding UTF8

$ZipPath = "dist\windows\StructuraX-$Version-windows-x86_64.zip"
if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force }
Compress-Archive -Path "$StageDir\*" -DestinationPath $ZipPath
Write-Host "==> Created portable Windows archive: $ZipPath"

if (Get-Command makensis -ErrorAction SilentlyContinue) {
    makensis packaging\windows\installer.nsi
    if ($LASTEXITCODE -ne 0) { throw "NSIS packaging failed" }
    Write-Host "==> Created Windows Setup Installer: dist\windows\StructuraX-$Version-Windows-x64-Setup.exe"
} else {
    Write-Host "==> Note: Install NSIS (makensis) to also build the Windows Setup .exe wizard."
}

