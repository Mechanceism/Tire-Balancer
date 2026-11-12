# Compile-check both sketches with arduino-cli (no upload).
#   powershell -ExecutionPolicy Bypass -File Tools\build.ps1
# Needs: arduino-cli, the "arduino:avr" and "esp32:esp32" cores, and the
# MPU6050 (Electronic Cats) + HX711 (Rob Tillaart) libraries for the Nano.
# The screen's libraries are bundled in Screen\libraries.

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$cli  = (Get-Command arduino-cli -ErrorAction SilentlyContinue).Source
if (-not $cli) { $cli = "C:\Program Files\Arduino CLI\arduino-cli.exe" }
if (-not (Test-Path $cli)) { throw "arduino-cli not found. Install it with: winget install ArduinoSA.CLI" }

Write-Host "== Nano ==" -ForegroundColor Cyan
& $cli compile --fqbn "arduino:avr:nano:cpu=atmega328old" "$repo\Nano\TireBalancer_Nano"
$nano = $LASTEXITCODE

Write-Host "== Screen ==" -ForegroundColor Cyan
& $cli compile --fqbn "esp32:esp32:esp32:FlashMode=dio" --libraries "$repo\Screen\libraries" "$repo\Screen\ui"
$screen = $LASTEXITCODE

if ($nano -or $screen) { throw "Build failed (Nano exit $nano, Screen exit $screen)" }
Write-Host "Both sketches compile." -ForegroundColor Green
