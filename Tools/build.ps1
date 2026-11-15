# Compile-check both sketches with arduino-cli (no upload).
#   powershell -ExecutionPolicy Bypass -File Tools\build.ps1
# Needs: arduino-cli and the "arduino:avr" and "esp32:esp32" cores.
# Libraries come only from the repo (Nano\libraries, Screen\libraries): each
# build points arduino-cli's sketchbook at that board's folder, the same as
# setting the Arduino IDE's Sketchbook location. Libraries installed on this PC
# are not used, so a passing build means the repo has everything it needs.

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$cli  = (Get-Command arduino-cli -ErrorAction SilentlyContinue).Source
if (-not $cli) { $cli = "C:\Program Files\Arduino CLI\arduino-cli.exe" }
if (-not (Test-Path $cli)) { throw "arduino-cli not found. Install it with: winget install ArduinoSA.CLI" }

$oldUser = $env:ARDUINO_DIRECTORIES_USER
try {
  Write-Host "== Nano ==" -ForegroundColor Cyan
  $env:ARDUINO_DIRECTORIES_USER = "$repo\Nano"
  & $cli compile --fqbn "arduino:avr:nano:cpu=atmega328old" "$repo\Nano\TireBalancer_Nano"
  $nano = $LASTEXITCODE

  Write-Host "== Screen ==" -ForegroundColor Cyan
  $env:ARDUINO_DIRECTORIES_USER = "$repo\Screen"
  & $cli compile --fqbn "esp32:esp32:esp32:FlashMode=dio,PartitionScheme=huge_app" "$repo\Screen\ui"
  $screen = $LASTEXITCODE
} finally {
  $env:ARDUINO_DIRECTORIES_USER = $oldUser
}

if ($nano -or $screen) { throw "Build failed (Nano exit $nano, Screen exit $screen)" }
Write-Host "Both sketches compile." -ForegroundColor Green
