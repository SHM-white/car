$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$stub = Join-Path $PSScriptRoot 'arduino_stubs'
$protocolInclude = Join-Path $root 'embedded\shared_protocol\src'
$protocolSource = Join-Path $protocolInclude 'DTaskProtocol.cpp'
$build = Join-Path $PSScriptRoot 'build\host-sketches'
New-Item -ItemType Directory -Force -Path $build | Out-Null

function Compile-Sketch([string]$name, [string]$folder, [string[]]$extraFlags = @()) {
  $sketch = Join-Path $folder "$name.ino"
  $objects = @(
    (Join-Path $build "$name-sketch.o"),
    (Join-Path $build "$name-protocol.o"),
    (Join-Path $build "$name-stubs.o"),
    (Join-Path $build "$name-main.o")
  )
  $common = @('-std=c++17', '-Wall', '-Wextra', '-Werror', '-Wno-error=cpp', '-I', $stub, '-I', $protocolInclude, '-I', $folder) + $extraFlags
  & g++ @common -x c++ -c $sketch -o $objects[0]
  if ($LASTEXITCODE -ne 0) { throw "$name sketch compile failed" }
  & g++ @common -c $protocolSource -o $objects[1]
  if ($LASTEXITCODE -ne 0) { throw "$name protocol compile failed" }
  & g++ @common -c (Join-Path $stub 'arduino_stubs.cpp') -o $objects[2]
  if ($LASTEXITCODE -ne 0) { throw "$name Arduino stub compile failed" }
  & g++ @common -c (Join-Path $stub 'sketch_main.cpp') -o $objects[3]
  if ($LASTEXITCODE -ne 0) { throw "$name test main compile failed" }
  & g++ @objects -o (Join-Path $build "$name.exe")
  if ($LASTEXITCODE -ne 0) { throw "$name link failed" }
}

Compile-Sketch 'car_esp32s3' (Join-Path $root 'embedded\car_esp32s3')
Compile-Sketch 'ground_station_esp32s3' (Join-Path $root 'embedded\ground_station_esp32s3') @('-DHMI_USE_SERIAL_DISPLAY=1')
Compile-Sketch 'car' (Join-Path $root 'car')
Write-Output 'Both Arduino sketches passed host-stub compilation.'
