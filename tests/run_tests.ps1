$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$protocol = Join-Path $root 'embedded\shared_protocol\src\DTaskProtocol.cpp'
$test = Join-Path $PSScriptRoot 'protocol_tests.cpp'
$output = Join-Path $PSScriptRoot 'protocol_tests.exe'

g++ -std=c++17 -Wall -Wextra -Werror `
  -I (Join-Path $root 'embedded\shared_protocol\src') `
  -I (Join-Path $root 'embedded\ground_station_esp32s3') `
  $protocol $test -o $output
if ($LASTEXITCODE -ne 0) { throw '测试程序编译失败' }

& $output
if ($LASTEXITCODE -ne 0) { throw '测试失败' }

$lineSensorTest = Join-Path $PSScriptRoot 'line_sensor_tests.cpp'
$lineSensorOutput = Join-Path $PSScriptRoot 'line_sensor_tests.exe'
$stubs = Join-Path $PSScriptRoot 'arduino_stubs\arduino_stubs.cpp'
g++ -std=c++17 -Wall -Wextra -Werror -Wno-error=cpp `
  -I (Join-Path $PSScriptRoot 'arduino_stubs') `
  -I (Join-Path $root 'embedded\shared_protocol\src') `
  -I (Join-Path $root 'embedded\car_esp32s3') `
  $lineSensorTest $stubs -o $lineSensorOutput
if ($LASTEXITCODE -ne 0) { throw 'I2C line sensor test compile failed' }

& $lineSensorOutput
if ($LASTEXITCODE -ne 0) { throw 'I2C line sensor test failed' }
