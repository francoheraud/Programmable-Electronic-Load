# Usage: .\pio.ps1 run -e ui_n16r8
#        .\pio.ps1 device list
#        .\pio.ps1 run -e ui_n16r8 -t upload --upload-port COM5
#        .\pio.ps1 device monitor -e ui_n16r8 -p COM5
$pythonExe = Join-Path $env:USERPROFILE 'AppData\Local\Programs\Python\Python312\python.exe'
$pioPackages = Join-Path $env:USERPROFILE '.platformio\penv\Lib\site-packages'
if (!(Test-Path -LiteralPath $pythonExe)) { throw "Python not found: $pythonExe" }
if (!(Test-Path -LiteralPath (Join-Path $pioPackages 'platformio'))) { throw "PlatformIO package missing: $pioPackages" }
$oldPythonPath = $env:PYTHONPATH
$pioExit = 1
Push-Location $PSScriptRoot
try {
    $env:PYTHONPATH = $pioPackages
    & $pythonExe -m platformio @args
    $pioExit = $LASTEXITCODE
} finally {
    $env:PYTHONPATH = $oldPythonPath
    Pop-Location
}
exit $pioExit
