$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$config = Join-Path $root "arduino-cli.yaml"

function Resolve-ArduinoCli {
    $fromPath = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }

    $candidates = @(
        "${env:ProgramFiles}\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        "${env:ProgramFiles(x86)}\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
        "${env:LOCALAPPDATA}\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    )

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return $candidate
        }
    }

    throw "Could not find arduino-cli. Install Arduino CLI or Arduino IDE 2.x, then ensure arduino-cli is on PATH."
}

$cli = Resolve-ArduinoCli
& $cli @args --config-file $config
