param(
  [string]$Port,
  [int]$BaudRate = 115200,
  [string]$OutputDirectory = (Join-Path $PSScriptRoot "downloaded-logs"),
  [int]$TimeoutSeconds = 20,
  [string]$RemotePath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Resolve-RobotPort {
  param([string]$RequestedPort)

  if ($RequestedPort) {
    return $RequestedPort.Trim()
  }

  $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
  if ($ports.Count -eq 1) {
    return $ports[0]
  }
  if ($ports.Count -eq 0) {
    throw "No serial ports were found. Connect the robot and rerun with -Port COMx."
  }

  $joined = $ports -join ", "
  throw "Multiple serial ports were found: $joined. Rerun with -Port COMx."
}

function Normalize-RemotePath {
  param([string]$PathText)

  $normalized = $PathText.Trim().Replace("\", "/")
  if (-not $normalized.StartsWith("/")) {
    $normalized = "/$normalized"
  }
  return $normalized
}

function Read-LineUntil {
  param(
    [System.IO.Ports.SerialPort]$SerialPort,
    [datetime]$Deadline
  )

  while ([datetime]::UtcNow -lt $Deadline) {
    try {
      return $SerialPort.ReadLine().TrimEnd("`r", "`n")
    } catch [System.TimeoutException] {
    }
  }

  throw "Timed out waiting for the robot to respond."
}

$portName = Resolve-RobotPort -RequestedPort $Port
$null = New-Item -ItemType Directory -Force -Path $OutputDirectory

$serial = [System.IO.Ports.SerialPort]::new(
  $portName,
  $BaudRate,
  [System.IO.Ports.Parity]::None,
  8,
  [System.IO.Ports.StopBits]::One
)
$serial.NewLine = "`n"
$serial.ReadTimeout = 250
$serial.WriteTimeout = 2000
$serial.DtrEnable = $true
$serial.RtsEnable = $true

$remoteFilePath = ""
$localPath = ""
$contentLines = [System.Collections.Generic.List[string]]::new()

try {
  $serial.Open()
  Start-Sleep -Milliseconds 1800
  $serial.DiscardInBuffer()

  $command = "DUMPLATEST"
  if ($RemotePath) {
    $remoteRequestPath = Normalize-RemotePath -PathText $RemotePath
    if ($remoteRequestPath -match '^/race_\d{4}\.csv$') {
      $command = "DUMPLOG $remoteRequestPath"
    } else {
      $command = "DUMPFILE $remoteRequestPath"
    }
  }

  $serial.WriteLine($command)
  $deadline = [datetime]::UtcNow.AddSeconds($TimeoutSeconds)

  while (-not $remoteFilePath) {
    $line = Read-LineUntil -SerialPort $serial -Deadline $deadline
    if ($line -match '^ERR ') {
      throw "Robot returned an error: $line"
    }
    if ($line -match '^BEGIN FILE (.+)$') {
      $remoteFilePath = $Matches[1].Trim()
    }
  }

  while ($true) {
    $line = Read-LineUntil -SerialPort $serial -Deadline $deadline
    if ($line -eq "END FILE $remoteFilePath") {
      break
    }
    $contentLines.Add($line)
  }

  $leafName = [System.IO.Path]::GetFileName(($remoteFilePath -replace '/', '\'))
  if (-not $leafName) {
    throw "Robot returned an invalid file path: $remoteFilePath"
  }

  $localPath = Join-Path $OutputDirectory $leafName
  $fileText = ""
  if ($contentLines.Count -gt 0) {
    $fileText = ($contentLines -join "`r`n") + "`r`n"
  }

  [System.IO.File]::WriteAllText($localPath, $fileText, [System.Text.Encoding]::ASCII)

  [pscustomobject]@{
    Port = $portName
    RemotePath = $remoteFilePath
    LocalPath = $localPath
    LineCount = $contentLines.Count
  }
} finally {
  if ($serial.IsOpen) {
    $serial.Close()
  }
  $serial.Dispose()
}
