# UGV helper for Windows (PowerShell). Same commands as ugv.sh:
#
#   .\ugv.ps1                  build + flash + monitor
#   .\ugv.ps1 build            build firmware in Docker
#   .\ugv.ps1 flash [COM5]     flash from the host with esptool
#   .\ugv.ps1 monitor [COM5]   serial monitor (pyserial, ships with esptool)
#   .\ugv.ps1 shell            interactive idf.py shell inside the container
#
# COM port is auto-detected when omitted. If script execution is blocked:
#   powershell -ExecutionPolicy Bypass -File ugv.ps1

param(
    [string]$Cmd = "all",
    [string]$Port = ""
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$BaudFlash = 460800
$BaudMonitor = 115200

function Get-BoardPort {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 0) { return "" }
    # The most recently plugged adapter is usually the highest-numbered port.
    return ($ports | Sort-Object { [int]($_ -replace '\D', '0') })[-1]
}

function Invoke-Compose {
    docker compose version *> $null
    if ($LASTEXITCODE -eq 0) { docker compose @args } else { docker-compose @args }
    if ($LASTEXITCODE -ne 0) { throw "docker compose failed (is Docker Desktop running?)" }
}

function Invoke-Esptool {
    if (Get-Command esptool -ErrorAction SilentlyContinue) { esptool @args }
    else { python -m esptool @args }
    if ($LASTEXITCODE -ne 0) { throw "esptool failed" }
}

function Do-Build { Invoke-Compose run --rm build }

function Do-Flash([string]$p) {
    if (-not $p) { throw "No COM port found. Plug in the board or pass it: .\ugv.ps1 flash COM5" }
    Write-Host "Flashing on $p"
    Invoke-Esptool -p $p -b $BaudFlash `
        --before default-reset --after hard-reset --chip esp32 `
        write-flash --flash-mode dio --flash-size detect --flash-freq 40m `
        0x1000 build/bootloader/bootloader.bin `
        0x8000 build/partition_table/partition-table.bin `
        0x10000 build/robots.bin
}

function Do-Monitor([string]$p) {
    if (-not $p) { throw "No COM port found." }
    # pyserial is installed together with esptool. Exit with Ctrl+]
    python -m serial.tools.miniterm --raw $p $BaudMonitor
}

if (-not $Port) { $Port = Get-BoardPort }

switch ($Cmd) {
    "build"   { Do-Build }
    "flash"   { Do-Flash $Port }
    "monitor" { Do-Monitor $Port }
    "all"     { Do-Build; Do-Flash $Port; Do-Monitor $Port }
    "clean"   { Invoke-Compose run --rm build idf.py fullclean }
    "shell"   { Invoke-Compose run --rm build bash }
    default   { Write-Host "Usage: .\ugv.ps1 [build|flash|monitor|all|clean|shell] [COMx]" }
}
