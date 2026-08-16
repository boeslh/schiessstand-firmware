# ============================================================================
#  flash.ps1 - Kompiliert die Firmware und uebertraegt sie per USB auf den ESP32
# ----------------------------------------------------------------------------
#  Aufruf:   .\flash.ps1                  (Port = COM3, Default)
#            .\flash.ps1 -Port COM5       (anderer Port)
#
#  Falls der Upload bei "Connecting...." haengt (haengt vom USB-Treiber des
#  ESP32-Boards ab, manche Boards resetten nicht automatisch): BOOT-Taste am
#  ESP32 gedrueckt HALTEN, bis "Connected to ESP32" erscheint, dann loslassen.
#  Das Skript weist an der richtigen Stelle automatisch darauf hin und bietet
#  bei Fehlschlag einen erneuten Versuch an.
# ============================================================================

param(
    [string]$Port       = "COM3",
    [string]$ProjectDir = "C:\Users\huber\Documents\PlatformIO\Projects\260801-182019-esp32dev",
    [string]$Pio        = "C:\Users\huber\.platformio\penv\Scripts\pio.exe",
    [int]$MaxAttempts   = 3
)

# UTF-8 erzwingen - sonst stuerzt PlatformIO beim Anzeigen der Fortschrittsbalken
# (Blockzeichen in der Upload-Prozentanzeige) auf Windows-Konsolen (Codepage
# cp1252) ab.
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUTF8       = "1"

function Write-Status {
    param([string]$Msg, [string]$Color = "Cyan")
    Write-Host "==> $Msg" -ForegroundColor $Color
}

if (-not (Test-Path $Pio)) {
    Write-Host "FEHLER: pio.exe nicht gefunden unter '$Pio'." -ForegroundColor Red
    Write-Host "Pfad ggf. mit -Pio <Pfad> angeben." -ForegroundColor Red
    exit 1
}
if (-not (Test-Path $ProjectDir)) {
    Write-Host "FEHLER: Projektverzeichnis nicht gefunden: '$ProjectDir'." -ForegroundColor Red
    exit 1
}

Push-Location $ProjectDir
try {
    $attempt = 1
    $success = $false

    while (-not $success -and $attempt -le $MaxAttempts) {
        Write-Status "Versuch $attempt von $MaxAttempts - baue Firmware und uebertrage auf $Port ..."
        $failed = $false

        & $Pio run --target upload --upload-port $Port 2>&1 | ForEach-Object {
            $line = $_.ToString()
            Write-Host $line

            switch -Regex ($line) {
                '^Processing '                       { Write-Status "Kompiliere Firmware ..." }
                'Looking for upload port'             { Write-Status "Suche ESP32 an $Port ..." }
                'Connecting\.'                        {
                    Write-Status "Verbinde mit ESP32 ... Haengt das laenger als 5-10 Sekunden: BOOT-Taste am ESP32 jetzt gedrueckt HALTEN." "Yellow"
                }
                'Connected to ESP32'                  { Write-Status "Verbunden - BOOT-Taste kann losgelassen werden." "Green" }
                "Writing '.*bootloader\.bin'"         { Write-Status "Schreibe Bootloader ..." }
                "Writing '.*partitions\.bin'"         { Write-Status "Schreibe Partitionstabelle ..." }
                "Writing '.*boot_app0\.bin'"          { Write-Status "Schreibe boot_app0 ..." }
                "Writing '.*firmware\.bin'"           { Write-Status "Schreibe Hauptfirmware (dauert ca. 15-20 Sekunden) ..." }
                'Hard resetting'                      { Write-Status "Starte ESP32 neu ..." }
                'Failed to connect|Timed out waiting for packet header|No serial data received' {
                    $failed = $true
                }
            }
        }

        if ($LASTEXITCODE -eq 0 -and -not $failed) {
            $success = $true
            Write-Status "Firmware erfolgreich uebertragen." "Green"
        } else {
            Write-Status "Uebertragung fehlgeschlagen (Versuch $attempt von $MaxAttempts)." "Red"
            if ($attempt -lt $MaxAttempts) {
                Write-Host ""
                Write-Host "BOOT-Taste am ESP32 gedrueckt HALTEN, dann Enter druecken, um es erneut zu versuchen ..." -ForegroundColor Yellow
                Read-Host | Out-Null
            }
        }
        $attempt++
    }

    if (-not $success) {
        Write-Host ""
        Write-Status "Uebertragung nach $MaxAttempts Versuchen fehlgeschlagen." "Red"
        exit 1
    }
}
finally {
    Pop-Location
}
