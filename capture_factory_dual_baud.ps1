param(
    [int]$SwitchAfterMs = 120,
    [int]$CaptureAfterMs = 600
)

$ErrorActionPreference = 'Stop'
$port = [System.IO.Ports.SerialPort]::new(
    'COM9', 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One
)
$port.ReadTimeout = 50
$port.DtrEnable = $false
$port.RtsEnable = $true

function Read-Available([System.IO.Ports.SerialPort]$Serial, [System.IO.MemoryStream]$Target) {
    $available = $Serial.BytesToRead
    if ($available -gt 0) {
        $chunk = [byte[]]::new($available)
        $read = $Serial.Read($chunk, 0, $chunk.Length)
        $Target.Write($chunk, 0, $read)
    }
}

$rom = [System.IO.MemoryStream]::new()
$app = [System.IO.MemoryStream]::new()
try {
    $port.Open()
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 120
    $port.RtsEnable = $false

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $SwitchAfterMs) {
        Start-Sleep -Milliseconds 5
        Read-Available $port $rom
    }

    $port.BaudRate = 1000000
    $timer.Restart()
    while ($timer.ElapsedMilliseconds -lt $CaptureAfterMs) {
        Start-Sleep -Milliseconds 5
        Read-Available $port $app
    }
}
finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}

$romBytes = $rom.ToArray()
$appBytes = $app.ToArray()
$rom.Dispose()
$app.Dispose()

$encoding = [System.Text.Encoding]::GetEncoding(28591)
$romText = $encoding.GetString($romBytes)
$appText = $encoding.GetString($appBytes)
$romRuns = [System.Text.RegularExpressions.Regex]::Matches($romText, '[\x20-\x7E\r\n]{4,}') |
    ForEach-Object { $_.Value }
$appRuns = [System.Text.RegularExpressions.Regex]::Matches($appText, '[\x20-\x7E\r\n]{4,}') |
    ForEach-Object { $_.Value }

"SwitchAfterMs=$SwitchAfterMs ROMBytes=$($romBytes.Length) AppBytes=$($appBytes.Length)"
'--- ROM 115200 ---'
$romRuns | Select-Object -First 40
'--- APP 1000000 ---'
$appRuns | Select-Object -First 80
