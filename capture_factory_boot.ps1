param(
    [int]$Baud = 1000000,
    [int]$Seconds = 12
)

$ErrorActionPreference = 'Stop'

$port = [System.IO.Ports.SerialPort]::new(
    'COM9',
    $Baud,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$port.Encoding = [System.Text.Encoding]::GetEncoding(28591)
$port.ReadTimeout = 100
$port.WriteTimeout = 100
$port.DtrEnable = $false
$port.RtsEnable = $true

$capturePath = "A:\NEXarm\Claude\factory_boot_capture_$Baud.bin"
$buffer = [System.IO.MemoryStream]::new()
try {
    $port.Open()
    $port.DtrEnable = $false
    $port.RtsEnable = $true
    Start-Sleep -Milliseconds 150
    $port.RtsEnable = $false

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        Start-Sleep -Milliseconds 20
        $available = $port.BytesToRead
        if ($available -gt 0) {
            $chunk = [byte[]]::new($available)
            $read = $port.Read($chunk, 0, $chunk.Length)
            $buffer.Write($chunk, 0, $read)
        }
    }
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
    $port.Dispose()
}

$bytes = $buffer.ToArray()
$buffer.Dispose()
[System.IO.File]::WriteAllBytes($capturePath, $bytes)

$text = [System.Text.Encoding]::GetEncoding(28591).GetString($bytes)
$lines = [System.Text.RegularExpressions.Regex]::Matches($text, '[\x20-\x7E]{4,}') |
    ForEach-Object { $_.Value }

"CapturedBytes=$($bytes.Length)"
"Baud=$Baud"
"CapturePath=$capturePath"
$lines | Select-Object -First 100
