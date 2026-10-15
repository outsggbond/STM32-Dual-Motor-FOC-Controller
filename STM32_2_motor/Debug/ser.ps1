# Send one command to the firmware over the ST-Link VCP and read the reply back.
#   .\ser.ps1 -Cmd "M2 STATUS" -ReadMs 700
# Keep this file ASCII-only: Windows PowerShell 5.1 reads .ps1 as ANSI/GBK,
# so UTF-8 Chinese comments get mangled and can even eat the following newline.
# The firmware treats both CR and LF as line ends and ignores empty lines,
# so terminating with CRLF is safe.
param(
    [string]$Cmd    = "",
    [int]   $ReadMs = 700,
    [string]$PortName = "COM4"
)

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true

try { $port.Open() }
catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }

Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

# $Cmd may hold several commands separated by ';' -- they are sent in order with
# a short gap so the firmware parses them as distinct lines.
foreach ($one in ($Cmd -split ';')) {
    $t = $one.Trim()
    if ($t -ne "") {
        $port.Write($t + "`r`n")
        Start-Sleep -Milliseconds 120
    }
}

$sw  = [System.Diagnostics.Stopwatch]::StartNew()
$buf = New-Object System.Text.StringBuilder
$tmp = New-Object char[] 1024

while ($sw.ElapsedMilliseconds -lt $ReadMs) {
    try {
        $n = $port.Read($tmp, 0, $tmp.Length)
        if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) }
    } catch [System.TimeoutException] {
    } catch {
        Write-Output "READ FAILED: $($_.Exception.Message)"; break
    }
}

$port.Close()
$buf.ToString()
