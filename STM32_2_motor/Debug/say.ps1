# Send one command and decode the reply with a chosen encoding.
#   .\say.ps1 -Cmd "M1 XXX" -Enc utf8
#   .\say.ps1 -Cmd "HELP"   -Enc gbk
# Exists because ser.ps1 uses the SerialPort default (ASCII), which turns every
# Chinese character into '?'. ASCII-only source on purpose (PS 5.1 + GBK console).
param(
    [string]$Cmd = "STATUS",
    [int]   $ReadMs = 1200,
    [string]$Enc = "utf8",
    [string]$PortName = "COM4"
)

switch ($Enc.ToLower()) {
    "utf8"  { $encoding = [System.Text.Encoding]::UTF8 }
    "gbk"   { $encoding = [System.Text.Encoding]::GetEncoding(936) }
    default { $encoding = [System.Text.Encoding]::ASCII }
}

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true

try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

$port.Write($Cmd + "`r`n")

$ms  = New-Object System.IO.MemoryStream
$sw  = [System.Diagnostics.Stopwatch]::StartNew()
$tmp = New-Object byte[] 1024
while ($sw.ElapsedMilliseconds -lt $ReadMs) {
    try {
        $n = $port.Read($tmp, 0, $tmp.Length)
        if ($n -gt 0) { $ms.Write($tmp, 0, $n) }
    } catch [System.TimeoutException] {
    } catch { Write-Output "READ FAILED: $($_.Exception.Message)"; break }
}
$port.Close()

# Drop a stray UTF-8 BOM if the firmware emits one.
$bytes = $ms.ToArray()
if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
    $bytes = $bytes[3..($bytes.Length - 1)]
}
$encoding.GetString($bytes)
