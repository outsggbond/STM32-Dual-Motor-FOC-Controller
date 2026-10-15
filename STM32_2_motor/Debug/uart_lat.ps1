# Measure UART command->reply round-trip latency, PC side.
#   .\uart_lat.ps1 -N 30
# Stops both motors first so the 500 ms auto-telemetry is silent and cannot be
# mistaken for a reply (telemetry prints the same "Mx STATE=" format).
# This number is PC->CH340->MCU->parse->reply->CH340->PC: it is an UPPER bound on
# the firmware's own command latency, since the USB-serial path is in the loop.
param(
    [int]   $N        = 30,
    [string]$Cmd      = "M1 STATUS",
    [string]$Match    = "M1 STATE=",
    [string]$PortName = "COM4"
)

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 5
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true
$port.Open()
Start-Sleep -Milliseconds 80

# Silence the periodic telemetry so only our reply can match.
$port.Write("STOPALL`r`n")
Start-Sleep -Milliseconds 600
$port.DiscardInBuffer()

$tmp   = New-Object char[] 512
$times = New-Object System.Collections.ArrayList

for ($i = 0; $i -lt $N; $i++) {
    $port.DiscardInBuffer()
    $sb = New-Object System.Text.StringBuilder
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $port.Write($Cmd + "`r`n")
    $got = -1
    while ($sw.ElapsedMilliseconds -lt 500) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) {
                [void]$sb.Append($tmp, 0, $n)
                if ($sb.ToString().Contains($Match)) { $got = $sw.ElapsedMilliseconds; break }
            }
        } catch [System.TimeoutException] { }
    }
    if ($got -ge 0) { [void]$times.Add($got) }
    Start-Sleep -Milliseconds 30
}
$port.Close()

$sorted = @($times | Sort-Object)
if ($sorted.Count -eq 0) { Write-Output "NO REPLIES -- check port/command"; exit 1 }
$min = $sorted[0]
$max = $sorted[$sorted.Count - 1]
$med = $sorted[[int]($sorted.Count / 2)]
$sum = 0; foreach ($t in $sorted) { $sum += $t }
$avg = [math]::Round($sum / $sorted.Count, 1)

Write-Output ("cmd='{0}'  match='{1}'  replies={2}/{3}" -f $Cmd, $Match, $sorted.Count, $N)
Write-Output ("  min={0} ms   median={1} ms   mean={2} ms   max={3} ms" -f $min, $med, $avg, $max)
Write-Output ("  all: " + ($sorted -join ", "))
