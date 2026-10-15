# Decisive A/B: `SPD 0` (legitimate zero) vs `SPD nan`.
# If SPD 0 gives PWM=0.50,0.49,0.51 (the zero vector) and SPD nan gives
# PWM=0.00,0.00,0.00, then nan really is NaN reaching the PWM compare and not 0.
# Also: is ERR=32 clearable with CLR?
param([string]$PortName = "COM4")

$p = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 40; $p.WriteTimeout = 1000
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open(); Start-Sleep -Milliseconds 200; $p.DiscardInBuffer()

function C([string]$c, [int]$w = 500) {
    $p.DiscardInBuffer(); $p.Write($c + "`r`n")
    $b = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $t = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $w) {
        try { $n = $p.Read($t, 0, $t.Length); if ($n -gt 0) { $b.Write($t, 0, $n) } }
        catch [System.TimeoutException] {} catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($b.ToArray())
}
# last telemetry line only - the freshest truth
function Last([string]$s) {
    $lines = ($s -split "`r?`n") | Where-Object { $_.Trim() -ne "" }
    if ($lines.Count -eq 0) { return "<none>" }
    return $lines[$lines.Count - 1].Trim()
}

[void](C "STOPALL" 500); Start-Sleep -Milliseconds 700
[void](C "CLR" 400);     Start-Sleep -Milliseconds 400

Write-Output "== A: M1 SPD 0 (legitimate zero target) =="
[void](C "M1 SPD 0" 400)
Start-Sleep -Milliseconds 3000
Write-Output ("  after 3s : " + (Last (C "M1 STATUS" 400)))

[void](C "M1 STOP" 400); Start-Sleep -Milliseconds 800
[void](C "CLR" 400);     Start-Sleep -Milliseconds 400
Write-Output ("  cleared  : " + (Last (C "M1 STATUS" 400)))

Write-Output "== B: M1 SPD nan =="
[void](C "M1 SPD nan" 400)
Start-Sleep -Milliseconds 3000
Write-Output ("  after 3s : " + (Last (C "M1 STATUS" 400)))

[void](C "M1 STOP" 400); Start-Sleep -Milliseconds 800
Write-Output ("  stopped  : " + (Last (C "M1 STATUS" 400)))
[void](C "CLR" 500);     Start-Sleep -Milliseconds 600
Write-Output ("  cleared  : " + (Last (C "M1 STATUS" 400)))

Write-Output "== C: can it still run? =="
[void](C "M1 SPD 5" 400)
Start-Sleep -Milliseconds 2500
Write-Output ("  spd5     : " + (Last (C "M1 STATUS" 400)))
[void](C "M1 STOP" 400)
$p.Close()
