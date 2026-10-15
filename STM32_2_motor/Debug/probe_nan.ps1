# Probe: what EXACTLY does `M1 SPD nan` reply, and what does it do to the loop?
# The suite's 6.3 reported accepted=False while printing an "OK" first line -
# those cannot both be true, so read the raw bytes instead of guessing.
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
# show every non-empty line, so nothing hides behind "first line"
function Show([string]$tag, [string]$s) {
    $lines = ($s -split "`r?`n") | Where-Object { $_.Trim() -ne "" }
    if ($lines.Count -eq 0) { Write-Output ("{0}: <no reply>" -f $tag); return }
    foreach ($l in $lines) { Write-Output ("{0}: [{1}]" -f $tag, $l.Trim()) }
}

[void](C "STOPALL" 500)
Start-Sleep -Milliseconds 600
Show "baseline " (C "M1 STATUS" 400)

Write-Output "--- inject nan (LIMIT 1.0 in force) ---"
Show "limit    " (C "M1 LIMIT 1.0" 400)
Show "spd-nan  " (C "M1 SPD nan" 900)
Start-Sleep -Milliseconds 700
Show "status   " (C "M1 STATUS" 400)
[void](C "STOPALL" 600)
Start-Sleep -Milliseconds 800
Show "post-stop" (C "M1 STATUS" 400)

Write-Output "--- recovery: does a gain command clear the poisoned integral? ---"
Show "kp       " (C "M1 KP 0.05" 400)
Show "limit    " (C "M1 LIMIT 6.6" 400)
[void](C "M1 SPD 5" 400)
Start-Sleep -Milliseconds 2500
Show "spd5     " (C "M1 STATUS" 400)
[void](C "M1 STOP" 400)
$p.Close()
