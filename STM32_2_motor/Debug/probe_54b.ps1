# 5.4 again, but parse per motor. With both motors RUNNING the reply window of
# `M1 STATUS` also contains M2's unsolicited telemetry, so "last line" is not
# M1's line. Extract every M1 frame and every M2 frame separately, in order.
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
function F([string]$s, [string]$f) {
    if ($s -match "$f=(-?[0-9.]+)") { return [double]$matches[1] }
    return -999.0
}
# print every frame for one motor, in arrival order
function Frames([string]$tag, [string]$text, [string]$motor) {
    $i = 0
    foreach ($line in @(($text -split "`r?`n") | Where-Object { $_.Trim() -ne "" })) {
        $t = $line.Trim()
        if ($t.StartsWith($motor + " ")) {
            $i++
            $st = "?"; if ($t -match "STATE=(\S+)") { $st = $matches[1] }
            Write-Output ("    {0} #{1} {2,-9} ANGLE={3,8:N2} TARGET={4,8:N2} VEL={5,7:N2} ERR={6}" -f `
                $tag, $i, $st, (F $t "ANGLE"), (F $t "TARGET"), (F $t "VEL"), (F $t "ERR"))
        }
    }
    if ($i -eq 0) { Write-Output ("    {0} : NO {1} FRAME IN WINDOW" -f $tag, $motor) }
}

[void](C "STOPALL" 500); Start-Sleep -Milliseconds 700
[void](C "M1 CLR" 300); [void](C "M2 CLR" 300)
[void](C "M1 KP 0.05" 300); [void](C "M2 KP 0.05" 300)
Start-Sleep -Milliseconds 400

Write-Output "== 5.4 sequence: M2 RUNNING while M1 goes to POS 120 =="
Write-Output ("  ZERO reply: " + ((@(($(C "M1 ZERO" 400) -split "`r?`n") | Where-Object { $_.Trim() -ne "" })[0]).Trim()))
Frames "zero " (C "M1 STATUS" 500) "M1"

[void](C "M2 SPD 8" 200)
Start-Sleep -Milliseconds 300
Frames "pre  " (C "M1 STATUS" 500) "M1"

[void](C "M1 POS 120" 300)
Start-Sleep -Milliseconds 2000
Frames "t2.0 " (C "M1 STATUS" 500) "M1"
Start-Sleep -Milliseconds 3000
Frames "t5.0 " (C "M1 STATUS" 500) "M1"
Frames "M2at5" (C "M2 STATUS" 500) "M2"

[void](C "STOPALL" 500); Start-Sleep -Milliseconds 800
$p.Close()
