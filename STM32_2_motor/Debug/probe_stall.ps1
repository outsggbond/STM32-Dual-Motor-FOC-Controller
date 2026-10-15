# Isolate the ERR=32 (STALL) trigger. Each arm starts from a verified ERR=0.
# Answers: (1) does CLR actually clear ERR=32?  (2) does SPD 0 trip it?
# (3) does SPD nan trip it?  (4) does a latched ERR=32 block later motion?
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
# @() forces a real array; a bare pipeline that yields ONE string unrolls to a
# String and [-1] then indexes a Char. That bug ate the previous run's output.
function Last([string]$s) {
    $lines = @(($s -split "`r?`n") | Where-Object { $_.Trim() -ne "" })
    if ($lines.Count -eq 0) { return "<none>" }
    return $lines[$lines.Count - 1].Trim()
}
function Status([string]$tag) { Write-Output ("  {0,-9}: {1}" -f $tag, (Last (C "M1 STATUS" 400))) }

[void](C "STOPALL" 500); Start-Sleep -Milliseconds 700

Write-Output "== does CLR clear a latched ERR=32? =="
Status "before"
Write-Output ("  CLR reply: [" + (Last (C "CLR" 400)) + "]")
Start-Sleep -Milliseconds 400
Status "afterCLR"
Write-Output ("  M1CLR rep: [" + (Last (C "M1 CLR" 400)) + "]")
Start-Sleep -Milliseconds 400
Status "afterM1CLR"

Write-Output "== arm 1: SPD 0 from whatever state we are in =="
[void](C "M1 SPD 0" 400); Start-Sleep -Milliseconds 3000
Status "spd0"

Write-Output "== arm 2: SPD nan =="
[void](C "M1 SPD nan" 400); Start-Sleep -Milliseconds 3000
Status "spdnan"

Write-Output "== arm 3: SPD 5 (does a latch block motion?) =="
[void](C "M1 SPD 5" 400); Start-Sleep -Milliseconds 2500
Status "spd5"
[void](C "M1 STOP" 400)

Write-Output "== arm 4: KP (integral clear) then SPD 5 =="
[void](C "M1 KP 0.05" 400); Start-Sleep -Milliseconds 300
[void](C "M1 SPD 5" 400); Start-Sleep -Milliseconds 2500
Status "kp+spd5"
[void](C "M1 STOP" 400)
$p.Close()
