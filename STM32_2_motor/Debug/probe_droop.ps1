# Is M1's below-target mean at 20 rad/s a STEADY-STATE DROOP, or just slow
# integral settling? Read 8 consecutive windows at each speed and print the
# per-window mean. A droop sits still; settling drifts toward the target.
# Also gives the per-window SE directly, which is what a tolerance must clear.
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
# @() or a single-line window unrolls to a String and indexing breaks
function VelWindow([string]$text) {
    $vs = @()
    foreach ($line in @(($text -split "`r?`n") | Where-Object { $_.Trim() -ne "" })) {
        if ($line -match "^M1 STATE=" -and $line -match "VEL=(-?[0-9.]+)") { $vs += [double]$matches[1] }
    }
    if ($vs.Count -eq 0) { return $null }
    $m = ($vs | Measure-Object -Average).Average
    $mn = ($vs | Measure-Object -Minimum).Minimum
    $mx = ($vs | Measure-Object -Maximum).Maximum
    $sd = 0.0
    if ($vs.Count -gt 1) {
        $s = 0.0; foreach ($v in $vs) { $s += ($v - $m) * ($v - $m) }
        $sd = [math]::Sqrt($s / ($vs.Count - 1))
    }
    return [pscustomobject]@{ N = $vs.Count; Mean = $m; Min = $mn; Max = $mx; Sd = $sd }
}

function Trace([double]$target) {
    Write-Output ("== M1 SPD {0} : consecutive 1200 ms windows ==" -f $target)
    [void](C "STOPALL" 400); Start-Sleep -Milliseconds 500
    # clean integral + pin nominal gains (see suite 3.x: STOPALL does not clear it)
    [void](C "M1 VEL 0.05 0.5 0" 300); Start-Sleep -Milliseconds 200
    [void](C "M1 SPD $target" 300)
    Start-Sleep -Milliseconds 700
    for ($i = 1; $i -le 8; $i++) {
        $w = VelWindow (C "M1 STATUS" 400)
        Start-Sleep -Milliseconds 800
        if ($w -eq $null) { Write-Output ("  w{0} : no frame" -f $i); continue }
        Write-Output ("  w{0} : mean={1,7:N2} min={2,7:N2} max={3,7:N2} sd={4,5:N2} n={5}  err={6,6:N2}" -f `
            $i, $w.Mean, $w.Min, $w.Max, $w.Sd, $w.N, ($w.Mean - $target))
    }
    [void](C "M1 STOP" 300); Start-Sleep -Milliseconds 400
}

Trace 20.0
Trace 5.0
Trace 1.0

[void](C "STOPALL" 400); Start-Sleep -Milliseconds 400
[void](C "M1 CLR" 300); [void](C "M1 VEL 0.05 0.5 0" 300)
$p.Close()
