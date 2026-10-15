# How reproducible is the SUITE'S OWN 3 s window mean? Reproduce the exact
# Run-Spd timing (SPD -> 700 ms settle -> read 3000 ms) N times per speed and
# print every mean. This is the direct empirical basis for a tolerance: the
# frame count (n~7-8) does NOT set the error bar, because consecutive frames
# are strongly correlated (the velocity wanders on a ~1-3 s timescale, so a
# 3 s window samples only 1-3 independent pieces of it).
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

function Rep([double]$target, [int]$reps) {
    Write-Output ("== M1 SPD {0} : {1} x (700 ms settle + 3000 ms read) ==" -f $target, $reps)
    $means = @()
    for ($i = 1; $i -le $reps; $i++) {
        [void](C "STOPALL" 400); Start-Sleep -Milliseconds 400
        [void](C "M1 VEL 0.05 0.5 0" 300); Start-Sleep -Milliseconds 200
        [void](C "M1 SPD $target" 300)
        Start-Sleep -Milliseconds 700
        $vs = @()
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        while ($sw.ElapsedMilliseconds -lt 3000) {
            $t = C "M1 STATUS" 400
            foreach ($line in @(($t -split "`r?`n") | Where-Object { $_.Trim() -ne "" })) {
                if ($line -match "^M1 STATE=" -and $line -match "VEL=(-?[0-9.]+)") { $vs += [double]$matches[1] }
            }
            Start-Sleep -Milliseconds 100
        }
        if ($vs.Count -eq 0) { Write-Output ("  #{0} : no frame" -f $i); continue }
        $m = ($vs | Measure-Object -Average).Average
        $sd = 0.0
        if ($vs.Count -gt 1) {
            $s = 0.0; foreach ($v in $vs) { $s += ($v - $m) * ($v - $m) }
            $sd = [math]::Sqrt($s / ($vs.Count - 1))
        }
        $means += $m
        Write-Output ("  #{0} : mean={1,7:N2} err={2,6:N2} sd={3,5:N2} n={4}" -f $i, $m, ($m - $target), $sd, $vs.Count)
    }
    if ($means.Count -gt 1) {
        $lo = ($means | Measure-Object -Minimum).Minimum
        $hi = ($means | Measure-Object -Maximum).Maximum
        $am = ($means | Measure-Object -Average).Average
        Write-Output ("  --> means {0:N2} .. {1:N2}  (avg {2:N2}, half-spread {3:N2}, worst |err| {4:N2})" -f `
            $lo, $hi, $am, (($hi - $lo) / 2), ([math]::Max([math]::Abs($lo - $target), [math]::Abs($hi - $target))))
    }
    [void](C "M1 STOP" 300); Start-Sleep -Milliseconds 300
}

Rep 20.0 6
Rep 5.0 6
Rep 1.0 6

[void](C "STOPALL" 400); Start-Sleep -Milliseconds 400
[void](C "M1 CLR" 300); [void](C "M1 VEL 0.05 0.5 0" 300)
$p.Close()
