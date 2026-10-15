# probe_51b.ps1 - the run-to-run DISTRIBUTION of suite step 5.1's measurement.
#
# probe_51.ps1 showed the velocity loop is fine (both motors reach 8 rad/s within
# ~1.4 s and hold it: steady state M1 7.42 / M2 8.44), so the full-suite FAIL
# (M1 mean=6.43) cannot be a control problem. The remaining question is whether
# 6.43 is a rare tail or a typical draw from an under-sampled measurement.
#
# This replays 5.1's EXACT protocol - Stop-All, `M1 SPD 8` + `M2 SPD 8` in one
# raw write, sleep 900 ms, Read-For 3000, mean of the VEL fields - several times
# and prints every draw. That turns "the step is flaky" into a number.
#
# ASCII only: PowerShell 5.1 on this zh-CN system reads .ps1 as GBK.
param([string]$PortName = "COM4", [int]$Reps = 4, [int]$PreSleep = 900, [int]$ReadMs = 3000, [switch]$PosPrecondition)

$ErrorActionPreference = "Stop"

$port = New-Object System.IO.Ports.SerialPort $PortName, 115200, `
    ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 50
$port.Open()
Start-Sleep -Milliseconds 300
$port.DiscardInBuffer()

function Send-Cmd {
    param([string]$Cmd, [int]$WaitMs = 300)
    $port.DiscardInBuffer()
    $port.Write($Cmd + "`r`n")
    $ms = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $WaitMs) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $ms.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($ms.ToArray())
}

function Send-Raw {
    param([string]$Text)
    $port.DiscardInBuffer()
    $port.Write($Text)
}

function Read-For {
    param([int]$Ms = 500)
    $buf = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $Ms) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $buf.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($buf.ToArray())
}

$all1 = @(); $all2 = @()

Write-Host "replaying 5.1 exactly ($Reps reps): Stop-All, both SPD 8, sleep $PreSleep, Read-For $ReadMs"
Write-Host "each rep also splits the window: FIRST half of the frames vs LAST half."
Write-Host "If the first half reads much lower, the window is catching the ramp and the"
Write-Host "fix is to open it later - not to widen the tolerance."
Write-Host ""
for ($i = 1; $i -le $Reps; $i++) {
    # -PosPrecondition replays what step 4.7 leaves behind: M1 parked at 90 deg
    # under an active POS hold, which is the ONE thing the plain probe does not
    # reproduce about the full suite's context. If the low draw of 6.43 comes
    # from that hand-off, it shows up here and nowhere else.
    if ($PosPrecondition) {
        [void](Send-Cmd "M1 ZERO" 300)
        Start-Sleep -Milliseconds 300
        [void](Send-Cmd "M1 POS 90" 300)
        Start-Sleep -Milliseconds 1600
        # Parked, so nothing is streaming and the reply is a single complete line.
        $hold = Send-Cmd "M1 STATUS" 300
        $hold = ($hold -split "`r?`n" | Where-Object { $_.Trim() -ne "" })[0]
        Write-Host ("  rep {0} precondition: parked -> {1}" -f $i, $hold)
    }
    [void](Send-Cmd "STOPALL" 400)
    Start-Sleep -Milliseconds 200
    [void](Send-Cmd "M1 CLR" 250)
    [void](Send-Cmd "M2 CLR" 250)
    Start-Sleep -Milliseconds 300

    Send-Raw "M1 SPD 8`r`nM2 SPD 8`r`n"
    Start-Sleep -Milliseconds $PreSleep
    $r = Read-For $ReadMs

    # 5.1's own parse, kept verbatim so the numbers are comparable.
    $v1 = @(); $v2 = @()
    foreach ($line in @(($r -split "`r?`n") | Where-Object { $_.Trim() -ne "" })) {
        if ($line -match "M1 STATE=\S+.*VEL=(-?[0-9.]+)") { $v1 += [double]$matches[1] }
        if ($line -match "M2 STATE=\S+.*VEL=(-?[0-9.]+)") { $v2 += [double]$matches[1] }
    }
    $m1 = 0.0; $m2 = 0.0
    if ($v1.Count -gt 0) { $m1 = ($v1 | Measure-Object -Average).Average }
    if ($v2.Count -gt 0) { $m2 = ($v2 | Measure-Object -Average).Average }
    $all1 += $m1; $all2 += $m2

    # First half vs last half of M1's frames.
    $h1 = 0.0; $h2 = 0.0
    if ($v1.Count -ge 2) {
        $half = [int][math]::Floor($v1.Count / 2)
        $h1 = ($v1[0..($half - 1)] | Measure-Object -Average).Average
        $h2 = ($v1[$half..($v1.Count - 1)] | Measure-Object -Average).Average
    }

    $d1 = [math]::Abs($m1 - 8.0)
    $d2 = [math]::Abs($m2 - 8.0)
    $verdict = "ok"
    if ($d1 -ge 1.2 -or $d2 -ge 1.2) { $verdict = "WOULD FAIL" }
    Write-Host ("  rep {0}:  M1 mean={1,6:N2} (n={2})  M2 mean={3,6:N2} (n={4})   {5}" -f `
        $i, $m1, $v1.Count, $m2, $v2.Count, $verdict)
    Write-Host ("           M1 first-half={0,6:N2}   last-half={1,6:N2}   (ramp drag = {2:N2})" -f `
        $h1, $h2, ($h2 - $h1))
}

Write-Host ""
$lo1 = ($all1 | Measure-Object -Minimum).Minimum
$hi1 = ($all1 | Measure-Object -Maximum).Maximum
$av1 = ($all1 | Measure-Object -Average).Average
$lo2 = ($all2 | Measure-Object -Minimum).Minimum
$hi2 = ($all2 | Measure-Object -Maximum).Maximum
$av2 = ($all2 | Measure-Object -Average).Average
Write-Host ("M1 across {0} reps: min={1:N2}  mean={2:N2}  max={3:N2}  spread={4:N2}" -f $Reps, $lo1, $av1, $hi1, ($hi1 - $lo1))
Write-Host ("M2 across {0} reps: min={1:N2}  mean={2:N2}  max={3:N2}  spread={4:N2}" -f $Reps, $lo2, $av2, $hi2, ($hi2 - $lo2))
Write-Host ""
Write-Host ("5.1 requires |mean - 8| < 1.2, i.e. M1 in [{0:N2}, {1:N2}]" -f 6.8, 9.2)

[void](Send-Cmd "STOPALL" 400)
Start-Sleep -Milliseconds 200
[void](Send-Cmd "M1 CLR" 300)
[void](Send-Cmd "M2 CLR" 300)
$port.Close()
Write-Host ""
Write-Host "done"
