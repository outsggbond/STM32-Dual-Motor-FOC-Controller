param(
    [string]$Setup    = "",
    [string]$Cmd      = "",
    [int]   $ReadMs   = 8000,
    [int]   $Motor    = 1,
    [double]$SkipS    = 5.0,
    [string]$PortName = "COM4"
)

# Velocity-loop tuning yardstick.
#
# The firmware reports VEL only every 500 ms, and that instantaneous value is
# quantised to 0.19 rad/s (8 ms window on a 4096-count encoder). Judging a gain
# set by eyeballing those numbers is hopeless. Instead we timestamp every
# telemetry line as it arrives and differentiate ANGLE between consecutive
# lines -- that gives the true inter-sample speed, including how much the
# motion pulsates (stick-slip) and whether it ever reverses.
#
# ASCII only on purpose: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK,
# so UTF-8 comments corrupt the parse.

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 200
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true

try { $port.Open() }
catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

# Echo the raw command replies so a rejected/clamped gain set is visible.
foreach ($grp in @($Setup, $Cmd)) {
    foreach ($one in ($grp -split ';')) {
        $t = $one.Trim()
        if ($t -ne "") {
            $port.Write($t + "`r`n")
            Start-Sleep -Milliseconds 120
        }
    }
}

$sw      = [System.Diagnostics.Stopwatch]::StartNew()
$samples = New-Object System.Collections.ArrayList
$replies = New-Object System.Collections.ArrayList
$pat     = "^M$Motor STATE=RUNNING ANGLE=([0-9.]+).*VEL=([-0-9.]+)"

while ($sw.ElapsedMilliseconds -lt $ReadMs) {
    try {
        $line = $port.ReadLine()
        if ($line -match $pat) {
            [void]$samples.Add([pscustomobject]@{
                t = $sw.Elapsed.TotalSeconds
                a = [double]$matches[1]
                v = [double]$matches[2]
            })
        }
        elseif ($line.Trim() -ne "") {
            [void]$replies.Add($line.Trim())
        }
    }
    catch [System.TimeoutException] { }
    catch { break }
}
$port.Close()

# Print any non-telemetry lines (OK/ERR/WARN from the VEL command etc).
foreach ($r in ($replies | Select-Object -Unique)) {
    if ($r -match "VEL|ERR|WARN|ALIGN|SPD") { Write-Output "  reply: $r" }
}

# ALIGN spends its first `drag_ms` in OPEN loop (a voltage ramp that drags the
# rotor up to speed). Only what follows the handover is the closed loop being
# tuned, so drop everything before SkipS or the ramp pollutes every statistic.
$samples = @($samples | Where-Object { $_.t -ge $SkipS })

if ($samples.Count -lt 3) {
    Write-Output "only $($samples.Count) telemetry samples after ${SkipS}s in ${ReadMs}ms -- motor probably not RUNNING"
    exit 0
}

# Unwrap. The encoder is single-turn, so a large negative step is a wrap rather
# than a reversal -- but that fold is only valid while the step stays under 180
# deg. Telemetry lines arrive in BURSTS, and once the rotor is faster than about
# pi/gap_max the fold picks the wrong branch and reports the WRONG SIGN: at
# target 10 rad/s with 400 ms bursts this tool used to print -2.57 rad/s for a
# motor the firmware was correctly measuring at +10.18. The trap is that the
# result looks like clean data (sd 0.03, zero reversals), not like noise.
#
# The fix is to fold onto a branch chosen from an INDEPENDENT reference, not from
# the folded data itself. (An earlier version tried "median speed over the
# shortest half of the gaps", on the theory that short gaps cannot alias --
# that is not what this code does any more; taking a median of already-aliased
# folds still picks the wrong branch, it just does so more quietly.)
# The reference used below is the firmware's own VEL, taken as a median.
$raw = New-Object System.Collections.ArrayList
for ($i = 1; $i -lt $samples.Count; $i++) {
    $d = $samples[$i].a - $samples[$i - 1].a
    if ($d -lt -180.0) { $d += 360.0 }
    if ($d -gt  180.0) { $d -= 360.0 }
    [void]$raw.Add([pscustomobject]@{
        d  = $d
        dt = $samples[$i].t - $samples[$i - 1].t
    })
}

$gapMax = ($raw | Measure-Object -Property dt -Maximum).Maximum

# Reference speed for the branch choice: the firmware's OWN VEL field, taken as
# a median. It is sampled over a real 8 ms window so its sign is never in doubt,
# and although it is coarse (0.19 rad/s quantisation) that is plenty to pick a
# 360-deg branch. Differentiating ANGLE is still worth doing because a 1-count
# delta over 500 ms resolves 0.003 rad/s -- 60x finer -- which is what makes the
# sd/ripple columns able to see pulsation that VEL is blind to.
$vref = 0.0
$vsd  = 0.0
$vmin = 0.0
$vmax = 0.0
$vv = @($samples | ForEach-Object { $_.v }) | Sort-Object
if ($vv.Count -gt 0) {
    $vref = $vv[[int]($vv.Count / 2)]
    $vmin = $vv[0]
    $vmax = $vv[$vv.Count - 1]
    $vavg = ($vv | Measure-Object -Average).Average
    $vsd  = [math]::Sqrt((($vv | ForEach-Object { ($_ - $vavg) * ($_ - $vavg) }) |
             Measure-Object -Sum).Sum / $vv.Count)
}

$speeds = New-Object System.Collections.ArrayList
foreach ($r in $raw) {
    if ($r.dt -le 0.05) { continue }
    $best = $null
    foreach ($k in -8..8) {
        $cand = (($r.d + 360.0 * $k) * [math]::PI / 180.0) / $r.dt
        if ($null -eq $best -or
            [math]::Abs($cand - $vref) -lt [math]::Abs($best - $vref)) { $best = $cand }
    }
    [void]$speeds.Add($best)
}

if ($speeds.Count -lt 2) { Write-Output "not enough intervals"; exit 0 }

$rev = 0
for ($i = 1; $i -lt $speeds.Count; $i++) {
    if ($speeds[$i] * $speeds[$i - 1] -lt 0.0) { $rev++ }
}

$avg = ($speeds | Measure-Object -Average).Average
$mn  = ($speeds | Measure-Object -Minimum).Minimum
$mx  = ($speeds | Measure-Object -Maximum).Maximum
$sd  = [math]::Sqrt(
        (($speeds | ForEach-Object { ($_ - $avg) * ($_ - $avg) }) |
          Measure-Object -Sum).Sum / $speeds.Count)

# ripple = peak-to-peak as a fraction of the mean. Under ~0.5 is smooth;
# over ~1.5 means stick-slip (motor repeatedly stalls and breaks free).
$ripple = if ([math]::Abs($avg) -gt 0.001) { ($mx - $mn) / [math]::Abs($avg) } else { 0.0 }

Write-Output ("n={0}  mean={1:N3}  min={2:N3}  max={3:N3}  sd={4:N3}  ripple={5:N2}  reversals={6}  gapmax={7:N3}s" -f `
    $speeds.Count, $avg, $mn, $mx, $sd, $ripple, $rev, $gapMax)
Write-Output ("    firmware VEL (8ms window): min={0:N2} max={1:N2} sd={2:N2} span={3:N2}" -f `
    $vmin, $vmax, $vsd, ($vmax - $vmin))

# LIMIT-CYCLE GUARD.
# The ANGLE-derived columns above average over 500 ms, so a violent oscillation
# whose period is shorter than that averages out to something that looks like a
# settled speed at the right target. Measured the hard way: kp=0.1 ki=4.0 at
# target 0.5 rad/s printed "mean=0.501 ripple=0.20" while the firmware's own VEL
# was swinging -58..+58 rad/s. The 8 ms VEL is the only field that sees it, so
# judge stability on VEL's span, never on the averaged columns.
if (($vmax - $vmin) -gt 1.0 -and ($vmax - $vmin) -gt 2.0 * [math]::Abs($vref)) {
    Write-Output ("  *** LIMIT CYCLE: firmware VEL swings {0:N2}..{1:N2} around target {2:N2}. The averaged mean above is an oscillation, NOT a settled speed. ***" -f $vmin, $vmax, $vref)
}

# Sanity-check the branch choice. Adjacent 360-deg branches are 2*pi/gapmax
# apart, so if the fitted speed sits more than half a branch away from the
# reference VEL we folded onto the wrong one and the number is garbage -- say so
# rather than print it as if it were data. (This is the check that would have
# caught the original -2.57-for-+10 bug.)
if ($gapMax -gt 0.0) {
    $branch = 2.0 * [math]::PI / $gapMax
    if ([math]::Abs($avg - $vref) -gt 0.5 * $branch) {
        Write-Output ("  WARNING: fitted mean {0:N2} disagrees with the firmware VEL reference {1:N2} by more than half a {2:N2} rad/s unwrap branch - result is unreliable" -f $avg, $vref, $branch)
    }
    if ([math]::Abs($vref) -lt 0.001 -and [math]::Abs($avg) -gt 0.5) {
        Write-Output "  WARNING: firmware VEL reads 0 but ANGLE advances - branch reference is missing, result is unreliable"
    }
}
