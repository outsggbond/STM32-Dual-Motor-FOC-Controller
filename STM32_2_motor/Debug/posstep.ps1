param(
    [string] $Steps         = "1,10,45,90,170",
    [int]    $Repeat        = 2,
    [string] $Motors        = "1,2",
    [string] $Traj          = "",
    [int]    $SettleHoldMs  = 300,
    [double] $TolDeg        = 0.10,
    [int]    $MoveTimeoutMs = 8000,
    [string] $PortName      = "COM4"
)

# Position step-response measurement. Read-only: sends POS commands and logs
# ANGLE as fast as the link allows. No firmware change, no gain change.
#
# Why this exists: "smooth and fast" cannot be judged from a settle time alone.
# A move that takes 1.5 s could be a clean ramp, or 0.1 s of motion plus 1.4 s
# of creeping tail. Those need opposite fixes. The trace tells them apart.
#
# Polling: the 500 ms auto-telemetry is far too coarse -- a 90 deg move is over
# before the second auto line. So we send "M<n> STATUS" explicitly and read the
# reply. A STATUS line is ~100 chars; at 115200 8N1 that is ~9 ms of wire time,
# which sets the sample floor near 100 Hz. DiscardInBuffer() before each send
# matters: otherwise a stale auto-telemetry line sitting in the buffer gets
# parsed as if it were this sample, and the whole trace shifts in time.
#
# One motor at a time, on purpose. The two share a 12 V bus, so a simultaneous
# run measures the supply coupling as much as the controller. Clean single-motor
# traces first; a paired run afterwards if the coupling is what we want to see.
#
# ASCII only on purpose: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK.
#
# TolDeg = the "has it stopped" tolerance, in degrees. It MUST be slightly
# larger than one encoder count (360/4096 = 0.0879 deg). A tighter test demands
# bit-identical readings, and a rotor parked exactly on a code boundary dithers
# between two adjacent codes indefinitely -- the first run after the braking
# curve change failed to latch ZERO for precisely this reason (222.01 <-> 222.10,
# motor unpowered, VEL=0.00). 0.10 accepts one-count dither, which IS the
# physical resolution floor and cannot be improved on.
# Only the stationarity test uses this. The at-target test stays at 0.09
# (1 count) so the stick-slip settle trap is still guarded -- see below.

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 50
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true

try { $port.Open() }
catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

function Send-Cmd([string]$c) { $port.Write($c + "`r`n") }

# Drain for a bounded window, return everything seen. Used for setup waits where
# we do not care which line we get.
function Read-Lines([int]$ms) {
    $sw  = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 1024
    while ($sw.ElapsedMilliseconds -lt $ms) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return ($buf.ToString() -split "`r?`n")
}

# ★ Capture every group into a local BEFORE any further -match. $Matches is a
# single automatic variable that the next -match overwrites, and the ERR
# pattern has only one group -- reading $Matches[2]/$Matches[3] after it yields
# $null, and [double]$null is 0.0. That silently reported ANGLE=0.00 on every
# sample in an earlier script while the motors were moving normally.
function Parse-Statuses([string[]]$lines) {
    $out = @{}
    foreach ($l in $lines) {
        if ($l -match '^M(\d)\s+STATE=(\w+)\s+ANGLE=([-0-9.]+)') {
            $mv    = [int]$Matches[1]
            $state = $Matches[2]
            $ang   = [double]$Matches[3]
            $err   = -1
            if ($l -match '\bERR=(\d+)') { $err = [int]$Matches[1] }
            $out[$mv] = @{ State = $state; Angle = $ang; Err = $err }
        }
    }
    return $out
}

function Wrap180([double]$d) {
    while ($d -gt  180.0) { $d -= 360.0 }
    while ($d -lt -180.0) { $d += 360.0 }
    return $d
}

# One fresh sample for one motor. DiscardInBuffer first so the reply we parse is
# the reply to the STATUS we just sent, not an older auto-telemetry line.
function Sample([int]$mv, [int]$timeoutMs = 60) {
    $port.DiscardInBuffer()
    Send-Cmd "M$mv STATUS"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $sb = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 256
    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            for ($i = 0; $i -lt $n; $i++) {
                $ch = $tmp[$i]
                if ($ch -eq "`n") {
                    $s = $sb.ToString()
                    if ($s -match "^M$mv\s+STATE=(\w+)\s+ANGLE=([-0-9.]+)") {
                        return @{ State = $Matches[1]; Angle = [double]$Matches[2] }
                    }
                    [void]$sb.Clear()
                } elseif ($ch -ne "`r") {
                    [void]$sb.Append($ch)
                }
            }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return $null
}

# Wait until ANGLE agrees for a window of WALL TIME. Never a count of samples:
# a sample-counted criterion silently depends on the poll rate, and reported
# fake overshoot when an earlier script was sped up.
function Wait-Stable([int]$mv, [int]$timeoutMs, [int]$holdMs = 800) {
    $sw    = [System.Diagnostics.Stopwatch]::StartNew()
    $prev  = Sample $mv
    $since = -1
    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        Start-Sleep -Milliseconds 40
        $cur = Sample $mv
        if ($cur -and $prev -and
            [math]::Abs($cur.Angle - $prev.Angle) -lt $TolDeg) {
            if ($since -lt 0) { $since = $sw.ElapsedMilliseconds }
            elseif (($sw.ElapsedMilliseconds - $since) -ge $holdMs) { return $cur }
        } else {
            $since = -1
        }
        $prev = $cur
    }
    return $null
}

# ---- latch a known origin on every motor -------------------------------
$mvList = @()
foreach ($motor in ($Motors -split ',')) { $mvList += [int]$motor.Trim() }

# Optionally retune the position-loop braking curve before the run. This is the
# whole point of the live TRAJ command: the amax ladder (60 -> 120 -> 200 -> 280)
# would otherwise need a recompile and reflash per rung, which is slow enough
# that the tuning never actually gets done. Applies to every motor in the run.
# The firmware clamps the value and echoes what it took, so we print the echo
# rather than assume the setting landed.
if ($Traj -ne "") {
    foreach ($mv in $mvList) {
        Send-Cmd ("M{0} TRAJ {1}" -f $mv, $Traj)
        Start-Sleep -Milliseconds 120
    }
    Start-Sleep -Milliseconds 200
    foreach ($l in (Read-Lines 400)) {
        if ($l -match 'TRAJ') { Write-Output ("  " + $l.Trim()) }
    }
}

Send-Cmd "STOPALL"
Start-Sleep -Milliseconds 500
[void](Read-Lines 300)

$origin = @{}
foreach ($mv in $mvList) {
    $m = "M$mv"
    Send-Cmd "$m STOP"
    if (-not (Wait-Stable $mv 8000 600)) {
        Write-Output "$m : rotor will not settle, cannot latch ZERO."; $port.Close(); exit 1
    }
    Send-Cmd "$m POS 0"
    if (-not (Wait-Stable $mv 15000 600)) {
        Write-Output "$m : will not hold under POS 0, cannot latch ZERO."; $port.Close(); exit 1
    }
    $ok = $false
    for ($t = 1; $t -le 8; $t++) {
        Send-Cmd "$m ZERO"
        Start-Sleep -Milliseconds 500
        $a1 = Sample $mv
        Start-Sleep -Milliseconds 200
        $a2 = Sample $mv
        # Same one-encoder-count rule as Wait-Stable: a sub-count threshold can
        # never be met by a rotor dithering on a code boundary. This check is the
        # one that matters most -- if it passes while the rotor is still coasting,
        # the origin it records is wrong by that coast, and EVERY subsequent
        # reading in the run is offset by the same amount. That is exactly what
        # produced the amax=120 sweep where all four large moves "ended" at
        # -0.44 deg and reported settle=never: a 5-count bad origin, not a
        # control failure (verified by re-running those moves directly).
        if ($a1 -and $a2 -and [math]::Abs($a1.Angle - $a2.Angle) -lt $TolDeg) {
            $origin[$mv] = $a2.Angle; $ok = $true; break
        }
    }
    if (-not $ok) {
        Write-Output "$m : NO TELEMETRY - could not latch an origin."; $port.Close(); exit 1
    }
    Write-Output ("M{0} origin = {1:F2} deg" -f $mv, $origin[$mv])
}
Write-Output ""

$csv = New-Object System.Text.StringBuilder
[void]$csv.AppendLine("motor,step_deg,rep,t_ms,deg")

$stepList = @()
foreach ($s in ($Steps -split ',')) { $stepList += [double]$s.Trim() }

Write-Output "  motor  step   rep   start   t90   peak    over   settle   final    err   n"
Write-Output "  -----  -----  ---  ------  ----  -----  ------  -------  ------  -----  ---"

foreach ($mv in $mvList) {
    foreach ($step in $stepList) {
        for ($rep = 1; $rep -le $Repeat; $rep++) {

            # Return to the origin and let it truly stop before the next step.
            Send-Cmd ("M{0} POS 0" -f $mv)
            if (-not (Wait-Stable $mv 15000 400)) {
                Write-Output ("  M{0}  {1,5}  {2,3}  -- did not return to origin" -f $mv, $step, $rep)
                continue
            }

            # ---- fire the step and log the transient ---------------------
            $trace = New-Object System.Collections.ArrayList
            $sw    = [System.Diagnostics.Stopwatch]::StartNew()
            Send-Cmd ("M{0} POS {1:F4}" -f $mv, $step)

            $prev  = $null
            $since = -1
            while ($sw.ElapsedMilliseconds -lt $MoveTimeoutMs) {
                $s = Sample $mv 60
                if ($s) {
                    $rel = Wrap180 ($s.Angle - $origin[$mv])
                    [void]$trace.Add(@{ t = $sw.ElapsedMilliseconds; a = $rel })
                    [void]$csv.AppendLine(("{0},{1},{2},{3},{4:F3}" -f `
                        $mv, $step, $rep, $sw.ElapsedMilliseconds, $rel))

                    # Stop only when the reading is BOTH stationary AND at the
                    # target. Requiring stationarity alone fires during a
                    # stick-slip stall -- the rotor pauses for a few hundred ms
                    # mid-creep, which an earlier version accepted as "settled"
                    # and truncated the trace before the loop had corrected.
                    $nearTarget = [math]::Abs($rel - $step) -le 0.09
                    if ($nearTarget -and $null -ne $prev -and
                        [math]::Abs($rel - $prev) -lt $TolDeg) {
                        if ($since -lt 0) { $since = $sw.ElapsedMilliseconds }
                        elseif (($sw.ElapsedMilliseconds - $since) -ge $SettleHoldMs) { break }
                    } else {
                        $since = -1
                    }
                    $prev = $rel
                }
            }

            if ($trace.Count -lt 3) {
                Write-Output ("  M{0}  {1,5}  {2,3}  -- too few samples" -f $mv, $step, $rep)
                continue
            }

            # ---- metrics off the trace -----------------------------------
            $startMs = -1
            foreach ($p in $trace) {
                if ([math]::Abs($p.a) -ge 0.088) { $startMs = $p.t; break }
            }

            $t90 = -1
            foreach ($p in $trace) {
                if ([math]::Abs($p.a) -ge 0.9 * $step) { $t90 = $p.t; break }
            }

            $peak = -1e9
            foreach ($p in $trace) { if ($p.a -gt $peak) { $peak = $p.a } }

            # Settle = the earliest sample after which the reading stays inside
            # +/- one encoder count of target for the REST of the trace. Found by
            # locating the LAST sample outside the band and taking the next one,
            # which is O(n) instead of scanning every candidate start.
            $lastBad = -1
            for ($j = 0; $j -lt $trace.Count; $j++) {
                if ([math]::Abs($trace[$j].a - $step) -gt 0.09) { $lastBad = $j }
            }
            $settleMs = -1
            if (($lastBad + 1) -lt $trace.Count) { $settleMs = $trace[$lastBad + 1].t }

            $final = $trace[$trace.Count - 1].a
            $err   = $final - $step

            # {6,+5:F2} is NOT a valid .NET format string -- '+' cannot be
            # combined with the standard specifier F2, and it throws "Input
            # string was not in a correct format" at runtime. A three-section
            # custom format is the way to force an explicit sign.
            Write-Output ("  M{0}  {1,5}  {2,3}  {3,6}  {4,4}  {5,5:F2}  {6,6:+0.00;-0.00;+0.00}  {7,7}  {8,6:F2}  {9,6:+0.00;-0.00;+0.00}  {10,3}" -f `
                $mv, $step, $rep,
                $(if ($startMs -ge 0) { $startMs } else { "-" }),
                $(if ($t90 -ge 0) { $t90 } else { "-" }),
                $peak, ($peak - $step),
                $(if ($settleMs -ge 0) { $settleMs } else { "never" }),
                $final, $err, $trace.Count)
        }
    }
}

$csvPath = Join-Path $PSScriptRoot "posstep.csv"
[System.IO.File]::WriteAllText($csvPath, $csv.ToString())

Send-Cmd "STOPALL"
Start-Sleep -Milliseconds 400
[void](Read-Lines 300)
$port.Close()

Write-Output ""
Write-Output "times in ms. 'start' = first sample past 1 encoder count (0.088 deg);"
Write-Output "'t90' = first sample past 90% of target; 'over' = peak - target;"
Write-Output "'settle' = first sample after which it stays within 1 count of target."
Write-Output "full traces -> $csvPath"
