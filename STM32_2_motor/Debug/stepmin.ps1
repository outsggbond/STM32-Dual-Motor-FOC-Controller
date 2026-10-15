param(
    [string] $Motors        = "1,2",
    [string] $Steps         = "0.03,0.05,0.09,0.5,1,2,5",
    [int]    $PollReadMs    = 60,
    [int]    $PollSleepMs   = 10,
    [int]    $SettleHoldMs  = 1000,
    [int]    $StepTimeoutMs = 20000,
    [int]    $HoldTimeoutMs = 15000,
    [string] $PortName      = "COM4"
)

# Minimum-step / startup-latency yardstick. Companion to posstat.ps1.
#
# posstat.ps1 answers "does it land on the target" for LARGE steps. This one
# answers a different question: for a SMALL step, does the rotor move at all,
# and how long does it take to start?
#
# Why they differ. The position loop is pure P (MOTORn_POS_KP=20, POS_KI=0,
# POS_KD=0, motor_control.c Motor_PositionControl). It emits
#     target_velocity = 20 * error_rad   (clamped to +/-MAX_VELOCITY=20 rad/s)
# so a 1-encoder-count error (0.0879 deg = 0.001534 rad) asks for a mere
# 0.031 rad/s. The inner velocity loop must then integrate its way up to the
# ~1.5 V stiction breakaway before anything turns, and it climbs at only
# ki*error = 0.5*0.031 = 0.0155 V/s -- on the order of 100 s. Big steps
# saturate the position P term and look perfect, which is why the earlier
# 0/45/90/... sweep saw 0.00 deg everywhere and hid this.
#
# Method: for each step size, return to POS 0 and wait until ANGLE is stable,
# take that as the baseline, then command the step and poll ANGLE until it
# moves (>= half an encoder count) and re-settles. Reports the startup latency
# separately from the final error, because those are the two different limits.
#
# "Re-settles" must be measured in WALL TIME (SettleHoldMs), never in a count
# of agreeing samples -- see the comment at the settle test. Polling faster for
# better latency resolution silently corrupted the error column when it was
# sample-counted.
#
# MEASURED RESULT (2026-10-02, both motors, kp=0.05/ki=0.5, POS_KP=20/POS_KI=0):
# every step down to 0.09 deg = ONE encoder count moved, in well under a
# second, with error <= 0.06 deg. The prediction above (that a 1-count step
# would need ~100 s of integral wind-up) was WRONG by two orders of magnitude:
# moving one count only requires overcoming the local restoring stiffness over
# a tiny displacement, not the full stiction breakaway that governs low-speed
# CONTINUOUS motion. So the pure-P position loop is fine and POS_KI stays 0.
# Do not add an integral term here on the strength of the stick-slip finding.
#
# The min/max columns exist for the sub-count case: below 0.0879 deg the
# reading cannot land on the target, so the interesting question is whether it
# sits still (span 0) or dithers between two counts (span 1.0).
#
# ASCII only on purpose: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK,
# so UTF-8 comments corrupt the parse.

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true

try { $port.Open() }
catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

function Send-Cmd([string]$c) { $port.Write($c + "`r`n") }

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

# \b matters: the line also carries EANGLE=, which would match a bare "ANGLE=".
function Get-Angle([string[]]$lines) {
    for ($i = $lines.Count - 1; $i -ge 0; $i--) {
        if ($lines[$i] -match "\bANGLE=([-0-9.]+)") { return [double]$Matches[1] }
    }
    return [double]::NaN
}

# One fresh ANGLE sample: ask explicitly, take the last line that arrived.
# readMs is the resolution knob: the poll period is PollSleepMs + readMs, and
# the measured startup latency can never be resolved finer than that. The
# first run used 420 ms (0.62 s polls) and every step reported 0.65-0.73 s --
# i.e. "it moved on the first poll" for all of them, which measures the poll,
# not the motor. 60 ms here brings the floor down to ~0.07 s.
function Sample-Angle([string]$m, [int]$readMs = 420) {
    Send-Cmd "$m STATUS"
    return Get-Angle (Read-Lines $readMs)
}

function Wrap180([double]$d) {
    while ($d -gt  180.0) { $d -= 360.0 }
    while ($d -lt -180.0) { $d += 360.0 }
    return $d
}

# Poll ANGLE until two consecutive samples agree within 0.05 deg.
# Only ANGLE is usable here: VEL and EANGLE are FROZEN in READY (see
# posstat.ps1's note), and a stalled rotor sits in READY.
function Wait-Stable([string]$m, [int]$timeoutMs) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $prev = Sample-Angle $m
    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        Start-Sleep -Milliseconds 250
        $cur = Sample-Angle $m
        if (-not [double]::IsNaN($prev) -and -not [double]::IsNaN($cur) -and
            [math]::Abs($cur - $prev) -lt 0.05) { return $cur }
        $prev = $cur
    }
    return [double]::NaN
}

# Latch a known origin for one motor. Same proven sequence as posstat.ps1:
# STOP -> wait rest (on ANGLE) -> POS 0 to force POSITION mode -> wait hold
# -> ZERO with bounded retries. ZERO does NOT change control_mode, so the
# POS 0 is load-bearing, not decorative.
function Set-Origin([string]$m) {
    Send-Cmd "$m STOP"
    if ([double]::IsNaN((Wait-Stable $m 6000))) { return [double]::NaN }
    Send-Cmd "$m POS 0"
    if ([double]::IsNaN((Wait-Stable $m $HoldTimeoutMs))) { return [double]::NaN }
    for ($t = 1; $t -le 8; $t++) {
        Send-Cmd "$m ZERO"
        Start-Sleep -Milliseconds 600
        $a1 = Sample-Angle $m
        Start-Sleep -Milliseconds 300
        $a2 = Sample-Angle $m
        if (-not [double]::IsNaN($a1) -and -not [double]::IsNaN($a2) -and
            [math]::Abs($a1 - $a2) -lt 0.05) { return $a2 }
    }
    return [double]::NaN
}

$MOVE_EPS = 0.04    # half an encoder count (0.0879 deg) = "the reading changed"

foreach ($motor in ($Motors -split ',')) {
    $mv = [int]$motor.Trim()
    $m  = "M$mv"

    Write-Output ""
    Write-Output "================ $m ================"

    $origin = Set-Origin $m
    if ([double]::IsNaN($origin)) {
        Write-Output "$m : NO TELEMETRY - could not latch an origin, skipping."
        continue
    }
    Write-Output ("origin = {0:F2} deg" -f $origin)
    Write-Output " step    moved   start(s)   settled    error    min/max(rel)  span(cts)"
    Write-Output " -----   -----   --------   --------   -------   -----------   --------"

    foreach ($s in ($Steps -split ',')) {
        $sv = [double]$s.Trim()

        # Return to zero and let it settle; every step is then measured from
        # rest at a known place, so "did it move" is unambiguous.
        Send-Cmd ("$m POS 0")
        $base = Wait-Stable $m $HoldTimeoutMs
        if ([double]::IsNaN($base)) {
            Write-Output ("{0,6:F2}   NO DATA (never settled at 0)" -f $sv)
            continue
        }

        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        Send-Cmd ("$m POS {0}" -f $sv)

        $moved   = $false
        $startS  = [double]::NaN
        $last    = $base
        $stableSince = -1
        $minA    = [double]::PositiveInfinity
        $maxA    = [double]::NegativeInfinity

        while ($sw.ElapsedMilliseconds -lt $StepTimeoutMs) {
            if ($PollSleepMs -gt 0) { Start-Sleep -Milliseconds $PollSleepMs }
            $a = Sample-Angle $m $PollReadMs
            if ([double]::IsNaN($a)) { continue }

            # Track the excursion relative to the baseline. For a step smaller
            # than one encoder count the reading cannot land on the target, so
            # what happens (nothing, or a dither between two counts) shows up
            # here as the min/max spread rather than in the settled value.
            $rel = Wrap180 ($a - $base)
            if ($rel -lt $minA) { $minA = $rel }
            if ($rel -gt $maxA) { $maxA = $rel }

            if (-not $moved -and [math]::Abs($rel) -ge $MOVE_EPS) {
                $moved  = $true
                $startS = $sw.Elapsed.TotalSeconds
            }
            # Settled = the reading has agreed for SettleHoldMs of WALL TIME,
            # not for N samples. Counting samples made this condition depend on
            # the poll rate: at 70 ms polls "2 consecutive" is only 0.14 s of
            # agreement, which the rotor satisfies while still decelerating, so
            # the run stopped mid-transient and reported 1-2 counts of overshoot
            # at 0.5/1/2 deg that the slower 620 ms run did not see.
            if ($moved -and [math]::Abs((Wrap180 ($a - $last))) -lt 0.05) {
                if ($stableSince -lt 0) { $stableSince = $sw.ElapsedMilliseconds }
                elseif (($sw.ElapsedMilliseconds - $stableSince) -ge $SettleHoldMs) { break }
            } else {
                $stableSince = -1
            }
            $last = $a
        }

        $end     = Sample-Angle $m
        $elapsed = $sw.Elapsed.TotalSeconds

        # Double.IsFinite does NOT exist on .NET Framework (which is what
        # Windows PowerShell 5.1 runs on) -- use IsInfinity/IsNaN.
        if ([double]::IsInfinity($minA) -or [double]::IsNaN($minA)) { $minA = [double]::NaN }
        if ([double]::IsInfinity($maxA) -or [double]::IsNaN($maxA)) { $maxA = [double]::NaN }
        # Span in encoder counts: 1.00 means it moved exactly one quantum.
        $span = if (-not [double]::IsNaN($minA) -and -not [double]::IsNaN($maxA)) {
                    ($maxA - $minA) / 0.0879
                } else { [double]::NaN }

        $movedTxt = if ($moved) { "yes" } else { "NO" }
        $startTxt = if ($moved) { "{0,8:F2}" -f $startS } else { "       -" }
        if ([double]::IsNaN($end)) {
            Write-Output ("{0,6:F2}   {1,-5}   {2}   NO DATA" -f $sv, $movedTxt, $startTxt)
        } else {
            $err = Wrap180 ($end - $origin - $sv)
            $note = ""
            if (-not $moved) { $note = "  <- never moved in $([int]$elapsed)s" }
            elseif ($sw.ElapsedMilliseconds -ge $StepTimeoutMs) { $note = "  <- never settled (dithering?)" }
            Write-Output ("{0,6:F2}   {1,-5}   {2}   {3,8:F2}   {4,7:F2}   {5,7:F2}/{6,-7:F2} {7,5:F1}{8}" -f `
                $sv, $movedTxt, $startTxt, $end, $err, $minA, $maxA, $span, $note)
        }
    }

    Send-Cmd "$m STOP"
    Start-Sleep -Milliseconds 300
}

Send-Cmd "STOPALL"
Start-Sleep -Milliseconds 300
$port.Close()
Write-Output ""
Write-Output "done."
