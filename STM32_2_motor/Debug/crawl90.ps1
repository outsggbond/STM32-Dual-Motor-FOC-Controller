param(
    [double] $StepDeg      = 0.703125,          # 8 AS5600 counts; 90/128 exactly
    [double] $SpanDeg      = 90.0,
    [int]    $PollMs       = 60,
    [int]    $SettleHoldMs = 150,
    [int]    $StepTimeoutMs = 4000,
    [int]    $ReportEvery  = 16,
    [string] $Motors       = "1,2",
    [string] $PortName     = "COM4"
)

# Slow stepped crawl to 90 degrees, both motors together.
#
# Default StepDeg = 0.703125 deg = exactly 8 AS5600 counts, and 90/0.703125 =
# 128, so the crawl lands on 90.0000 deg with no rounding remainder.
#
# Pacing is NOT a timer. Each step waits until every motor has actually
# stopped moving (agreement held for SettleHoldMs of WALL TIME), then issues
# the next target. A fixed dwell would either overlap steps -- turning this
# into a continuous turn that merely looks stepped -- or waste time. The
# measured floor for a single encoder count is ~0.15-0.4 s to move and settle,
# which is why the minimum step size cannot go faster than ~3.5 min for 90 deg
# no matter how the loop is written; a bigger step is the only real lever.
#
# Absolute targets, not incremental. Sending POS k*StepDeg each time means a
# step that fails to register is simply re-requested by the next one -- error
# cannot accumulate over 128 steps. Incremental commands would drift.
#
# ASCII only on purpose: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK.

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

# Latest parsed STATUS per motor. Tolerant of the truncated lines that dual
# telemetry produces: anything that does not parse is skipped, and ERR stays
# -1 ("not seen") rather than being read as a healthy 0.
#
# ★ Capture every group into a local BEFORE any further -match. $Matches is a
# single automatic variable that the next -match overwrites, and the ERR
# pattern below has only one group -- so reading $Matches[2]/$Matches[3] after
# it yields $null, and [double]$null is 0.0. That silently reported ANGLE=0.00
# for every motor on every sample (the whole first crawl looked frozen while
# the motors were almost certainly moving normally).
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

function Sample-Angle([string]$m, [int]$readMs = 420) {
    Send-Cmd "$m STATUS"
    return (Get-AngleFrom (Read-Lines $readMs) $m)
}

function Get-AngleFrom([string[]]$lines, [string]$m) {
    $st = Parse-Statuses $lines
    $mv = [int]$m.Substring(1)
    if ($st.ContainsKey($mv)) { return $st[$mv].Angle }
    return [double]::NaN
}

# Wait until ANGLE agrees for SettleHoldMs of WALL TIME -- never a count of
# samples. See stepmin.ps1: a sample-counted settle criterion silently depends
# on the poll rate and reported fake overshoot when the poll was sped up.
function Wait-Stable([string]$m, [int]$timeoutMs, [int]$holdMs = 800) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $prev = Sample-Angle $m
    $since = -1
    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        Start-Sleep -Milliseconds 200
        $cur = Sample-Angle $m
        if (-not [double]::IsNaN($prev) -and -not [double]::IsNaN($cur) -and
            [math]::Abs($cur - $prev) -lt 0.05) {
            if ($since -lt 0) { $since = $sw.ElapsedMilliseconds }
            elseif (($sw.ElapsedMilliseconds - $since) -ge $holdMs) { return $cur }
        } else {
            $since = -1
        }
        $prev = $cur
    }
    return [double]::NaN
}

# ---- latch a known origin on every motor -------------------------------
$mvList = @()
foreach ($motor in ($Motors -split ',')) { $mvList += [int]$motor.Trim() }

Send-Cmd "STOPALL"
Start-Sleep -Milliseconds 500
[void](Read-Lines 300)

$origin = @{}
foreach ($mv in $mvList) {
    $m = "M$mv"
    Send-Cmd "$m STOP"
    if ([double]::IsNaN((Wait-Stable $m 8000))) {
        Write-Output "$m : rotor will not settle, cannot latch ZERO."; $port.Close(); exit 1
    }
    Send-Cmd "$m POS 0"
    if ([double]::IsNaN((Wait-Stable $m 15000))) {
        Write-Output "$m : will not hold under POS 0, cannot latch ZERO."; $port.Close(); exit 1
    }
    $ok = $false
    for ($t = 1; $t -le 8; $t++) {
        Send-Cmd "$m ZERO"
        Start-Sleep -Milliseconds 600
        $a1 = Sample-Angle $m
        Start-Sleep -Milliseconds 300
        $a2 = Sample-Angle $m
        if (-not [double]::IsNaN($a1) -and -not [double]::IsNaN($a2) -and
            [math]::Abs($a1 - $a2) -lt 0.05) { $origin[$mv] = $a2; $ok = $true; break }
    }
    if (-not $ok) {
        Write-Output "$m : NO TELEMETRY - could not latch an origin."; $port.Close(); exit 1
    }
    Write-Output ("$m origin = {0:F2} deg" -f $origin[$mv])
}

$steps = [int][math]::Round($SpanDeg / $StepDeg)
Write-Output ("crawl: {0} steps x {1:F4} deg = {2:F4} deg, hold {3} ms" -f `
    $steps, $StepDeg, ($steps * $StepDeg), $SettleHoldMs)
Write-Output ""

# ---- crawl --------------------------------------------------------------
$swAll   = [System.Diagnostics.Stopwatch]::StartNew()
$aborted = $false
$last    = @{}

for ($k = 1; $k -le $steps; $k++) {
    $t = $k * $StepDeg
    # Both motors get the same absolute target on the same tick. One command
    # per line: the parser has no both-motors syntax.
    foreach ($mv in $mvList) { Send-Cmd ("M{0} POS {1:F4}" -f $mv, $t) }

    # Step-and-settle: block until every motor has stopped, then move on.
    $swStep = [System.Diagnostics.Stopwatch]::StartNew()
    $prev   = @{}
    $since  = @{}
    foreach ($mv in $mvList) { $since[$mv] = -1 }

    while ($swStep.ElapsedMilliseconds -lt $StepTimeoutMs) {
        # Ask explicitly -- the 500 ms auto-telemetry is far too coarse to
        # detect a 0.15 s settle.
        foreach ($mv in $mvList) { Send-Cmd "M$mv STATUS" }
        $st = Parse-Statuses (Read-Lines $PollMs)

        foreach ($mv in $mvList) {
            if (-not $st.ContainsKey($mv)) { continue }
            # A fault (stall / thermal / encoder) must stop this, not be
            # hammered through for another hundred steps.
            if ($st[$mv].Err -gt 0) {
                Write-Output ("ABORT at step {0}/{1}: M{2} ERR={3} STATE={4}" -f `
                    $k, $steps, $mv, $st[$mv].Err, $st[$mv].State)
                $aborted = $true
                break
            }
            $a = $st[$mv].Angle
            if ($prev.ContainsKey($mv) -and
                [math]::Abs((Wrap180 ($a - $prev[$mv]))) -lt 0.05) {
                if ($since[$mv] -lt 0) { $since[$mv] = $swStep.ElapsedMilliseconds }
            } else {
                $since[$mv] = -1      # moved again -> restart the hold window
            }
            $prev[$mv] = $a
            $last[$mv] = $st[$mv]
        }
        if ($aborted) { break }

        $all = $true
        foreach ($mv in $mvList) {
            if ($since[$mv] -lt 0 -or
                ($swStep.ElapsedMilliseconds - $since[$mv]) -lt $SettleHoldMs) { $all = $false }
        }
        if ($all) { break }
    }
    if ($aborted) { break }

    if ($k % $ReportEvery -eq 0) {
        $parts = @()
        foreach ($mv in $mvList) {
            if ($last.ContainsKey($mv)) {
                $rel = Wrap180 ($last[$mv].Angle - $origin[$mv])
                $parts += ("M{0} {1,7:F2} deg (err {2})" -f $mv, $rel, $last[$mv].Err)
            } else {
                $parts += ("M{0}   no data" -f $mv)
            }
        }
        Write-Output ("step {0,4}/{1}  t={2,4:F0}s  {3}" -f $k, $steps, $swAll.Elapsed.TotalSeconds, ($parts -join "   "))
    }
}

# ---- settle on the final target ----------------------------------------
Write-Output ""
if (-not $aborted) {
    Write-Output "--- final settle ---"
    foreach ($mv in $mvList) {
        $a = Wait-Stable ("M$mv") 20000 1200
        if ([double]::IsNaN($a)) { Write-Output ("M{0} : NO DATA at end" -f $mv); continue }
        $rel = Wrap180 ($a - $origin[$mv])
        # {3:+.2f} is NOT a valid .NET format string -- '+' cannot be combined
        # with the standard specifier F2, and it throws "Input string was not in
        # a correct format" at runtime. A three-section custom format is the
        # way to force an explicit sign on every value.
        Write-Output ("M{0}  final {1,7:F2} deg   target {2:F2}   error {3:+0.00;-0.00;+0.00} deg" -f `
            $mv, $rel, ($steps * $StepDeg), (Wrap180 ($rel - $steps * $StepDeg)))
    }
}

Send-Cmd "STOPALL"
Start-Sleep -Milliseconds 400
[void](Read-Lines 300)
$port.Close()
Write-Output ""
Write-Output ("done. elapsed {0:F0} s" -f $swAll.Elapsed.TotalSeconds)
