param(
    [int]    $Motor    = 1,
    [string] $Targets  = "0,45,90,135,180,270,315",
    [int]    $SettleMs = 2500,
    [string] $PortName = "COM4"
)

# Position-loop accuracy yardstick, companion to velstat.ps1.
#
# Sends Mx ZERO first to pin a known origin, then steps POS <deg> through the
# target list and reports the settled mechanical angle plus the residual error.
# Measuring against the ZERO origin makes the result independent of whatever
# absolute angle the rotor happened to be resting at.
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

# Collect everything that arrives within $ms as an array of lines.
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

# Two spaced reads of the LIVE field, for settling tests.
#
# ANGLE is the only live field while the driver is off. Do NOT use VEL for this:
# Motor_UpdateVelocity() is called only in MOTOR_STATE_RUNNING (motor_control.c),
# so in READY it keeps the last running value forever -- it read a frozen 26.84
# for 8 s after STOP on a rotor that had already come to rest. EANGLE is frozen
# in READY for the same reason (Motor_UpdateAngle). ANGLE is refreshed by
# Motor_ReadEncoder() in the background task, which is NOT gated on state, so it
# is live: it visibly moved 306.9 -> 48.1 deg in the 450 ms after STOP.
function Read-AnglePair([int]$gapMs) {
    Send-Cmd "$m STATUS"
    $a1 = Get-Angle (Read-Lines 500)
    Start-Sleep -Milliseconds $gapMs
    Send-Cmd "$m STATUS"
    $a2 = Get-Angle (Read-Lines 500)
    return @($a1, $a2)
}

function Wrap180([double]$d) {
    while ($d -gt  180.0) { $d -= 360.0 }
    while ($d -lt -180.0) { $d += 360.0 }
    return $d
}

$m = "M$Motor"

Send-Cmd "$m STOP"

# Wait for the rotor to come to rest before commanding a position.
# STOP only de-energises (PWM 0, STATE READY) and the rotor then coasts --
# but only briefly: measured from 25 rad/s it is stationary within ~0.6 s
# (ANGLE 306.9 at t=190 ms, 48.1 and frozen from t=640 ms). Waiting on VEL
# here is what made this tool report "ROTOR STILL COASTING after 10s" on both
# motors: VEL never decays in READY, so the condition could never be met.
$rest = $false
for ($w = 1; $w -le 12; $w++) {
    $pair = Read-AnglePair 400
    if (-not [double]::IsNaN($pair[0]) -and -not [double]::IsNaN($pair[1]) -and
        [math]::Abs($pair[1] - $pair[0]) -lt 0.05) { $rest = $true; break }
    Start-Sleep -Milliseconds 200
}
if (-not $rest) {
    Write-Output "ROTOR WILL NOT SETTLE (ANGLE still moving after 12 reads) - cannot latch ZERO."
    $port.Close(); exit 1
}

# Force POSITION mode BEFORE latching ZERO. This is the actual root cause of the
# old "NO TELEMETRY" failure, and it is not about coasting at all:
# ZERO sets position_offset and target_angle but does NOT touch control_mode
# (uart_debug.c). Run this tool straight after a speed test and the motor is
# still in MOTOR_CONTROL_VELOCITY with target 25 rad/s, so the START below spun
# it straight back up to 25 rad/s -- the two origin reads could never agree and
# all 8 retries failed. POS 0 switches to POSITION mode and holds still.
Send-Cmd "$m POS 0"

# Wait for that hold to settle, same ANGLE-stability test as above.
$held = $false
for ($w = 1; $w -le 12; $w++) {
    Start-Sleep -Milliseconds 400
    $pair = Read-AnglePair 400
    if (-not [double]::IsNaN($pair[0]) -and -not [double]::IsNaN($pair[1]) -and
        [math]::Abs($pair[1] - $pair[0]) -lt 0.05) { $held = $true; break }
}
if (-not $held) {
    Write-Output "ROTOR WILL NOT SETTLE under POS 0 - cannot latch ZERO."
    $port.Close(); exit 1
}

# Now the rotor is genuinely holding still, so ZERO pins a real origin.
# Retries stay bounded: 8 tries x 700 ms. If it never agrees the rotor is
# genuinely moving (or the telemetry is stale) and we say so rather than
# printing numbers that look fine.
$origin = [double]::NaN
$try    = 0
for ($try = 1; $try -le 8; $try++) {
    Send-Cmd "$m ZERO"
    Start-Sleep -Milliseconds 700
    Send-Cmd "$m STATUS"
    $a1 = Get-Angle (Read-Lines 700)
    Start-Sleep -Milliseconds 400
    Send-Cmd "$m STATUS"
    $a2 = Get-Angle (Read-Lines 700)
    if (-not [double]::IsNaN($a1) -and -not [double]::IsNaN($a2) -and
        [math]::Abs($a1 - $a2) -lt 0.05) {
        $origin = $a2
        break
    }
}

if ([double]::IsNaN($origin)) {
    Write-Output "NO TELEMETRY - rotor will not hold still (check STATE and the COM port)"
    $port.Close(); exit 1
}

Write-Output ("origin = {0:F2} deg  (M{1} ZERO, settled after {2} try/tries)" -f $origin, $Motor, $try)
Write-Output "target   settled    error"

foreach ($t in ($Targets -split ',')) {
    $tv = [double]$t.Trim()
    Send-Cmd ("$m POS {0}" -f $tv)
    Start-Sleep -Milliseconds $SettleMs
    Send-Cmd "$m STATUS"
    $a = Get-Angle (Read-Lines 700)
    if ([double]::IsNaN($a)) { Write-Output ("{0,6:F1}   NO DATA" -f $tv); continue }
    $err = Wrap180 ($a - $origin - $tv)
    Write-Output ("{0,6:F1}   {1,8:F2}   {2,7:F2}" -f $tv, $a, $err)
}

Send-Cmd "$m STOP"
Start-Sleep -Milliseconds 200
$port.Close()
