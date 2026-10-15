# Exercise ZERO and POS and check what the telemetry then means.
#   .\zeropos.ps1 -Motor 1
#   .\zeropos.ps1 -Motor 2 -Targets 0,10,90,-45,180,359,450
#
# What the firmware actually does (motor_control.c, uart_debug.c):
#   mechanical_angle = encoder_raw/4096*2*pi, normalised to [0,2pi) -- the RAW
#     mechanical angle. position_offset is NOT subtracted from it.
#   STATUS ANGLE   = mechanical_angle in degrees  -> absolute, 0..360, unrelated
#                    to where the zero is.
#   STATUS TARGET  = position_offset + commanded_degrees (POSITION mode) -> the
#                    absolute mechanical angle the loop is aiming at.
#   ZERO  -> position_offset = mechanical_angle, target_angle = mechanical_angle.
#            So ZERO does NOT make ANGLE read 0; it makes TARGET read equal to
#            ANGLE. That is the thing to watch.
#   POS d -> target_angle = position_offset + d*DEG_TO_RAD, i.e. d degrees
#            RELATIVE to the ZERO point, taken the short way round.
#
# The script sends ZERO first and only then POS 0, so entering position mode can
# never fire the rotor at a stale target.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [int]    $Motor        = 1,
    [string] $PortName     = "COM4",
    [int[]]  $Targets      = @(0, 10, 90, -45, 180, 359, 450),
    [double] $SettleTol    = 0.10,
    [int]    $SettleHoldMs = 400,
    [int]    $WaitMs       = 10000
)

function Wrap180([double]$a) {
    while ($a -gt  180.0) { $a -= 360.0 }
    while ($a -le -180.0) { $a += 360.0 }
    return $a
}
function Wrap360([double]$a) {
    while ($a -ge 360.0) { $a -= 360.0 }
    while ($a -lt   0.0) { $a += 360.0 }
    return $a
}

$m = "M$Motor"

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 30
$port.WriteTimeout = 1000
$port.DtrEnable    = $false
$port.RtsEnable    = $false
try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 80
$port.DiscardInBuffer()

$script:tail   = ""
$script:last   = $null
$script:boots  = 0

function Send-Cmd([string]$c) { $port.Write($c + "`r`n") }

# Read for windowMs, keeping a leftover tail so a line split across two reads is
# not lost, and return the last complete STATUS line seen.
function Poll([int]$windowMs) {
    $sw  = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object char[] 512
    while ($sw.ElapsedMilliseconds -lt $windowMs) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) {
                $script:tail += (-join $tmp[0..($n - 1)])
                $cut = $script:tail.LastIndexOf("`n")
                if ($cut -ge 0) {
                    $whole = $script:tail.Substring(0, $cut)
                    $script:tail = $script:tail.Substring($cut + 1)
                    if ($whole -match 'STM32_2_motor ready') { $script:boots++ }
                    foreach ($ln in ($whole -split "`n")) {
                        if ($ln -match 'STATE=(\w+)\s+ANGLE=(-?[\d.]+)\s+EANGLE=(-?[\d.]+)\s+VEL=(-?[\d.]+)\s+TARGET=(-?[\d.]+)') {
                            $script:last = @{
                                State  = $matches[1]
                                Angle  = [double]$matches[2]
                                EAngle = [double]$matches[3]
                                Vel    = [double]$matches[4]
                                Target = [double]$matches[5]
                            }
                        }
                    }
                } elseif ($script:tail.Length -gt 4096) {
                    $script:tail = $script:tail.Substring($script:tail.Length - 4096)
                }
            }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return $script:last
}

# Ask for STATUS and wait until the angle stops changing for SettleHoldMs of WALL
# TIME (never a sample count -- the poll rate is a tunable).
function Wait-Stable([int]$timeoutMs) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $stableSince = -1
    $prev = $null
    while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
        Send-Cmd "$m STATUS"
        $s = Poll 50
        if ($s) {
            if ($prev -and [math]::Abs($s.Angle - $prev.Angle) -lt $SettleTol) {
                if ($stableSince -lt 0) { $stableSince = $sw.ElapsedMilliseconds }
                if (($sw.ElapsedMilliseconds - $stableSince) -ge $SettleHoldMs) { return $s }
            } else {
                $stableSince = -1
            }
            $prev = $s
        }
    }
    return $prev
}

# --- make sure we start from a known, still state -------------------------
Send-Cmd "$m STOP"
Start-Sleep -Milliseconds 400
Send-Cmd "$m STATUS"
$s = Poll 300
if ($s) {
    Write-Output ("start: STATE={0} ANGLE={1:N2} TARGET={2:N2} EANGLE={3:N2} ERR?" -f `
        $s.State, $s.Angle, $s.Target, $s.EAngle)
}

# --- ZERO, then enter position mode ---------------------------------------
# Sample EANGLE with the rotor STATIONARY, immediately BEFORE ZERO. This is the
# only moment the invariance is testable: ZERO must not move the commutation
# reference, but EANGLE legitimately changes every time the rotor turns, so a
# reading taken after the moves below says nothing about ZERO.
Send-Cmd "$m STATUS"
$s = Poll 300
$eangle_before = $s.EAngle
$angle_before  = $s.Angle

Send-Cmd "$m ZERO"
Start-Sleep -Milliseconds 150
Send-Cmd "$m STATUS"
$s = Poll 300
$zeroDeg = $s.Angle
Write-Output ""
Write-Output "around ZERO (rotor stationary throughout):"
Write-Output ("  ANGLE  {0:N2} -> {1:N2}" -f $angle_before, $s.Angle)
Write-Output ("  EANGLE {0:N2} -> {1:N2}   (must be identical -- ZERO only writes position_offset)" -f `
    $eangle_before, $s.EAngle)
Write-Output "  TARGET still prints the VELOCITY here, because STATUS only shows target_angle"
Write-Output "  in POSITION mode -- it becomes the angle on the next line."

Send-Cmd "$m POS 0"
$s = Wait-Stable 8000
Write-Output ("  POS 0 -> ANGLE = {0:N2}  TARGET = {1:N2}  ({2})   <- equal, so ZERO left it sitting still" -f `
    $s.Angle, $s.Target, $s.State)
Write-Output ("  zero point sits at mechanical angle {0:N2} deg" -f $zeroDeg)
Write-Output ""

# --- step through the targets ---------------------------------------------
Write-Output ("  cmd     TARGET      ANGLE     err=wrap180(ANGLE-TARGET)   moved=wrap180(ANGLE-{0:N2})" -f $zeroDeg)
Write-Output "  -----   --------   --------   ------------------------   ----------------------"

foreach ($t in $Targets) {
    Send-Cmd ("$m POS {0}" -f $t)
    $s = Wait-Stable $WaitMs
    if (-not $s) { Write-Output ("  {0,5}   (no telemetry)" -f $t); continue }
    $err   = Wrap180 ($s.Angle - $s.Target)
    $moved = Wrap180 ($s.Angle - $zeroDeg)
    $want  = Wrap180 $t
    $flag  = ""
    if ([math]::Abs($err) -gt 0.15) { $flag = "  <-- did NOT reach target" }
    if ([math]::Abs((Wrap180 ($moved - $want))) -gt 0.15) { $flag += "  <-- moved the wrong amount" }
    Write-Output ("  {0,5}   {1,8:F2}   {2,8:F2}   {3,22:F2}   {4,22:F2}{5}" -f `
        $t, $s.Target, $s.Angle, $err, $moved, $flag)
}

# --- back to the zero, and confirm EANGLE was never touched ---------------
Send-Cmd "$m POS 0"
$s = Wait-Stable $WaitMs
Write-Output ""
Write-Output ("back to POS 0: ANGLE = {0:N2} (zero was {1:N2}, delta {2:N2})" -f `
    $s.Angle, $zeroDeg, (Wrap180 ($s.Angle - $zeroDeg)))
Write-Output ("EANGLE now = {0:N2} (was {1:N2} at the ZERO point; it differs only because the rotor moved)" -f `
    $s.EAngle, $eangle_before)

Send-Cmd "$m STOP"
Start-Sleep -Milliseconds 200
$port.Close()

Write-Output ""
if ($script:boots -gt 0) {
    Write-Output ("*** the board RESET {0} time(s) during this run -- some readings above may be junk ***" -f $script:boots)
} else {
    Write-Output "no board reset during this run."
}
