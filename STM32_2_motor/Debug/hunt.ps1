# Hunt for the intermittent large-move runaway.
#   .\hunt.ps1 -Motor 2 -Step 170 -Iterations 30
# Repeatedly: return to origin, fire one big POS step, and log the whole transient.
# Flags any iteration whose relative angle overshoots the target by more than
# RunawayMarginDeg, and dumps its trace to runaway_<motor>_<step>_<iter>.csv.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [int]    $Motor            = 2,
    [double] $Step             = 170,
    [int]    $Iterations       = 20,
    [int]    $SampleMs         = 4000,
    [double] $RunawayMarginDeg = 15.0,
    [string] $PortName         = "COM4"
)

function Wrap180([double]$a) {
    while ($a -gt  180.0) { $a -= 360.0 }
    while ($a -lt -180.0) { $a += 360.0 }
    return $a
}

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 60
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true
try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 60
$port.DiscardInBuffer()

$m = "M$Motor"

function Send-Cmd([string]$c) { $port.Write($c + "`r`n") }

# Read for WindowMs and return the LAST parsed ANGLE/VEL line, plus every line seen.
function Sample([int]$WindowMs) {
    $sw  = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 1024
    $last = $null
    while ($sw.ElapsedMilliseconds -lt $WindowMs) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) {
                [void]$buf.Append($tmp, 0, $n)
                $txt = $buf.ToString()
                foreach ($ln in ($txt -split "`n")) {
                    if ($ln -match "STATE=(\w+) ANGLE=(-?[\d.]+).*VEL=(-?[\d.]+).*TARGET=(-?[\d.]+).*ERR=(\d+)") {
                        $last = @{ State = $matches[1]; Angle = [double]$matches[2]
                                   Vel   = [double]$matches[3]; Target = [double]$matches[4]
                                   Err   = [int]$matches[5] }
                    }
                }
                [void]$buf.Clear()
            }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return $last
}

# The firmware only auto-streams while state==RUNNING, so STATUS must be asked for.
function Sample-Cmd() {
    Send-Cmd "$m STATUS"
    return Sample 55
}

$runaways = 0
Write-Output ("hunting: M{0} step={1} iters={2}" -f $Motor, $Step, $Iterations)
Write-Output "  iter  maxRel   finalVel  t_last_move  verdict"
Write-Output "  ----  ------   --------  -----------  -------"

for ($i = 1; $i -le $Iterations; $i++) {

    # --- return to the origin ------------------------------------------------
    # "At the origin" means ANGLE == TARGET, NOT ANGLE == 0. STATUS prints ANGLE
    # as the RAW mechanical angle (motor_control.c: mechanical_angle =
    # encoder_raw/4096*2pi, wrapped to [0,2pi)) -- position_offset is never
    # subtracted from it. ANGLE reads ~0 only when the encoder itself happens to
    # sit at raw 0. TARGET is position_offset in degrees, so the arrival test is
    # |wrap180(ANGLE - TARGET)| < tol.
    # Two earlier versions of this test were both wrong: "|VEL| < 0.2" passes
    # while the rotor is still short of the origin, and "|ANGLE| < 0.5" is not a
    # test of arrival at all -- it reported 12/12 "never reached origin" on a
    # motor that was sitting exactly on target.
    Send-Cmd "$m POS 0"
    $base = $null
    $errAtOrigin = 0
    $t0 = (Get-Date)
    while (((Get-Date) -lt $t0.AddSeconds(20))) {
        $s = Sample-Cmd
        if ($s) {
            $errAtOrigin = $s.Err
            if ([math]::Abs((Wrap180 ($s.Angle - $s.Target))) -lt 0.5 -and [math]::Abs($s.Vel) -lt 0.5) {
                $base = $s.Angle; break
            }
        }
        Start-Sleep -Milliseconds 80
    }
    if (-not $base) {
        Write-Output ("  {0,4}  -- never reached origin (ERR={1})" -f $i, $errAtOrigin)
        continue
    }
    Start-Sleep -Milliseconds 150

    # --- fire the step and log the whole transient ---
    $trace = New-Object System.Collections.ArrayList
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    Send-Cmd ("$m POS {0:F4}" -f $Step)
    $maxRel = 0.0
    $maxRelT = 0
    $lastMoveT = 0
    $finalVel = 0.0
    $maxErr = 0
    while ($sw.ElapsedMilliseconds -lt $SampleMs) {
        $s = Sample-Cmd
        if ($s) {
            $rel = Wrap180($s.Angle - $base)
            [void]$trace.Add(@{ t = $sw.ElapsedMilliseconds; rel = $rel; vel = $s.Vel })
            if ([math]::Abs($rel) -gt $maxRel) { $maxRel = [math]::Abs($rel); $maxRelT = $sw.ElapsedMilliseconds }
            if ([math]::Abs($s.Vel) -gt 0.5) { $lastMoveT = $sw.ElapsedMilliseconds }
            $finalVel = $s.Vel
            if ($s.Err -gt $maxErr) { $maxErr = $s.Err }
        }
    }

    $verdict = "ok"
    if ($maxRel -gt ($Step + $RunawayMarginDeg)) {
        $verdict = "RUNAWAY"
        $runaways++
        $csv = "runaway_{0}_{1}_{2}.csv" -f $Motor, $Step, $i
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine("t_ms,rel_deg,vel")
        foreach ($p in $trace) { [void]$sb.AppendLine(("{0},{1:F3},{2:F3}" -f $p.t, $p.rel, $p.vel)) }
        Set-Content -Path $csv -Value $sb.ToString() -Encoding ASCII
        $verdict = "RUNAWAY -> $csv"
    } elseif ([math]::Abs($finalVel) -gt 1.0) {
        $verdict = "still moving"
    }

    $errTxt = ""
    if ($maxErr -ne 0) { $errTxt = "  ERR=$maxErr" }
    Write-Output ("  {0,4}  {1,6:F1}   {2,8:F2}  {3,8}     {4}{5}" -f `
        $i, $maxRel, $finalVel, $lastMoveT, $verdict, $errTxt)
}

Write-Output ("done: {0}/{1} runaways" -f $runaways, $Iterations)
$port.Close()
