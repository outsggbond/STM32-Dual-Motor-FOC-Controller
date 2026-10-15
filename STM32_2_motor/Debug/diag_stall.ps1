# =====================================================================
# diag_stall.ps1 - two questions the suite cannot answer by itself.
#
# Q1  Can the stall guard be tripped WITHOUT the NaN bug?  suite.ps1 step
#     6.2 used `M1 SPD nan` as its trigger, and 2026-10-03's fix made nan a
#     rejected input, so 6.2 now fails (ERR 0 -> 0, STATE=READY): no fault
#     was ever injected. The guard runs in the OPEN branch too
#     (motor_control.c:1419), and its arming gate is only
#     `|target_voltage| >= MOTOR_STALL_MIN_VOLT` (1.5 V) - so a STATIC
#     open-loop vector should arm it and then sit still for 2 s.
#     This script tests that, plus the NEGATIVE control (0.5 V, below the
#     gate, must NOT trip) which proves the trip is really voltage-gated
#     and not just "open loop always trips".
#
# Q2  Where does step 4.6's ERR=32 come from?  On the last full run 4.6
#     failed on `ERR=32` while its position error was 0.060 deg (well inside
#     the 0.50 bound), and 4.6 inherits whatever 4.5 left behind because it
#     never clears. This replays 4.6's exact sequence from a cleared latch,
#     sampling ERR every 300 ms, so the trip (if any) is timestamped and can
#     be attributed to a phase instead of guessed at.
#
# ASCII only: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK, so Chinese
# literals here would be mangled. Chinese replies are matched by their ASCII
# field names only.
#
#   .\diag_stall.ps1
# =====================================================================
param(
    [string] $PortName  = "COM4",
    [double] $LowVolt   = 0.5,     # below MOTOR_STALL_MIN_VOLT (1.5 V)
    [double] $HighVolt  = 3.0,     # above it
    [int]    $WatchSec  = 4        # each VOL phase is watched this long
)

$ErrorActionPreference = "Stop"

$Port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$Port.ReadTimeout  = 40
$Port.WriteTimeout = 1000
$Port.DtrEnable    = $true
$Port.RtsEnable    = $true
$Port.Open()

function Send-Cmd {
    param([string]$Cmd, [int]$WaitMs = 400)
    $Port.DiscardInBuffer()
    $Port.Write($Cmd + "`r`n")
    $ms = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $WaitMs) {
        try {
            $n = $Port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $ms.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    $b = $ms.ToArray()
    if ($b.Length -ge 3 -and $b[0] -eq 0xEF -and $b[1] -eq 0xBB -and $b[2] -eq 0xBF) {
        $b = $b[3..($b.Length - 1)]
    }
    return [System.Text.Encoding]::UTF8.GetString($b)
}

# Last "NAME=value" anywhere in the reply. STATUS is one line, and a stale
# field is better than no field for this purpose.
function Fld {
    param([string]$Text, [string]$Name)
    if ($Text -match ("{0}=(\S+)" -f [regex]::Escape($Name))) { return $matches[1] }
    return "?"
}

function FldD {
    param([string]$Text, [string]$Name)
    $v = Fld $Text $Name
    if ($v -match "^-?[0-9.]+$") { return [double]$v }
    return -999.0
}

# ------------------------------------------------------------------ Q1
Write-Host ""
Write-Host "===== Q1: static open-loop vector as a stall trigger =====" -ForegroundColor Cyan

function Watch-Vol {
    param([string]$Label, [double]$Volts, [int]$Secs)
    [void](Send-Cmd "M1 STOP" 300)
    Start-Sleep -Milliseconds 500
    [void](Send-Cmd "M1 CLR" 300)
    Start-Sleep -Milliseconds 300
    $pre = Send-Cmd "M1 STATUS" 300
    Write-Host ("  {0}: arm ERR={1}  sending 'M1 VOL {2}'" -f $Label, (Fld $pre "ERR"), $Volts)
    $rep = Send-Cmd ("M1 VOL {0}" -f $Volts) 400
    Write-Host ("         reply: {0}" -f $rep.Trim())
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tripAt = -1
    $rows = @()
    while ($sw.ElapsedMilliseconds -lt ($Secs * 1000)) {
        $r   = Send-Cmd "M1 STATUS" 200
        $err = Fld $r "ERR"
        $st  = Fld $r "STATE"
        $ang = FldD $r "ANGLE"
        $pwm = Fld $r "PWM"
        $dev = 0.0
        if ($pwm -match "([0-9.]+),([0-9.]+),([0-9.]+)") {
            foreach ($g in 1..3) {
                $d = [math]::Abs([double]$matches[$g] - 0.5)
                if ($d -gt $dev) { $dev = $d }
            }
        }
        $rows += [pscustomobject]@{
            t = $sw.ElapsedMilliseconds; st = $st; err = $err
            ang = $ang; dev = $dev }
        if ($err -ne "0" -and $tripAt -lt 0) { $tripAt = $sw.ElapsedMilliseconds }
        Start-Sleep -Milliseconds 120
    }
    foreach ($row in $rows) {
        Write-Host ("         t={0,5} ms  STATE={1,-8} ERR={2,-3} ANGLE={3,8} |PWM-0.5|={4:N3}" -f `
            $row.t, $row.st, $row.err, $row.ang, $row.dev)
    }
    $first = $rows[0]
    $last  = $rows[$rows.Count - 1]
    Write-Host ("         -> ERR {0} -> {1}; first nonzero at {2} ms; ANGLE moved {3:N2} deg; |PWM-0.5| {4:N3} -> {5:N3}" -f `
        $first.err, $last.err, $(if ($tripAt -lt 0) { "never" } else { "$tripAt" }),
        ([math]::Abs($last.ang - $first.ang)), $first.dev, $last.dev) -ForegroundColor Yellow
    [void](Send-Cmd "M1 STOP" 300)
    Start-Sleep -Milliseconds 500
    return $rows
}

$rowsLow  = Watch-Vol "LOW control"  $LowVolt  $WatchSec
$rowsHigh = Watch-Vol "HIGH trigger" $HighVolt $WatchSec

$lowTripped  = @($rowsLow  | Where-Object { $_.err -ne "0" }).Count
$highTripped = @($rowsHigh | Where-Object { $_.err -ne "0" }).Count
Write-Host ""
Write-Host ("  Q1 verdict: {0} V tripped {1} of {2} samples; {3} V tripped {4} of {5}" -f `
    $LowVolt, $lowTripped, $rowsLow.Count, $HighVolt, $highTripped, $rowsHigh.Count) -ForegroundColor Yellow
if ($lowTripped -eq 0 -and $highTripped -gt 0) {
    Write-Host "  Q1 => the guard IS voltage-gated and a static vector IS a usable trigger for 6.2" -ForegroundColor Green
} elseif ($lowTripped -gt 0) {
    Write-Host "  Q1 => the 1.5 V gate does NOT hold; open loop trips at any voltage" -ForegroundColor Red
} else {
    Write-Host "  Q1 => neither voltage tripped it; 6.2 needs a different trigger" -ForegroundColor Red
}

# ------------------------------------------------------------------ Q2
Write-Host ""
Write-Host "===== Q2: replay 4.6's retarget sequence from a cleared latch =====" -ForegroundColor Cyan

[void](Send-Cmd "STOPALL" 300)
Start-Sleep -Milliseconds 900
[void](Send-Cmd "M1 CLR" 300)
Start-Sleep -Milliseconds 300
$base = Send-Cmd "M1 STATUS" 300
Write-Host ("  baseline after CLR: ERR={0} STATE={1} ANGLE={2}" -f (Fld $base "ERR"), (Fld $base "STATE"), (Fld $base "ANGLE"))

[void](Send-Cmd "M1 ZERO" 300)
Start-Sleep -Milliseconds 500
$z = Send-Cmd "M1 STATUS" 300
Write-Host ("  after ZERO:         ERR={0} STATE={1} ANGLE={2} TARGET={3}" -f (Fld $z "ERR"), (Fld $z "STATE"), (Fld $z "ANGLE"), (Fld $z "TARGET"))

$sw = [System.Diagnostics.Stopwatch]::StartNew()
[void](Send-Cmd "M1 POS 30" 60)
$tA = $sw.ElapsedMilliseconds
Start-Sleep -Milliseconds 250
[void](Send-Cmd "M1 POS 90" 60)
$tB = $sw.ElapsedMilliseconds
Start-Sleep -Milliseconds 250
[void](Send-Cmd "M1 POS 60" 60)
$tC = $sw.ElapsedMilliseconds
Write-Host ("  retargets at t={0} / {1} / {2} ms" -f $tA, $tB, $tC)

$rows = @()
while ($sw.ElapsedMilliseconds -lt 6500) {
    $r   = Send-Cmd "M1 STATUS" 200
    $err = Fld $r "ERR"
    $st  = Fld $r "STATE"
    $ang = FldD $r "ANGLE"
    $tgt = FldD $r "TARGET"
    $vel = FldD $r "VEL"
    $rows += [pscustomobject]@{
        t = $sw.ElapsedMilliseconds; st = $st; err = $err
        ang = $ang; tgt = $tgt; vel = $vel }
    Start-Sleep -Milliseconds 120
}
foreach ($row in $rows) {
    Write-Host ("         t={0,5} ms  STATE={1,-8} ERR={2,-3} ANGLE={3,8} TARGET={4,8} VEL={5,7}" -f `
        $row.t, $row.st, $row.err, $row.ang, $row.tgt, $row.vel)
}
$firstErrRow = @($rows | Where-Object { $_.err -ne "0" } | Select-Object -First 1)
$last = $rows[$rows.Count - 1]
if ($firstErrRow.Count -gt 0) {
    Write-Host ("  Q2 => ERR went nonzero at t={0} ms (STATE={1}, ANGLE={2}, TARGET={3}, VEL={4})" -f `
        $firstErrRow[0].t, $firstErrRow[0].st, $firstErrRow[0].ang, $firstErrRow[0].tgt, $firstErrRow[0].vel) -ForegroundColor Red
} else {
    Write-Host "  Q2 => ERR stayed 0 for the whole sequence: 4.6's ERR=32 was INHERITED, not produced here" -ForegroundColor Green
}
Write-Host ("  final: STATE={0} ERR={1} ANGLE={2} TARGET={3}" -f $last.st, $last.err, $last.ang, $last.tgt)

[void](Send-Cmd "M1 CLR" 300)
[void](Send-Cmd "STOPALL" 300)
$Port.Close()
Write-Host ""
Write-Host "diag_stall done."
