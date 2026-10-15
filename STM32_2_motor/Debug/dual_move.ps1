# Dual simultaneous position move with arrival timing + accuracy.
#   .\dual_move.ps1 -Target1 30 -Target2 90
# Opens COM4 once, zeroes both motors, fires both POS commands as two separate
# lines (never chained with ';' -- that garbles dual telemetry), then polls.
# ASCII-only source: PS 5.1 on a zh-CN box reads .ps1 as GBK.
param(
    [double]$Target1  = 30,
    [double]$Target2  = 90,
    [int]   $RunMs    = 5000,
    [double]$TolDeg   = 0.09,     # 1 encoder count
    [string]$PortName = "COM4"
)

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 40
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true
$port.Open()
Start-Sleep -Milliseconds 80
$port.DiscardInBuffer()

function Send-Cmd([string]$c) { $port.Write($c + "`r`n") }

# Read whatever arrives within $ms, return the lines.
function Read-Lines([int]$ms) {
    $sw  = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 1024
    while ($sw.ElapsedMilliseconds -lt $ms) {
        try { $n = $port.Read($tmp, 0, $tmp.Length); if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) } }
        catch [System.TimeoutException] { }
    }
    return ($buf.ToString() -split "`n")
}

function Wrap180([double]$d) {
    while ($d -gt  180) { $d -= 360 }
    while ($d -lt -180) { $d += 360 }
    return $d
}

# Parse every "M<n> STATE=..." line found in the buffer.
function Poll-Both($sw, $state) {
    Send-Cmd "M1 STATUS"
    Send-Cmd "M2 STATUS"
    $lines = Read-Lines 60
    foreach ($l in $lines) {
        # NB: anchor "ANGLE=" with a leading space -- a bare ".*ANGLE=" is greedy and
        # matches inside "EANGLE=" (the ELECTRICAL angle), which silently corrupts
        # every error number. Cost me one bogus run.
        if ($l -match "^M(\d) STATE=(\w+) ANGLE=([-0-9.]+) EANGLE=[-0-9.]+ VEL=([-0-9.]+) TARGET=([-0-9.]+) PWM=[0-9.,]+ ERR=(\d+)") {
            $m = [int]$Matches[1]
            if ($m -ne 1 -and $m -ne 2) { continue }
            $ang = [double]$Matches[3]; $vel = [double]$Matches[4]
            $tgt = [double]$Matches[5]; $err = [int]$Matches[6]
            $e   = Wrap180($ang - $tgt)
            $t   = $sw.ElapsedMilliseconds
            if ([math]::Abs($vel) -gt $state[$m].peak) { $state[$m].peak = [math]::Abs($vel) }
            if ($state[$m].arrive -lt 0 -and [math]::Abs($e) -le $TolDeg) { $state[$m].arrive = $t }
            if ($err -ne 0) { $state[$m].err = $err }
            $state[$m].lastE = $e
            $state[$m].lastT = $t
            [void]$state[$m].traj.Add([pscustomobject]@{ t = $t; e = $e; v = $vel })
        }
    }
}

Write-Output "--- STOPALL / ZERO both ---"
Send-Cmd "STOPALL"; Start-Sleep -Milliseconds 400
Send-Cmd "M1 ZERO"; Start-Sleep -Milliseconds 250
Send-Cmd "M2 ZERO"; Start-Sleep -Milliseconds 400
$null = Read-Lines 300

$state = @{
    1 = @{ arrive = -1; peak = 0.0; err = 0; lastE = 0.0; lastT = 0; traj = (New-Object System.Collections.ArrayList) }
    2 = @{ arrive = -1; peak = 0.0; err = 0; lastE = 0.0; lastT = 0; traj = (New-Object System.Collections.ArrayList) }
}

Write-Output ("--- M1 POS {0}  +  M2 POS {1} (two separate lines) ---" -f $Target1, $Target2)
$sw = [System.Diagnostics.Stopwatch]::StartNew()
Send-Cmd ("M1 POS " + $Target1)
Start-Sleep -Milliseconds 60
Send-Cmd ("M2 POS " + $Target2)

while ($sw.ElapsedMilliseconds -lt $RunMs) { Poll-Both $sw $state }

Send-Cmd "STOPALL"
$null = Read-Lines 300
$port.Close()

foreach ($m in @(1,2)) {
    $tgt = if ($m -eq 1) { $Target1 } else { $Target2 }
    $s = $state[$m]
    $arr = if ($s.arrive -lt 0) { "NEVER" } else { "$($s.arrive) ms" }
    "M{0}: target {1,5} deg  arrive={2,-9} peakVEL={3,6} rad/s  final err={4,6} deg  ERR={5}" -f `
        $m, $tgt, $arr, [math]::Round($s.peak,2), [math]::Round($s.lastE,2), $s.err
}

foreach ($m in @(1,2)) {
    Write-Output ("--- M{0} trajectory (first 2.0 s) ---" -f $m)
    Write-Output "   t(ms)   err(deg)   VEL"
    foreach ($p in $state[$m].traj) {
        if ($p.t -gt 2000) { break }
        "  {0,6}   {1,8}   {2,6}" -f $p.t, [math]::Round($p.e,2), [math]::Round($p.v,2)
    }
}
