# Rapid position CHAIN test: POS a -> POS b -> POS c, back-to-back, order preserved.
#   .\pos_chain.ps1 -Seq "30,90,60" -Mode arrive
#   .\pos_chain.ps1 -Seq "30,90,60" -Mode fixed -StepMs 250
# All targets are absolute-from-ZERO (POS semantics), so 30 -> 90 -> 60 means
# +30, +60, -30. Every step is issued the instant the previous one ARRIVES
# (Mode=arrive, tolerance $TolDeg) or on a hard timer (Mode=fixed, which lands
# the next command in the middle of the previous slew -- the harshest case).
# Both motors run the same chain in lockstep. ASCII-only source: PS 5.1 + GBK.
param(
    [string]$Seq           = "30,90,60",
    [ValidateSet("arrive","fixed")][string]$Mode = "arrive",
    [int]   $StepMs        = 300,     # fixed mode: interval between commands
    [double]$TolDeg        = 1.0,     # arrive mode: "reached" threshold
    [int]   $StepTimeoutMs = 2500,    # arrive mode: give up waiting, advance anyway
    [int]   $RunMs         = 12000,
    [string]$PortName      = "COM4"
)

$steps = @($Seq -split ',' | ForEach-Object { [double]$_ })

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

# ---- reset both motors to a fresh common zero ---------------------------------
Write-Output "--- STOPALL / ZERO both ---"
Send-Cmd "STOPALL"; Start-Sleep -Milliseconds 400
Send-Cmd "M1 ZERO"; Start-Sleep -Milliseconds 250
Send-Cmd "M2 ZERO"; Start-Sleep -Milliseconds 400
$null = Read-Lines 300

$state = @{}
foreach ($m in @(1,2)) {
    $state[$m] = @{ tgt = $null; pending = $true; rec = $null; err = 0
                    tgtSeq = (New-Object System.Collections.ArrayList)
                    log    = (New-Object System.Collections.ArrayList) }
}

$sw    = [System.Diagnostics.Stopwatch]::StartNew()
$cur   = -1
$tIssue = 0

function Issue-Step([int]$k) {
    $d = $steps[$k]
    $t = $sw.ElapsedMilliseconds
    Send-Cmd ("M1 POS " + $d)
    Send-Cmd ("M2 POS " + $d)
    $script:tIssue = $t
    foreach ($m in @(1,2)) {
        $rec = [pscustomobject]@{ i = $k; deg = $d; tIssue = $t; tArrive = -1
                                  met = $false; timedOut = $false
                                  peak = 0.0; maxErr = 0.0; lastE = 0.0 }
        $state[$m].rec     = $rec
        $state[$m].pending = $true
    }
}

function Poll-Both {
    Send-Cmd "M1 STATUS"
    Send-Cmd "M2 STATUS"
    $lines = Read-Lines 60
    foreach ($l in $lines) {
        # Leading space before ANGLE is load-bearing: a bare ".*ANGLE=" is greedy
        # and matches inside "EANGLE=" (electrical angle). See dual_move.ps1.
        if ($l -notmatch "^M(\d) STATE=(\w+) ANGLE=([-0-9.]+) EANGLE=[-0-9.]+ VEL=([-0-9.]+) TARGET=([-0-9.]+) PWM=[0-9.,]+ ERR=(\d+)") { continue }
        $m = [int]$Matches[1]
        if ($m -ne 1 -and $m -ne 2) { continue }
        $ang = [double]$Matches[3]; $vel = [double]$Matches[4]
        $tgt = [double]$Matches[5]; $errc = [int]$Matches[6]
        $s = $state[$m]
        $t = $sw.ElapsedMilliseconds

        # A new command has landed when TARGET changes from the previous reading.
        if ($s.pending -and ($null -eq $s.tgt -or [math]::Abs($tgt - $s.tgt) -gt 1e-6)) {
            $s.pending = $false
            [void]$s.tgtSeq.Add([pscustomobject]@{ t = $t; tgt = $tgt; step = $s.rec.i })
        }
        $s.tgt = $tgt

        $e = Wrap180($ang - $tgt)
        if (-not $s.pending -and $null -ne $s.rec) {
            if ([math]::Abs($vel)   -gt $s.rec.peak)   { $s.rec.peak   = [math]::Abs($vel) }
            if ([math]::Abs($e)     -gt $s.rec.maxErr) { $s.rec.maxErr = [math]::Abs($e) }
            $s.rec.lastE = $e
            if (-not $s.rec.met -and [math]::Abs($e) -le $TolDeg) {
                $s.rec.met = $true; $s.rec.tArrive = $t
            }
        }
        if ($errc -ne 0) { $s.err = $errc }
        [void]$s.log.Add([pscustomobject]@{ t = $t; e = $e; v = $vel; tgt = $tgt })
    }
}

Write-Output ("--- chain: {0}   mode={1} ---" -f ($steps -join " -> "), $Mode)
$cur = 0
Issue-Step 0

while ($sw.ElapsedMilliseconds -lt $RunMs) {
    Poll-Both
    if ($cur -lt ($steps.Count - 1)) {
        $t     = $sw.ElapsedMilliseconds
        $ready = $false
        if ($Mode -eq "arrive") {
            $ready = $true
            foreach ($m in @(1,2)) {
                $r = $state[$m].rec
                if ($state[$m].pending -or -not $r.met) { $ready = $false }
            }
            if (-not $ready -and ($t - $tIssue) -gt $StepTimeoutMs) {
                $ready = $true
                foreach ($m in @(1,2)) { if (-not $state[$m].rec.met) { $state[$m].rec.timedOut = $true } }
            }
        } else {
            if (($t - $tIssue) -ge $StepMs) { $ready = $true }
        }
        if ($ready) { $cur++; Issue-Step $cur }
    }
}

$total = $sw.ElapsedMilliseconds
Send-Cmd "STOPALL"
$null = Read-Lines 300
$port.Close()

# ---- report -------------------------------------------------------------------
# Per-step timing is reconstructed from two things: tgtSeq tells us WHEN each
# command actually landed (TARGET changed), the log tells us what the motor did.
# Nothing is taken from the Issue-Step bookkeeping, so the table cannot agree
# with a command the firmware never accepted.
foreach ($m in @(1,2)) {
    Write-Output ("--- M{0} : {1} ---" -f $m, (($steps | ForEach-Object { "$_" }) -join " -> "))
    Write-Output "  step  deg   t_issue  t_arrive  move_ms  peakVEL  maxErr  lastErr"
    Write-Output "  step  deg   t_issue  t_arrive  move_ms  peakVEL  maxErr  lastErr"
    $recs = @{}
    for ($k = 0; $k -lt $steps.Count; $k++) {
        $recs[$k] = [pscustomobject]@{ i=$k; tIssue=-1; tArrive=-1; peak=0.0; maxErr=0.0; lastE=0.0 }
    }
    $ci = -1
    foreach ($q in $state[$m].tgtSeq) { if ($q.step -gt $ci) { $ci = $q.step; $recs[$ci].tIssue = $q.t } }
    $ci = -1
    foreach ($p in $state[$m].log) {
        # attribute each sample to the latest step whose tIssue <= t
        $k = -1
        for ($j = 0; $j -lt $steps.Count; $j++) { if ($recs[$j].tIssue -ge 0 -and $p.t -ge $recs[$j].tIssue) { $k = $j } }
        if ($k -lt 0) { continue }
        if ([math]::Abs($p.v) -gt $recs[$k].peak)   { $recs[$k].peak   = [math]::Abs($p.v) }
        if ([math]::Abs($p.e) -gt $recs[$k].maxErr) { $recs[$k].maxErr = [math]::Abs($p.e) }
        if ($recs[$k].tArrive -lt 0 -and [math]::Abs($p.e) -le $TolDeg) { $recs[$k].tArrive = $p.t }
        $recs[$k].lastE = $p.e
    }
    for ($k = 0; $k -lt $steps.Count; $k++) {
        $r = $recs[$k]
        $mv = if ($r.tIssue -ge 0 -and $r.tArrive -ge 0) { $r.tArrive - $r.tIssue } else { -1 }
        Write-Output ("  {0,4}  {1,5}  {2,8}  {3,8}  {4,7}  {5,7}  {6,6}  {7,7}" -f `
            $k, $steps[$k], $r.tIssue, $r.tArrive, $mv,
            [math]::Round($r.peak,2), [math]::Round($r.maxErr,2), [math]::Round($r.lastE,2))
    }
    Write-Output ("  ERR={0}" -f $state[$m].err)
}

# order check
foreach ($m in @(1,2)) {
    Write-Output ("--- M{0} TARGET order ---" -f $m)
    $vals = @($state[$m].tgtSeq | ForEach-Object { $_.tgt })
    Write-Output ("  observed TARGET sequence: " + (($vals | ForEach-Object { [math]::Round($_,2) }) -join "  ->  "))
    if ($vals.Count -ge 2) {
        $obs = @()
        for ($i = 1; $i -lt $vals.Count; $i++) { $obs += [math]::Round($vals[$i] - $vals[$i-1], 2) }
        $exp = @()
        for ($i = 1; $i -lt $steps.Count; $i++) { $exp += [math]::Round($steps[$i] - $steps[$i-1], 2) }
        Write-Output ("  observed deltas (deg): " + ($obs -join "  ,  ") + "   expected: " + ($exp -join "  ,  "))
    }
}

Write-Output ("--- total run {0} ms ---" -f $total)

# compact timeline, first 3.5 s
foreach ($m in @(1,2)) {
    Write-Output ("--- M{0} timeline (first 3500 ms) ---" -f $m)
    Write-Output "    t(ms)   err(deg)    VEL"
    foreach ($p in $state[$m].log) {
        if ($p.t -gt 3500) { break }
        "  {0,7}  {1,8}  {2,6}" -f $p.t, [math]::Round($p.e, 2), [math]::Round($p.v, 2)
    }
}
