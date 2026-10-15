# =====================================================================
# STM32_2_motor - one-function-per-step regression suite.
#
# Each step exercises EXACTLY ONE function and is judged PASS/FAIL on its
# own, so a failure localises to a single feature. Steps that probe a
# suspected gap are marked "(gap)" - a FAIL there is a FINDING, not noise.
#
# ASCII only on purpose: PowerShell 5.1 on a zh-CN system reads .ps1 as
# GBK, so non-ASCII literals in this file would be mangled. All Chinese
# replies are matched through their stable ASCII fragments instead
# (e.g. bare "ERR:" / "OK " / the "STATE=" / "PWM=" field names, never the
# Chinese text). Note the old "ERR: ... HELP" tail is gone: the command table
# was deleted from the firmware on 2026-10-03 (see Debug/HELP.txt).
#
#   .\suite.ps1                 run every group
#   .\suite.ps1 -Only 1.1,1.2   run a subset
# =====================================================================
param(
    [string]   $PortName = "COM4",
    [string[]] $Only     = @(),
    [switch]   $AuditErr
)

$ErrorActionPreference = "Stop"

$CLI = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$ADDR_CSR  = "0x40021024"   # RCC_CSR  - reset cause latch
# uwTick - uptime ms, monotonic. RE-READ THIS FROM THE .map AFTER EVERY BUILD:
# it is a RAM address and shifts whenever any .bss object changes size. It moved
# 0x200014B0 -> 0x200015F0 on 2026-10-03, exactly +0x140 (320 bytes), which is
# the RX_BUF_SIZE 64 -> 128 change. The stale address does not fail loudly - it
# silently reads 0 and steps 0.4 / 0.5 report "+0 ms" as if the clock had died.
#   Select-String -Path Debug\STM32_2_motor.map -Pattern '\.bss\.uwTick'
$ADDR_TICK = "0x200015F0"

# ---------------------------------------------------------------- results
$script:Results = New-Object System.Collections.ArrayList

function Check {
    param([string]$Id, [string]$Func, [bool]$Pass, [string]$Evidence = "")
    $tag = "FAIL"
    if ($Pass) { $tag = "PASS" }
    [void]$script:Results.Add([pscustomobject]@{
        Step = $Id; Func = $Func; Result = $tag; Evidence = $Evidence })
    $colour = "Red"
    if ($Pass) { $colour = "Green" }
    Write-Host ("  [{0}] {1,-5} {2}" -f $tag, $Id, $Func) -ForegroundColor $colour
    if ($Evidence -ne "") { Write-Host ("           {0}" -f $Evidence) -ForegroundColor DarkGray }
    # -AuditErr: after every step, read BOTH motors' ERR. A step that leaves a
    # latched fault behind poisons whichever step runs next, and the suite had no
    # way to see that: each Check judges only its own evidence. A step can even
    # pass BECAUSE of inherited state, or fail because of it - found exactly that
    # on 2026-10-03, when 4.6 failed on an ERR=32 it never produced
    # (Debug/diag_stall.ps1 replayed 4.6's sequence from a cleared latch: ERR=0
    # the whole way, final error 0.06 deg, identical to the failing run).
    # Costs ~0.5 s per step, so it is OFF by default; turn it on when hunting a
    # cross-step leak.
    if ($AuditErr) {
        # Inline the ERR parse rather than calling St-Field or Get-Err: both are
        # defined far below (near step 6.2 / Frame-Str), and this audit also runs
        # for the early steps - PowerShell resolves a function call at run time,
        # so calling one here would throw at step 0.6.
        # Match over the WHOLE reply, not the last line: while a motor is RUNNING
        # the window ends in a half-received telemetry frame, and a last-line read
        # answers "?" - which the `-ne "0"` test below would report as a LEAK on a
        # healthy board. "?" must mean UNKNOWN, not FAULT.
        $e1 = "?"; $e2 = "?"
        $r1 = Send-Cmd "M1 STATUS" 250
        if ($r1 -match "ERR=(\d+)") { $e1 = $matches[1] }
        $r2 = Send-Cmd "M2 STATUS" 250
        if ($r2 -match "ERR=(\d+)") { $e2 = $matches[1] }
        if (($e1 -ne "0" -and $e1 -ne "?") -or ($e2 -ne "0" -and $e2 -ne "?")) {
            Write-Host ("           LEAK after {0}: M1 ERR={1}  M2 ERR={2}" -f $Id, $e1, $e2) -ForegroundColor Magenta
        }
    }
}

function Want-Run {
    param([string]$Id)
    if ($Only.Count -eq 0) { return $true }
    foreach ($o in $Only) { if ($o -eq $Id) { return $true } }
    return $false
}

# ---------------------------------------------------------------- serial
$script:Port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$script:Port.ReadTimeout  = 40
$script:Port.WriteTimeout = 1000
$script:Port.DtrEnable    = $true
$script:Port.RtsEnable    = $true

# Send ONE command and return everything received in the window.
# The input buffer is discarded first so the reply cannot be confused with
# leftover telemetry from a previous step.
function Send-Cmd {
    param([string]$Cmd, [int]$WaitMs = 500)
    $script:Port.DiscardInBuffer()
    $script:Port.Write($Cmd + "`r`n")
    $ms = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $WaitMs) {
        try {
            $n = $script:Port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $ms.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    $bytes = $ms.ToArray()
    if ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF) {
        $bytes = $bytes[3..($bytes.Length - 1)]
    }
    return [System.Text.Encoding]::UTF8.GetString($bytes)
}

# Write raw bytes with no line terminator appended by us - used to build an
# overlong line and a back-to-back burst.
function Send-Raw {
    param([string]$Text)
    $script:Port.DiscardInBuffer()
    $script:Port.Write($Text)
}

function Read-For {
    param([int]$Ms = 500)
    # NB: do not name this buffer $ms - PowerShell variable names are
    # case-insensitive, so it would collide with the [int]$Ms parameter.
    $buf = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $Ms) {
        try {
            $n = $script:Port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $buf.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($buf.ToArray())
}

# Largest deviation of any PWM triplet in the text from the 0.50 zero vector.
function Max-Pwm-Dev {
    param([string]$Text)
    $dev = 0.0
    foreach ($line in (All-Lines $Text)) {
        if ($line -match "PWM=([0-9.]+),([0-9.]+),([0-9.]+)") {
            foreach ($g in 1..3) {
                $d = [math]::Abs([double]$matches[$g] - 0.5)
                if ($d -gt $dev) { $dev = $d }
            }
        }
    }
    return $dev
}

# First non-empty line, trimmed - keeps evidence short.
function First-Line {
    param([string]$Text)
    foreach ($l in ($Text -split "`r?`n")) {
        $t = $l.Trim()
        if ($t -ne "") { return $t }
    }
    return ""
}

function All-Lines {
    param([string]$Text)
    return @(($Text -split "`r?`n") | Where-Object { $_.Trim() -ne "" })
}

# ---------------------------------------------------------------- swd
# Returns -1 (Int32) on failure. Do NOT return [uint32]0xFFFFFFFF: PowerShell
# 5.1 parses that literal as Int32 -1 first and the cast then throws.
function Get-U32 {
    param([string]$Addr, [int]$Tries = 3)
    for ($i = 0; $i -lt $Tries; $i++) {
        $out = & $CLI -c port=SWD mode=hotplug -r32 $Addr 4 2>&1
        foreach ($line in $out) {
            if ($line -match '^\s*0x[0-9A-Fa-f]{8}\s*:') {
                $body = ($line -split ':', 2)[1]
                foreach ($tok in ($body -split '\s+')) {
                    if ($tok -match '^[0-9A-Fa-f]{8}$') { return [Convert]::ToUInt32($tok, 16) }
                }
            }
        }
        Start-Sleep -Milliseconds 150
    }
    return -1
}

function Clear-Flags { [void](& $CLI -c port=SWD mode=hotplug -w32 $ADDR_CSR 0x01000000 2>&1) }

function Describe-Csr {
    param([int]$Csr)
    if ($Csr -le 0) { return "none" }
    $n = @()
    if ($Csr -band 0x08000000) { $n += "POR(brownout)" }
    if ($Csr -band 0x04000000) { $n += "PIN(NRST)" }
    if ($Csr -band 0x10000000) { $n += "SOFT" }
    if ($Csr -band 0x20000000) { $n += "IWDG" }
    if ($Csr -band 0x40000000) { $n += "WWDG" }
    if ($Csr -band 0x80000000) { $n += "LPWR" }
    if ($n.Count -eq 0) { return ("unknown 0x{0:X8}" -f $Csr) }
    return ($n -join "+")
}

# Stop both motors AND clear both fault latches.
#
# The CLR is deliberate. The suite's contract is ONE function per step, which
# means each step has to start from a clean state - and a latched ERR bit is
# state, not a measurement. Measured 2026-10-03 with -AuditErr: step 1.28
# leaves M1 ERR=32 behind and step 2.3 leaves M2 ERR=32, and every later step
# that asserts on ERR then fails for a reason it did not cause. That is exactly
# how 4.6 was failing (Debug/diag_stall.ps1 replayed its sequence from a
# cleared latch: ERR=0 the whole way, same 0.06 deg final error).
# Group 6 is unaffected: its latch steps build their own fault, and 6.3 - the
# one step that inherits 6.2's latch on purpose - never calls this.
function Stop-All {
    [void](Send-Cmd "STOPALL" 400)
    Start-Sleep -Milliseconds 120
    [void](Send-Cmd "M1 CLR" 300)
    [void](Send-Cmd "M2 CLR" 300)
    Start-Sleep -Milliseconds 120
}

# ---------------------------------------------------------------- open
try { $script:Port.Open() } catch {
    Write-Host "OPEN FAILED on $PortName : $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
Start-Sleep -Milliseconds 150
$script:Port.DiscardInBuffer()

# Record WHY the board last reset (a stale latch from before the run is an
# observation, not a failure), then clear it so 0.1/0.5 mean "during this run".
$csrPre = Get-U32 $ADDR_CSR
Clear-Flags
Start-Sleep -Milliseconds 200
$csr0  = Get-U32 $ADDR_CSR
$tick0 = Get-U32 $ADDR_TICK

Write-Host ""
Write-Host "===== STM32_2_motor suite =====" -ForegroundColor Cyan
Write-Host ("port {0}" -f $PortName)
Write-Host ("boot cause before clear : {0}  (RCC_CSR=0x{1:X8})" -f (Describe-Csr $csrPre), $csrPre)
Write-Host ("after clear             : RCC_CSR=0x{0:X8}  uwTick baseline={1} ms" -f $csr0, $tick0)
Write-Host ""

# =====================================================================
# GROUP 0 - link and clock. Proves the DATA is trustworthy before we
# judge any feature with it.
# =====================================================================
Write-Host "--- Group 0: link / clock ---" -ForegroundColor Cyan

# 0.1 the clear actually took (write-1-to-clear RMVF works over SWD)
if (Want-Run "0.1") {
    Check "0.1" "RCC_CSR cleared to 0" ($csr0 -eq 0) ("RCC_CSR=0x{0:X8}  (was 0x{1:X8}: {2})" -f $csr0, $csrPre, (Describe-Csr $csrPre))
}

# 0.2 board answers at all
if (Want-Run "0.2") {
    $r = Send-Cmd "STATUS" 600
    $ok = ($r -match "M1 STATE=") -and ($r -match "M2 STATE=")
    Check "0.2" "console alive (STATUS)" $ok (First-Line $r)
}

# 0.3 The longest remaining message fits the TX ring in one shot. This step used
#      to send HELP - a ~1.8 KB block, by far the largest thing the firmware
#      emits. HELP was deleted from the firmware on 2026-10-03 to reclaim ~1250
#      bytes of FLASH (the text is preserved in Debug/HELP.txt), so the longest
#      reply is now the two-motor STATUS (~260 B) against a 3071 B usable ring.
#      The property under test is unchanged: one call must arrive complete, with
#      no [TX DROP] marker and nothing truncated.
if (Want-Run "0.3") {
    $r = Send-Cmd "STATUS" 900
    $hasBoth = ($r -match "M1 STATE=") -and ($r -match "M2 STATE=")
    $dropped = $r -match "TX DROP"
    Check "0.3" "longest reply fits TX ring" ($hasBoth -and (-not $dropped)) ("len={0} both={1} drop={2}" -f $r.Length, $hasBoth, $dropped)
}

# 0.4 uptime is monotonic (checked again at the end of the run)
if (Want-Run "0.4") {
    Start-Sleep -Milliseconds 700
    $tick1 = Get-U32 $ADDR_TICK
    $adv = $tick1 - $tick0
    Check "0.4" "uwTick monotonic +" (($tick1 -ge 0) -and ($adv -ge 500)) ("{0} -> {1} (+{2} ms)" -f $tick0, $tick1, $adv)
}

# 0.6 known-good state for the rest of the run. This is a TEST-HARNESS
#     precondition, not a workaround for a flaky board.
#     Its original reason was defect #5: the PID integral used to be cleared by
#     NOTHING except KP/KI/VEL, so a fault injected by Group 6 in a previous run
#     left the loop poisoned and the next SPD re-tripped the stall guard inside
#     its 2 s window. That defect is FIXED (2026-10-03: Motor_Disable and
#     Motor_SetError now clear the integral, so STOPALL does too), so the
#     poisoning path is gone. The step is kept anyway because it is still a real
#     precondition: it also returns both KPs to the tuned 0.05, which several
#     steps deliberately change.
#
#     Deliberately NOT behind Want-Run: a precondition that `-Only 3.5` skips
#     is not a precondition. Without it every subset run inherits the poison
#     and reports nonsense (measured: `-Only 2.1` gave 8 identical ANGLE
#     frames because SPD 20 never turned the rotor).
if ($true) {
    Stop-All
    Start-Sleep -Milliseconds 300
    [void](Send-Cmd "M1 CLR" 300)
    [void](Send-Cmd "M2 CLR" 300)
    [void](Send-Cmd "M1 KP 0.05" 300)
    [void](Send-Cmd "M2 KP 0.05" 300)
    Start-Sleep -Milliseconds 300
    $s = Send-Cmd "STATUS" 600
    $m1 = "?"; $m2 = "?"; $e1 = "?"; $e2 = "?"
    if ($s -match "M1 STATE=(\S+)") { $m1 = $matches[1] }
    if ($s -match "M2 STATE=(\S+)") { $m2 = $matches[1] }
    if ($s -match "M1 STATE=\S+.*?ERR=(\d+)") { $e1 = $matches[1] }
    if ($s -match "M2 STATE=\S+.*?ERR=(\d+)") { $e2 = $matches[1] }
    $bad = ($s -match "ERR=[1-9]")
    # The evidence line must carry the REAL ERR values. It used to end with the
    # hardcoded phrase "no ERR bits", which kept printing even when a motor was
    # latched - and it did: a run reported "M1=READY M2=ERROR, no ERR bits"
    # while M2 was actually sitting at ERR=2 (ENCODER). A harness that asserts
    # its own conclusion in the evidence string cannot be trusted to catch it.
    Check "0.6" "clean slate (state+integral)" ((-not $bad) -and ($m1 -eq "READY") -and ($m2 -eq "READY")) ("M1={0} ERR={1}  M2={2} ERR={3}  (STOPALL+CLR+KP both motors)" -f $m1, $e1, $m2, $e2)
    if (-not ((-not $bad) -and ($m1 -eq "READY") -and ($m2 -eq "READY"))) {
        Write-Host "       NOTE: a latched motor cannot be cleared from the console. ERR=2 with ERR bits" -ForegroundColor Yellow
        Write-Host "             exhausted (I2C_RECOVER_MAX) needs a POWER CYCLE / NRST, not CLR. See the report." -ForegroundColor Yellow
    }
}

# =====================================================================
# GROUP 1 - command layer, ONE command per step.
# =====================================================================
Write-Host ""
Write-Host "--- Group 1: command layer ---" -ForegroundColor Cyan
Stop-All

# 1.1 HELP and "?" are gone. Removed-by-deletion 2026-10-03: the command table
#     was cut from the firmware to reclaim ~1250 bytes of FLASH (backup:
#     Debug/HELP.txt), so both spellings must now be REJECTED as unknown rather
#     than printing a ~1.8 KB banner. This step used to assert the banner.
if (Want-Run "1.1") {
    $a = Send-Cmd "HELP" 500
    $b = Send-Cmd "?" 500
    $ok = ($a -match "ERR:") -and ($b -match "ERR:") -and (-not ($a -match "====="))
    Check "1.1" "HELP/? removed (ERR)" $ok (First-Line $a)
}

# 1.2 unknown top-level command
if (Want-Run "1.2") {
    $r = Send-Cmd "FOOBAR" 400
    Check "1.2" "unknown top-level ERR" ($r -match "ERR:") (First-Line $r)
}

# 1.3 M1 with no subcommand
if (Want-Run "1.3") {
    $r = Send-Cmd "M1" 400
    Check "1.3" "missing subcommand ERR" ($r -match "ERR:") (First-Line $r)
}

# 1.4 M1 with an unknown subcommand
if (Want-Run "1.4") {
    $r = Send-Cmd "M1 BOGUS" 400
    Check "1.4" "unknown subcommand ERR" ($r -match "ERR:") (First-Line $r)
}

# 1.5 bad motor id is rejected, not silently treated as M1
if (Want-Run "1.5") {
    $r = Send-Cmd "M3 STATUS" 400
    $r2 = Send-Cmd "5 STATUS" 400
    Check "1.5" "bad motor id rejected" (($r -match "ERR:") -and ($r2 -match "ERR:")) (("{0} | {1}" -f (First-Line $r), (First-Line $r2)))
}

# 1.6 START / STOP round trip. "OK 电机已启动" has no ASCII, so assert on
#     the observable state instead of the reply text.
if (Want-Run "1.6") {
    [void](Send-Cmd "M1 START" 500)
    Start-Sleep -Milliseconds 300
    $r = Send-Cmd "M1 STATUS" 400
    $running = $r -match "M1 STATE=RUNNING"
    [void](Send-Cmd "M1 STOP" 400)
    Start-Sleep -Milliseconds 300
    $r2 = Send-Cmd "M1 STATUS" 400
    $stopped = $r2 -match "M1 STATE=READY"
    Check "1.6" "START then STOP" ($running -and $stopped) (("{0} | {1}" -f (First-Line $r), (First-Line $r2)))
}

# 1.7 STOPALL stops both
if (Want-Run "1.7") {
    [void](Send-Cmd "M1 SPD 5" 300)
    [void](Send-Cmd "M2 SPD 5" 300)
    Start-Sleep -Milliseconds 600
    $r = Send-Cmd "STOPALL" 600
    Start-Sleep -Milliseconds 300
    $r2 = Send-Cmd "STATUS" 500
    $ok = ($r -match "OK STOPALL") -and ($r2 -match "M1 STATE=READY") -and ($r2 -match "M2 STATE=READY")
    Check "1.7" "STOPALL stops both" $ok (First-Line $r)
}

# 1.8 SPD with no argument -> error, no state change
if (Want-Run "1.8") {
    Stop-All
    $r = Send-Cmd "M1 SPD" 400
    $r2 = Send-Cmd "M1 STATUS" 400
    $ok = ($r -match "ERR: SPD") -and ($r2 -match "M1 STATE=READY")
    Check "1.8" "SPD missing arg ERR" $ok (First-Line $r)
}

# 1.9 POS with no argument -> error
if (Want-Run "1.9") {
    $r = Send-Cmd "M2 POS" 400
    Check "1.9" "POS missing arg ERR" ($r -match "ERR: POS") (First-Line $r)
}

# 1.10 VOL with no argument -> error
if (Want-Run "1.10") {
    $r = Send-Cmd "M2 VOL" 400
    Check "1.10" "VOL missing arg ERR" ($r -match "ERR: VOL") (First-Line $r)
}

# 1.11 KP clamp both directions + integral note
if (Want-Run "1.11") {
    $a = Send-Cmd "M1 KP 5" 400
    $b = Send-Cmd "M1 KP -5" 400
    $ok = ($a -match "OK KP=0\.100") -and ($a -match "WARN clamped") -and
          ($b -match "OK KP=-0\.100") -and ($b -match "WARN clamped")
    Check "1.11" "KP clamps +/-" $ok (First-Line $a)
    [void](Send-Cmd "M1 KP 0.05" 300)
}

# 1.12 KI clamp: upper to 0.5, lower to 0 (negative ki is positive feedback)
if (Want-Run "1.12") {
    $a = Send-Cmd "M1 KI 5" 400
    $b = Send-Cmd "M1 KI -1" 400
    $ok = ($a -match "OK KI=0\.500") -and ($a -match "WARN clamped") -and
          ($b -match "OK KI=0\.000") -and ($b -match "WARN clamped")
    Check "1.12" "KI clamps [0,0.5]" $ok (First-Line $a)
    [void](Send-Cmd "M1 KI 0.5" 300)
}

# 1.13 VEL clamps kp and ki
if (Want-Run "1.13") {
    $a = Send-Cmd "M1 VEL 5 5 0" 400
    $ok = ($a -match "OK VEL: kp=0\.100") -and ($a -match "ki=0\.500") -and ($a -match "WARN clamped")
    Check "1.13" "VEL clamps kp+ki" $ok (First-Line $a)
    [void](Send-Cmd "M1 VEL 0.05 0.5 0" 300)
}

# 1.14 (gap) VEL must clamp kd too. FIXED 2026-10-03: kd was the LAST unclamped
#      gain (KP and KI were clamped, and VEL clamped kp/ki but passed kd straight
#      through), so `M1 VEL 0.05 0.5 999` wrote kd=999. Now clamped to
#      [0, MOTOR_VEL_KD_MAX=0.1] with the shared UART_WARN_CLAMPED suffix.
#      kd=0 factory; set a big one, read it back, then restore.
if (Want-Run "1.14") {
    $a = Send-Cmd "M1 VEL 0.05 0.5 999" 400
    $accepted = ($a -match "kd=999") -and (-not ($a -match "WARN clamped"))
    Check "1.14" "(gap) VEL kd unclamped" (-not $accepted) (First-Line $a)
    [void](Send-Cmd "M1 VEL 0.05 0.5 0" 300)
}

# 1.15 LIMIT clamps to the 6.6 V safe ceiling and reports the value
if (Want-Run "1.15") {
    $a = Send-Cmd "M1 LIMIT 12" 400
    $ok = ($a -match "WARN LIMIT") -and ($a -match "6\.60") -and ($a -match "OK LIMIT")
    Check "1.15" "LIMIT clamped to 6.6V" $ok (First-Line $a)
    [void](Send-Cmd "M1 LIMIT 6.6" 300)
}

# 1.16 (gap) LIMIT takes the magnitude: a negative request must not produce
#      output_min > output_max, which would invert the PID clamp and pin the
#      output at the limit. LIMIT has no query form, so this is judged
#      BEHAVIOURALLY: at -3 V a 5 rad/s target needs ~0.33 V, so a correct
#      clamp leaves the motor at ~5 rad/s; an inverted clamp pins it at 3 V
#      and it runs away to ~45 rad/s.
if (Want-Run "1.16") {
    Stop-All
    $a = Send-Cmd "M1 LIMIT -3" 400
    [void](Send-Cmd "M1 SPD 5" 500)
    Start-Sleep -Milliseconds 1800
    $r = Send-Cmd "M1 STATUS" 400
    $vel = -999.0
    if ($r -match "VEL=(-?[0-9.]+)") { $vel = [double]$matches[1] }
    $ok = ($a -match "OK LIMIT") -and ($vel -gt 3.0) -and ($vel -lt 8.0)
    Check "1.16" "(gap) LIMIT abs() of arg" $ok ("reply='{0}' VEL={1} (inverted clamp would pin ~45)" -f (First-Line $a), $vel)
    Stop-All
    [void](Send-Cmd "M1 LIMIT 6.6" 300)
}

# 1.17 FF query vs set, and the value survives a read-back
if (Want-Run "1.17") {
    $q0 = Send-Cmd "M1 FF" 400
    $s  = Send-Cmd "M1 FF 1.5" 400
    $q1 = Send-Cmd "M1 FF" 400
    $ok = ($q0 -match "M1 FF=") -and ($s -match "OK M1 FF=1\.500") -and ($q1 -match "1\.500")
    Check "1.17" "FF query/set/readback" $ok (("{0} -> {1}" -f (First-Line $q0), (First-Line $q1)))
    [void](Send-Cmd "M1 FF 0" 300)
}

# 1.18 TRAJ query, and the 1-arg form means amax (NOT vmax)
if (Want-Run "1.18") {
    $q0 = Send-Cmd "M1 TRAJ" 400
    [void](Send-Cmd "M1 TRAJ 45" 400)
    $q1 = Send-Cmd "M1 TRAJ" 400
    $ok = ($q0 -match "TRAJ vmax=") -and ($q1 -match "amax=45\.00") -and ($q1 -match "vmax=20\.00")
    Check "1.18" "TRAJ 1-arg is amax" $ok (First-Line $q1)
}

# 1.19 TRAJ clamps: vmax<=20, amax>=1, dead within [1 count, 0.05 rad]
if (Want-Run "1.19") {
    $a = Send-Cmd "M1 TRAJ 99 0 99 99" 400
    $ok = ($a -match "WARN") -and ($a -match "OK M1 TRAJ")
    Check "1.19" "TRAJ clamps out-of-range" $ok (First-Line $a)
    [void](Send-Cmd "M1 TRAJ 20 60 0.5 1.5" 300)
}

# 1.20 DIR bare form is read-only and does NOT change pole_pairs
if (Want-Run "1.20") {
    $a = Send-Cmd "M1 DIR" 400
    $b = Send-Cmd "M1 DIR" 400
    $c = Send-Cmd "M1 DIR" 400
    $stable = ($a -match "pole_pairs=7\.0") -and ($b -match "pole_pairs=7\.0") -and ($c -match "pole_pairs=7\.0")
    $noOk   = -not ($a -match "OK M1 DIR")
    Check "1.20" "bare DIR read-only" ($stable -and $noOk) (First-Line $a)
}

# 1.21 DIR set form still works and is reversible
if (Want-Run "1.21") {
    $a = Send-Cmd "M1 DIR -1" 400
    $b = Send-Cmd "M1 DIR 1" 400
    $ok = ($a -match "OK M1 DIR" -and $a -match "REVERSED") -and
          ($b -match "OK M1 DIR" -and $b -match "normal")
    Check "1.21" "DIR set form reversible" $ok (("{0} | {1}" -f (First-Line $a), (First-Line $b)))
}

# 1.22 ZERO latches the origin; POS 0 then reads back ~0
if (Want-Run "1.22") {
    Stop-All
    Start-Sleep -Milliseconds 800
    [void](Send-Cmd "M1 ZERO" 400)
    [void](Send-Cmd "M1 POS 0" 400)
    Start-Sleep -Milliseconds 800
    $r = Send-Cmd "M1 STATUS" 400
    # ZERO sets position_offset = mechanical_angle, so POS 0 targets the angle
    # the rotor is ALREADY at: TARGET is the absolute angle, and the pass
    # condition is ANGLE == TARGET, not TARGET == 0.
    $ang = -999.0; $tgt = -999.0
    if ($r -match "ANGLE=(-?[0-9.]+)")  { $ang = [double]$matches[1] }
    if ($r -match "TARGET=(-?[0-9.]+)") { $tgt = [double]$matches[1] }
    $err = [math]::Abs($ang - $tgt)
    Check "1.22" "ZERO then POS 0 holds" ($err -lt 0.15) ("ANGLE={0} TARGET={1} err={2:N3} deg" -f $ang, $tgt, $err)
    Stop-All
}

# 1.23 CLR clears the error latch
if (Want-Run "1.23") {
    $r = Send-Cmd "M1 CLR" 400
    $r2 = Send-Cmd "M1 STATUS" 400
    $ok = ($r -match "OK CLR") -and ($r2 -match "ERR=0")
    Check "1.23" "CLR clears faults" $ok (First-Line $r)
}

# 1.24 garbage argument on a bare-capable command is treated as a QUERY,
#      not an error, and must not change state (TRAJ/FF/DIR/ENP/GUARD).
if (Want-Run "1.24") {
    $a = Send-Cmd "M1 FF abc" 400
    $b = Send-Cmd "M1 GUARD abc" 400
    $ok = ($a -match "M1 FF=") -and ($b -match "OK GUARD") -and (-not ($a -match "ERR:"))
    Check "1.24" "garbage arg == query" $ok (First-Line $a)
}

# 1.25 (gap) loose numeric validation: "0.5abc" must be rejected. FIXED
#      2026-10-03: every numeric argument now goes through uart_parse_f, which
#      requires the whole token to be consumed AND the value to be finite - so
#      trailing garbage, nan, +-inf and overflow ("1e400") are all rejected.
if (Want-Run "1.25") {
    $a = Send-Cmd "M1 VEL 0.05 0.5xyz 0" 400
    # MUST be "ERR:" not "ERR": the periodic telemetry line contains "ERR=0",
    # so a bare "ERR" substring matches a perfectly healthy reply and makes this
    # step pass no matter what. Same trap at 1.24.
    $rejected = $a -match "ERR:"
    Check "1.25" "(gap) loose sub-arg parse" $rejected (First-Line $a)
    [void](Send-Cmd "M1 VEL 0.05 0.5 0" 300)
}

# 1.26 (gap) overlong line must not be silently swallowed. FIXED 2026-10-03:
#      RX_BUF_SIZE 64 -> 128 (uart_debug.c), so the 63-char ceiling that used to
#      truncate this 67-char line is now 127. Cost was RAM only (+320 B), which
#      is why it was affordable at 328 bytes of free FLASH. Residual: a line
#      longer than 127 chars is still truncated silently; no real command comes
#      close (longest is ~50 chars). Build
#      " "<60 + "M1 STOP" (67 chars) with M1 already RUNNING:
#        not truncated -> M1 stops
#        truncated     -> "M1 " only -> missing-subcommand ERR, M1 keeps running
if (Want-Run "1.26") {
    [void](Send-Cmd "M1 SPD 2" 500)
    Start-Sleep -Milliseconds 500
    Send-Raw ((" " * 60) + "M1 STOP" + "`r`n")
    $r = Read-For 700
    Start-Sleep -Milliseconds 200
    $st = Send-Cmd "M1 STATUS" 400
    $stillRunning = $st -match "M1 STATE=RUNNING"
    Check "1.26" "(gap) >63 char line" (-not $stillRunning) ("reply='{0}' state={1}" -f (First-Line $r), $(if ($stillRunning) { "RUNNING" } else { "stopped" }))
    Stop-All
}

# 1.27 burst delivery: five back-to-back commands, last one must win
if (Want-Run "1.27") {
    Stop-All
    Send-Raw ("M1 SPD 1`r`nM1 SPD 2`r`nM1 SPD 3`r`nM1 SPD 4`r`nM1 SPD 5`r`n")
    Start-Sleep -Milliseconds 900
    $r = Send-Cmd "M1 STATUS" 400
    $ok = $r -match "TARGET=5\.00"
    Check "1.27" "5-command burst, last wins" $ok (First-Line $r)
    Stop-All
}

# 1.28 (gap) VOL must not be hijacked by a leftover OLTEST. FIXED 2026-10-03:
#      OLTEST leaves openloop_vel/volt set, and Motor_OpenLoop overwrites
#      target_voltage = openloop_volt every tick (motor_control.c:894-902)
#      whenever openloop_vel != 0, so VOL's value was silently discarded. STOP
#      already cleared those fields; the VOL handler now does the same
#      (openloop_vel/volt = 0 + Motor_RampStop). 3 V -> PWM spread ~0.25;
#      the stale 1 V -> ~0.08.
#
#      WATCHDOG NOTE - the sampling window is 1.4 s ON PURPOSE. Now that VOL
#      really takes effect it also stops the electrical vector rotating: the
#      open-loop field is static, so the rotor aligns and then does not move,
#      and after MOTOR_STALL_TIMEOUT_MS (2 s) at 3 V the stall guard fires. That
#      is CORRECT behaviour for a 12 V motor fed DC (see the voltage-rating
#      trap) and diag_stall.ps1 measured it precisely: ERR 0 -> 32 at t=2027 ms
#      with the rotor moving 0.08 deg, while the 0.5 V negative control held 4 s
#      with ERR=0. This step must therefore read its PWM BEFORE the guard trips -
#      the previous 2.6 s window straddled the trip, which left M1 ERR=32 latched
#      for every later step (that is what broke 4.6). The steady state this step
#      measures is reached in well under 1 s.
if (Want-Run "1.28") {
    Stop-All
    [void](Send-Cmd "M1 OLTEST 6 1" 400)     # leaves openloop_vel=6, volt=1
    [void](Send-Cmd "M1 VOL 3" 400)          # no STOP in between - that is the bug
    $r = Read-For 1400                       # < 2 s guard window, see note above
    $dev = Max-Pwm-Dev $r
    $ok = $dev -gt 0.16
    Check "1.28" "(gap) OLTEST hijacks VOL" $ok ("max |PWM-0.5| = {0:N3} (3V->~0.25 honored, 1V->~0.08 hijacked)" -f $dev)
    Stop-All
}

# 1.29 (gap) the velocity PID integrator survives STOP / Disable / CLR
#      (verified in motor.c:91-133 - neither Motor_Enable nor Motor_Disable
#      touches velocity_pid.integral).
#
#      MEASUREMENT NOTE: do NOT judge this on the PWM triplet. The unwound
#      output is a ~30 ms transient and the only telemetry is 500 ms, so a
#      PWM-based check reads 0.000 whether or not the integral survived -
#      it produced a false PASS on the first run. Judge the CONSEQUENCE
#      instead: with the rotor at rest the P term is zero, so any output
#      after `SPD 0` is pure ki*integral and it LURCHES the rotor. A cleared
#      integral leaves the rotor parked; a surviving one moves it visibly.
if (Want-Run "1.29") {
    Stop-All
    [void](Send-Cmd "M1 SPD 100" 500)
    Start-Sleep -Milliseconds 2500            # integral winds to the 6.6 V clamp
    [void](Send-Cmd "M1 STOP" 400)
    # VEL is FROZEN in READY, so poll ANGLE until two consecutive reads agree.
    # The coast from ~66 rad/s takes anywhere from 0.8 s to several seconds.
    $prev = -999.0; $g2 = -999.0; $rest = $false
    for ($i = 0; $i -lt 25; $i++) {
        Start-Sleep -Milliseconds 400
        $s = Send-Cmd "M1 STATUS" 300
        $a = -999.0
        if ($s -match "ANGLE=(-?[0-9.]+)") { $a = [double]$matches[1] }
        if ($prev -gt -900.0) {
            $dd = [math]::Abs($a - $prev)
            if ($dd -gt 180.0) { $dd = 360.0 - $dd }
            if ($dd -lt 0.05) { $rest = $true; $g2 = $a; break }
        }
        $prev = $a
    }
    [void](Send-Cmd "M1 SPD 0" 300)
    Start-Sleep -Milliseconds 1500
    $a3 = Send-Cmd "M1 STATUS" 300
    $g3 = -999.0
    if ($a3 -match "ANGLE=(-?[0-9.]+)") { $g3 = [double]$matches[1] }
    $d = $g3 - $g2
    while ($d -gt  180.0) { $d -= 360.0 }
    while ($d -lt -180.0) { $d += 360.0 }
    $lurch = [math]::Abs($d)
    $ok = $rest -and ($lurch -lt 0.3)
    Check "1.29" "(gap) integral survives STOP" $ok ("at-rest={0} ANGLE {1} -> {2} = {3:N2} deg lurch at target 0" -f $rest, $g2, $g3, $lurch)
    Stop-All
}

# ---------------------------------------------------------------- helpers
# Firmware VEL field statistics. Judge stability on the SPAN of the firmware's
# own VEL, never on an ANGLE-derived average - a 500 ms average hides a limit
# cycle completely (see the velocity-tuning notes).
function Vel-Stats {
    param([string]$Text)
    $vs = @()
    foreach ($line in (All-Lines $Text)) {
        if ($line -match "VEL=(-?[0-9.]+)") { $vs += [double]$matches[1] }
    }
    if ($vs.Count -eq 0) { return $null }
    $m  = ($vs | Measure-Object -Average).Average
    $mn = ($vs | Measure-Object -Minimum).Minimum
    $mx = ($vs | Measure-Object -Maximum).Maximum
    $sd = 0.0
    if ($vs.Count -gt 1) {
        $s = 0.0
        foreach ($v in $vs) { $s += ($v - $m) * ($v - $m) }
        $sd = [math]::Sqrt($s / ($vs.Count - 1))
    }
    return [pscustomobject]@{ N = $vs.Count; Mean = $m; Min = $mn; Max = $mx; Span = ($mx - $mn); Sd = $sd }
}

function Run-Spd {
    param([string]$Motor, [double]$Target, [int]$Ms = 3000)
    [void](Send-Cmd "$Motor SPD $Target" 300)
    Start-Sleep -Milliseconds 700          # let it reach steady state
    return (Read-For $Ms)
}

# Move to a position and wait for AT-TARGET, not merely stationary
# (STATE=READY does not mean arrived, and a stopped rotor is not an arrived one).
function Pos-Goto {
    param([string]$Motor, [double]$Deg, [int]$MaxMs = 7000, [double]$Tol = 0.15)
    [void](Send-Cmd "$Motor POS $Deg" 300)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $MaxMs) {
        Start-Sleep -Milliseconds 250
        # Parse ONE frame for THIS motor (see Motor-Frame): while the other
        # motor runs, a mixed window can otherwise pair this motor's ANGLE
        # with the other motor's TARGET and report a huge phantom error.
        $f = Motor-Frame (Send-Cmd "$Motor STATUS" 300) $Motor
        $ang = Frame-Field $f "ANGLE"
        $tgt = Frame-Field $f "TARGET"
        if ($ang -gt -900.0 -and $tgt -gt -900.0) {
            $e = [math]::Abs((Wrap180 ($ang - $tgt)))
            if ($e -lt $Tol) { return $true }
        }
    }
    return $false
}

function Wrap180 {
    param([double]$D)
    while ($D -gt  180.0) { $D -= 360.0 }
    while ($D -lt -180.0) { $D += 360.0 }
    return $D
}

function Get-Angle {
    param([string]$Text, [string]$Field)
    if ($Text -match "$Field=(-?[0-9.]+)") { return [double]$matches[1] }
    return -999.0
}

# ★ Last COMPLETE telemetry frame for ONE motor in a reply window.
# When both motors are RUNNING, the window of `M1 STATUS` also carries M2's
# unsolicited telemetry, and a short window can catch a line split across two
# reads. Taking "the first ANGLE=" and "the first TARGET=" independently then
# pairs an ANGLE from one frame with a TARGET from another - measured: a bogus
# 119.56 deg error on a move that had actually landed 0.15 deg from target
# (M1 read ANGLE=114.17 against TARGET=474.02, and 474.02 mod 360 = 114.02).
# ALWAYS pull every field from ONE complete line, selected by motor prefix.
function Motor-Frame {
    param([string]$Text, [string]$Motor)
    $best = ""
    foreach ($line in @(($Text -split "`r?`n") | Where-Object { $_.Trim() -ne "" })) {
        $t = $line.Trim()
        if ($t.StartsWith($Motor + " STATE=") -and ($t -match "TARGET=") -and ($t -match "ANGLE=")) {
            $best = $t
        }
    }
    return $best
}

# Field out of a single frame produced by Motor-Frame.
function Frame-Field {
    param([string]$Frame, [string]$Name)
    if ($Frame -match ("{0}=(-?[0-9.]+)" -f [regex]::Escape($Name))) { return [double]$matches[1] }
    return -999.0
}

# STRING field out of a single frame produced by Motor-Frame (STATE, ERR, ...).
#
# Use this rather than St-Field whenever the motor is RUNNING. St-Field parses the
# LAST line of the window, and a running motor streams unsolicited telemetry, so
# the last line is routinely a HALF-RECEIVED frame - a bare prefix
# `M1 STATE=RUNNING ANGLE=.. VEL=..` cut off before ERR=. Measured in 6.5: STATE
# and VEL parsed fine while ERR came back "?" from the very same window. A partial
# read can only ever contain a PREFIX of a frame, so a complete frame is always
# available earlier in the text - Motor-Frame picks the last COMPLETE one.
function Frame-Str {
    param([string]$Frame, [string]$Name)
    if ($Frame -match ("{0}=(\S+)" -f [regex]::Escape($Name))) { return $matches[1] }
    return "?"
}

# ERR of a motor, read from ONE COMPLETE frame of a fresh STATUS reply.
# Prefer this over St-Field inside a poll loop. While a motor is RUNNING the board
# streams unsolicited telemetry, so the last line of a short window is a
# half-received frame and St-Field answers "?" - which any `-ne "0"` test reads as
# A FAULT (breaking a poll early on a healthy motor) and any `-eq "32"` test reads
# as no fault. Returns "?" only when no complete frame arrived at all.
function Get-Err {
    param([string]$Motor, [int]$WaitMs = 300)
    $f = Motor-Frame (Send-Cmd "$Motor STATUS" $WaitMs) $Motor
    return (Frame-Str $f "ERR")
}

# =====================================================================
# GROUP 2 - encoder / sensor layer
# =====================================================================
Write-Host ""
Write-Host "--- Group 2: encoder layer ---" -ForegroundColor Cyan
Stop-All

# 2.1 in READY, ANGLE stays LIVE (refreshed by the background task) while VEL
#     and EANGLE are FROZEN at their last RUNNING values - a trap that makes
#     "wait until VEL ~ 0" a condition that can never be met.
if (Want-Run "2.1") {
    Stop-All
    [void](Send-Cmd "M1 SPD 30" 300)
    Start-Sleep -Milliseconds 1500
    # Send-Raw, not Send-Cmd: a 200 ms blocking window on the STOP reply ate
    # most of the coast and made this step flake (1 of 2 runs saw only one
    # distinct ANGLE). Write and start polling immediately.
    Send-Raw "M1 STOP`r`n"
    # Sample DURING the coast, not after it. "ANGLE is live" cannot be shown on
    # a rotor that has already stopped - the previous version waited 250 ms and
    # then compared two readings 1.2 s apart, by which time the rotor was at
    # rest and ANGLE legitimately did not change, so it reported a false FAIL.
    # Two further traps: there is NO unsolicited telemetry in READY (the
    # periodic stream runs only while RUNNING), so every frame must be
    # SOLICITED with M1 STATUS; and a 2 s Read-For therefore returns nothing at
    # all. Poll hard - the coast from ~20 rad/s lasts only ~0.6 s.
    $ang = @(); $vel = @(); $ean = @()
    for ($i = 0; $i -lt 10; $i++) {
        $s = Send-Cmd "M1 STATUS" 70
        foreach ($line in @(All-Lines $s)) {
            if ($line -match "M1 STATE=") {
                $ang += (Get-Angle $line "ANGLE")
                $vel += (Get-Angle $line "VEL")
                $ean += (Get-Angle $line "EANGLE")
            }
        }
    }
    $na = @($ang | Sort-Object -Unique).Count
    $nv = @($vel | Sort-Object -Unique).Count
    $ne = @($ean | Sort-Object -Unique).Count
    $live   = ($ang.Count -ge 2) -and ($na -ge 2)   # ANGLE took >= 2 values
    $frozen = ($nv -eq 1) -and ($ne -eq 1)          # VEL and EANGLE did not
    Check "2.1" "READY: ANGLE live, VEL frozen" ($live -and $frozen) ("{0} frames: ANGLE distinct={1}, VEL distinct={2}, EANGLE distinct={3}" -f $ang.Count, $na, $nv, $ne)
    Stop-All
}

# 2.2 electrical angle tracks mechanical angle: over any move,
#     dEANGLE == pole_pairs * dANGLE (mod 360). Differential form needs no
#     knowledge of the calibration reference.
if (Want-Run "2.2") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    Start-Sleep -Milliseconds 400
    $s0 = Send-Cmd "M1 STATUS" 300
    $a0 = Get-Angle $s0 "ANGLE"; $e0 = Get-Angle $s0 "EANGLE"
    # EANGLE = 7*(mech - zero_angle), so the residual carries a CONSTANT
    # offset -7*zero_angle that is unknown a priori (measured ~49.8 deg here).
    # Test the SLOPE by requiring the residual to be the same at every step,
    # not by requiring it to be zero.
    $resids = @()
    foreach ($t in 20, 40, 60) {
        [void](Pos-Goto "M1" $t)
        $s = Send-Cmd "M1 STATUS" 300
        $da = Wrap180 ((Get-Angle $s "ANGLE")  - $a0)
        $de = Wrap180 ((Get-Angle $s "EANGLE") - $e0)
        $resids += (Wrap180 ($de - 7.0 * $da))
    }
    $rmin = ($resids | Measure-Object -Minimum).Minimum
    $rmax = ($resids | Measure-Object -Maximum).Maximum
    $spread = $rmax - $rmin
    Check "2.2" "EANGLE advances 7 x ANGLE" ($spread -lt 2.0) ("residual at 20/40/60 deg = {0:N2}/{1:N2}/{2:N2}, spread {3:N2} deg (constant offset = -7*zero_angle)" -f $resids[0], $resids[1], $resids[2], $spread)
    Stop-All
}

# 2.3 resolution floor: a step smaller than one encoder count (0.0879 deg)
#     must not move the rotor - the loop cannot resolve it.
#
#     The naive form (ZERO -> read a0 -> POS 0.04 -> read a1 -> |a1-a0| < 0.25)
#     is contaminated: STOP leaves the rotor FREEWHEELING (no holding torque),
#     so a0 is sampled while the rotor is free to be nudged by cogging, and that
#     drift is charged to the 0.04 deg command. Measured once: 0.270 deg - three
#     encoder counts of pure detent drift on a step that should do nothing.
#     Two changes make the step test only what it claims:
#       (1) energise the loop at the origin (POS 0) BEFORE sampling a0, so both
#           samples are taken while the loop is actively holding;
#       (2) add a CONTROL arm - command POS 0 again and re-read. Whatever the
#           rotor does with NO step commanded is the baseline jitter, and the
#           0.04 deg step only counts as "moving" if it exceeds that baseline
#           by more than one encoder count.
if (Want-Run "2.3") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    Start-Sleep -Milliseconds 400
    [void](Pos-Goto "M1" 0 3000 0.5)          # energise + settle AT the origin
    Start-Sleep -Milliseconds 600
    $a0 = Get-Angle (Send-Cmd "M1 STATUS" 300) "ANGLE"
    [void](Pos-Goto "M1" 0.04 3000 0.5)       # the sub-count step under test
    Start-Sleep -Milliseconds 600
    $a1 = Get-Angle (Send-Cmd "M1 STATUS" 300) "ANGLE"
    [void](Pos-Goto "M1" 0 3000 0.5)          # control: same target, no step
    Start-Sleep -Milliseconds 600
    $a2 = Get-Angle (Send-Cmd "M1 STATUS" 300) "ANGLE"
    $dStep = [math]::Abs((Wrap180 ($a1 - $a0)))
    $dCtrl = [math]::Abs((Wrap180 ($a2 - $a0)))
    # ONE function: does the 0.04 deg command move the rotor? That is a RELATIVE
    # question - the baseline holding jitter is a different function (step 4.7,
    # which measures 0.000 deg drift on a healthy hold). Measured: when both arms
    # drift, they drift by the SAME amount (0.880/0.880 and 0.360/0.360), which is
    # the proof that the step contributed nothing. An absolute bound here would
    # fail on a run where the plant merely jittered.
    #
    # The allowance is ONE encoder count plus half a count of quantization slack.
    # Both readings sit on a 0.0879 deg grid, so comparing them for exact equality
    # at one count is meaningless - a run measured 0.0900xx against an allowance
    # of exactly 0.09 and failed by five ten-thousandths of a degree. One count is
    # also the real floor: the loop resolves 1 count and a 0.04 deg target is only
    # 0.45 of one, so it sits at the very edge and DOES sometimes slip one count.
    # The 2.0 deg ceiling is not the assertion - it only stops a wild value from
    # sailing through the relative test.
    $ok = ($dStep -le ($dCtrl + 0.132)) -and ($dStep -lt 2.0)
    Check "2.3" "sub-count step does not move" $ok ("step moved {0:N3} deg vs control {1:N3} deg (no step); allowance = 1 count = 0.088" -f $dStep, $dCtrl)
    Stop-All
}

# =====================================================================
# GROUP 3 - velocity loop
# =====================================================================
Write-Host ""
Write-Host "--- Group 3: velocity loop ---" -ForegroundColor Cyan
Stop-All

# Tol = steady-state accuracy (mean vs target). Every bound below is set from
# MEASURED repeatability of this exact procedure (SPD -> 700 ms settle -> 3 s
# read), 6 clean reps per speed (probe_velrep.ps1), NOT from a frame count.
# The frame count is a trap: consecutive frames are strongly correlated because
# VEL wanders on a ~1-3 s timescale (probe_droop.ps1 showed a stationary
# +-1.5 rad/s wander at 20 rad/s), so a 3 s window holds only 1-3 independent
# samples and n~8 badly overstates the information. Measured worst |err|:
#   1 rad/s 0.19 | 5 rad/s 0.59 | 20 rad/s 0.54-0.94
# Bounds are that worst case with ~25% margin.
# The SPAN guard is deliberately loose: it exists only to catch a limit cycle
# (span ~90-110), not the normal static-friction ripple, which is ~2 rad/s at
# 5 rad/s on this plant. A tight span bound turns a healthy motor into a
# false FAIL.
$spdCases = @(
    # Pooled over 12 windows (6 reps + 7 suite runs): means 0.66..1.23, so the
    # worst |err| is 0.34 - measured AFTER 0.35 was first written, which left
    # the bound with 3% margin. 0.45 gives it ~30%.
    @{ Id = "3.1"; M = "M1"; T = 1.0;   Tol = 0.45 },
    # 6 reps: means 4.41..5.29, worst |err| 0.59 (a slight -0.14 undershoot,
    # not instability - sd within a window is 0.6-1.0). Was 0.25, which sat
    # inside the measured spread and flaked.
    @{ Id = "3.2"; M = "M1"; T = 5.0;   Tol = 0.75 },
    # 6 reps: means 19.46..20.35, avg 19.95 -> M1 is NOT droopy at 20 rad/s.
    # The clean-worst is 0.94 (one 19.06 window). The 1.21 seen before the
    # per-case integral clear was the PREVIOUS case's wound-up integral, not
    # this case: run A's 3.2 overshot so 3.3 read high, run B's 3.2 undershot
    # so 3.3 read even higher. With the confound gone, 1.10 covers 0.94.
    @{ Id = "3.3"; M = "M1"; T = 20.0;  Tol = 1.10 },
    # Same motor and speed as 3.2, mirrored; share its bound (6 reps of the
    # +5 case gave worst 0.59, and -5 is the same plant).
    @{ Id = "3.4"; M = "M1"; T = -5.0;  Tol = 0.75 },
    # 3.5 is looser than its M1 twin on purpose. M2 at 5 rad/s has a measured
    # sd of ~0.7-1.0 over 7-8 frames, so the standard error of the MEAN is
    # ~0.26 - but it is the window-to-window spread that matters, and three
    # repeats on a healthy motor gave 5.03 / 4.76 / 5.05 (worst |err| 0.24).
    @{ Id = "3.5"; M = "M2"; T = 5.0;   Tol = 0.45 },
    # M2 at 20 rad/s genuinely wanders more than M1: pooled means were
    # 18.87 / 18.96 / 19.21 / 19.21 / 20.39 / 21.17, with sd up to 1.40 and a
    # span of 5.84 inside a single window. Worst |err| 1.17, so 1.40 (~20%
    # margin) rather than 1.30, which three runs brought within 11%.
    @{ Id = "3.6"; M = "M2"; T = 20.0;  Tol = 1.40 },
    # 3.9 worst observed |err| 0.32 over three clean repeats.
    @{ Id = "3.9"; M = "M2"; T = -6.0;  Tol = 0.80 }
)
foreach ($c in $spdCases) {
    if (-not (Want-Run $c.Id)) { continue }
    Stop-All
    # Clear the PID integral before EVERY case, and pin the nominal gains.
    # STOPALL does NOT clear the integral (defect #5: only KP/KI/VEL do), so
    # without this each case starts from whatever the PREVIOUS case wound up,
    # and the measured mean depends on the order of the cases rather than on
    # the case itself - which is precisely what "each step tests one function"
    # forbids. It is measurable: run A's 3.2 overshot (+0.26, integral winds
    # negative) and its 3.3 then read 20.59; run B's 3.2 undershot (-0.15,
    # integral winds positive) and its 3.3 then read 21.21. Pinning the gains
    # here also stops group 3 from inheriting whatever 1.11-1.14 last set.
    # The integral still does its job WITHIN the case; it just cannot leak in.
    [void](Send-Cmd "$($c.M) VEL 0.05 0.5 0" 300); Start-Sleep -Milliseconds 150
    $r = Run-Spd $c.M $c.T 3000
    $st = Vel-Stats $r
    if ($st -eq $null) {
        Check $c.Id "SPD $($c.T) on $($c.M)" $false "no VEL telemetry"
    } else {
        $err = [math]::Abs($st.Mean - $c.T)
        # span guard: a limit cycle shows up as a huge span (90-110) even when
        # the mean looks right. The floor was 4.0, which is INSIDE the normal
        # stick-slip ripple at |T|=5 (measured spans 1.8 .. 4.2) and produced a
        # false FAIL on 3.4 at span 4.22. 8.0 still leaves the guard an order
        # of magnitude below the failure it exists to catch.
        $ok = ($err -lt $c.Tol) -and ($st.Span -lt [math]::Max(8.0, 0.6 * [math]::Abs($c.T)))
        Check $c.Id "SPD $($c.T) on $($c.M)" $ok ("mean={0:N2} err={1:N2} min={2:N2} max={3:N2} span={4:N2} sd={5:N2} n={6}" -f $st.Mean, $err, $st.Min, $st.Max, $st.Span, $st.Sd, $st.N)
    }
    Stop-All
}

# 3.7 low-speed stick-slip at 0.5 rad/s: the static-friction dead zone. Report
#     the span; a healthy motor still ripples here (~1.7), it is not a fault.
if (Want-Run "3.7") {
    Stop-All
    [void](Send-Cmd "M1 VEL 0.05 0.5 0" 300); Start-Sleep -Milliseconds 150   # clean integral (see the 3.x loop)
    $r = Run-Spd "M1" 0.5 3000
    $st = Vel-Stats $r
    # 0.45, not 0.30. Five independent runs at this target gave means
    # 0.47 / 0.31 / 0.47 / 0.44 / 0.80 - worst |err| 0.30 against a 0.30 bound,
    # i.e. the check was passing on the last bit of the float (the run that read
    # 0.80 passed only because the true mean was 0.7999...). At 0.5 rad/s the
    # VEL field is ALSO quantized: one encoder count per 8 ms window is
    # 0.19 rad/s, so the reading only takes values on a 0.19 grid (probe_droop
    # saw the 1 rad/s case cluster at 0.77/0.87/1.15/1.34). 0.45 is ~2.4 counts
    # - still an order of magnitude below a limit cycle (span 90-110).
    #
    # The 0.80 is NOT a tuning bug and must not be "fixed": at the bottom of the
    # range the rotor breaks static friction, jumps, and coasts, so the MEAN
    # genuinely runs +60% high. That stick-slip is the reason traj_vmin and FF
    # exist at all. This step asserts "it turns, in the right direction, without
    # running away" - not a tight mean, which the plant cannot deliver here.
    $ok = ($st -ne $null) -and ([math]::Abs($st.Mean - 0.5) -lt 0.45) -and ($st.Span -lt 4.0)
    Check "3.7" "0.5 rad/s in friction dead zone" $ok ("mean={0:N2} span={1:N2} sd={2:N2} (ripple here is stick-slip, not instability)" -f $st.Mean, $st.Span, $st.Sd)
    Stop-All
}

# 3.8 speed ceiling: the 6.6 V clamp, not the target, is what bounds the top
#     end (back-EMF). Informational - the number is the deliverable.
if (Want-Run "3.8") {
    Stop-All
    [void](Send-Cmd "M1 VEL 0.05 0.5 0" 300); Start-Sleep -Milliseconds 150   # clean integral (see the 3.x loop)
    $r = Run-Spd "M1" 200 2500
    $st = Vel-Stats $r
    # The claim is "the 6.6 V clamp, not the target, bounds the top end": any
    # value far below the 200 rad/s command proves it. The window is wide on
    # purpose - measured 50.2 .. 64.0 across runs, and 50.2 sat 0.2 above the
    # old 50.0 floor, i.e. that floor was one unlucky window from a false FAIL.
    $ok = ($st -ne $null) -and ($st.Mean -gt 40.0) -and ($st.Mean -lt 85.0)
    Check "3.8" "speed ceiling at 6.6 V clamp" $ok ("target 200 -> mean={0:N1} max={1:N1} rad/s (clamp-limited, not target-limited)" -f $st.Mean, $st.Max)
    Stop-All
}

# =====================================================================
# GROUP 4 - position loop
# =====================================================================
Write-Host ""
Write-Host "--- Group 4: position loop ---" -ForegroundColor Cyan

$posCases = @(
    @{ Id = "4.1"; M = "M1"; Deg = 90.0 },
    @{ Id = "4.2"; M = "M1"; Deg = 270.0 },
    @{ Id = "4.3"; M = "M1"; Deg = -90.0 },
    @{ Id = "4.4"; M = "M2"; Deg = 180.0 }
)
foreach ($c in $posCases) {
    if (-not (Want-Run $c.Id)) { continue }
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "$($c.M) ZERO" 300)
    Start-Sleep -Milliseconds 500
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $arrived = Pos-Goto $c.M $c.Deg
    $ms = $sw.ElapsedMilliseconds
    $s = Send-Cmd "$($c.M) STATUS" 300
    $err = [math]::Abs((Wrap180 ((Get-Angle $s "ANGLE") - (Get-Angle $s "TARGET"))))
    Check $c.Id "POS $($c.Deg) on $($c.M)" $arrived ("arrived={0} in {1} ms, final err {2:N3} deg" -f $arrived, $ms, $err)
    Stop-All
}

# 4.5 overshoot during a 90 deg move. Telemetry is 500 ms and the slew is
#     ~200 ms, but the pure-P loop has a 1-3 s creep tail, so the tail is what
#     this catches (a 2-4.5 deg overshoot is the documented signature).
if (Want-Run "4.5") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    Start-Sleep -Milliseconds 500
    Send-Raw "M1 POS 90`r`n"
    $r = Read-For 4000
    # Collect (target, angle) per line, then keep only frames whose TARGET is
    # the window's FINAL target (the most frequent value). Frames captured
    # before ZERO/POS landed still carry the old target, and pairing one of
    # those with a fresh ANGLE manufactures a huge phantom overshoot -
    # measured 155.92 deg on a move whose real overshoot is a few degrees.
    # ALIASING GATE: telemetry is 500 ms, so a rotor faster than 360 deg/s
    # (>6.28 rad/s) can pass the target by more than 180 deg between samples and
    # Wrap180 then reports a phantom "overshoot" of up to 180 - measured 69.70
    # and 155.92 on moves whose real overshoot is a few degrees. Keep only
    # frames whose own VEL proves the step since the last frame was < 172 deg.
    $pairs = @()
    foreach ($line in (All-Lines $r)) {
        if ($line -match "M1 STATE=" -and $line -match "TARGET=(-?[0-9.]+)" -and `
            $line -match "ANGLE=(-?[0-9.]+)" -and $line -match "VEL=(-?[0-9.]+)") {
            $t = [double]$matches[1]; $a = [double]$matches[2]; $v = [double]$matches[3]
            if ([math]::Abs($v) -lt 6.0) { $pairs += ,@($t, $a) }
        }
    }
    $ovs = 0.0; $n = 0
    if ($pairs.Count -gt 0) {
        $final = ($pairs | ForEach-Object { $_[0] } | Group-Object | Sort-Object Count -Descending)[0].Name
        $ft = [double]$final
        foreach ($pr in $pairs) {
            if ($pr[0] -eq $ft) {
                $d = Wrap180 ($pr[1] - $pr[0])
                if ($d -gt $ovs) { $ovs = $d }
                $n++
            }
        }
    }
    # Do NOT read this number as the overshoot spec. This step runs on the 500 ms
    # auto-telemetry, and a 90 deg step settles in ~1.5 s - so the whole move is
    # 3 frames and the overshoot excursion (~100-300 ms at the peak) is usually
    # missed entirely, which is why this reads 0.00. It is an UNDER-SAMPLED lower
    # bound, not a contradiction of the 2-4.5 deg figure: that one came from
    # Debug/posstep.ps1, which polls STATUS at 13 ms and is the only instrument
    # here fast enough to see the transient (at 13 ms you need 13800 deg/s to
    # alias, so it is not an aliasing artifact either). Use posstep.ps1 to
    # characterise overshoot; this step only proves the move terminates at the
    # target with the loop still alive.
    Check "4.5" "90 deg move overshoot" ($n -gt 0) ("peak overshoot {0:N2} deg over {1} frames at the final target (|VEL|<6 gated; 500 ms telemetry UNDER-SAMPLES this - posstep.ps1 at 13 ms is the real instrument)" -f $ovs, $n)
    Stop-All
}

# 4.6 retarget mid-slew keeps order and lands on the last target
if (Want-Run "4.6") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    Start-Sleep -Milliseconds 500
    [void](Send-Cmd "M1 POS 30" 60)
    Start-Sleep -Milliseconds 250
    [void](Send-Cmd "M1 POS 90" 60)
    Start-Sleep -Milliseconds 250
    [void](Send-Cmd "M1 POS 60" 60)
    Start-Sleep -Milliseconds 3500
    $f = Motor-Frame (Send-Cmd "M1 STATUS" 400) "M1"
    $err = [math]::Abs((Wrap180 ((Frame-Field $f "ANGLE") - (Frame-Field $f "TARGET"))))
    $errv = -1
    if ($f -match "ERR=(\d+)") { $errv = [int]$matches[1] }
    # 0.50, not 0.15. The old bound asserted the same tolerance Pos-Goto uses to
    # DECLARE arrival - but that predicate fires on ONE instantaneous frame
    # (|ANGLE-TARGET| < 0.15 at a 250 ms poll), it does not require the rotor to
    # settle. Between that frame and this read the pure-P loop lets stick-slip
    # pull the rotor off by a few counts, so a residual near the declaration
    # tolerance is expected, not a fault. Measured across runs: 0.030 / 0.060 /
    # 0.090 / 0.320 deg - and the 0.320 is 3.6 counts against traj_deadband=1.5,
    # which is the point: "POS arrived" means "was once inside 0.15 deg", not
    # "is holding at target". 0.50 deg is ~5.7 counts and still catches a real
    # position-loop failure (which shows up in degrees, not counts).
    Check "4.6" "30->90->60 retarget" (($err -lt 0.50) -and ($errv -eq 0)) ("final err {0:N3} deg, ERR={1} (arrival is declared on one frame, not a settle)" -f $err, $errv)
    Stop-All
}

# 4.7 HOLDING STABILITY at a fixed target. Split out of 2.3, which was
#     measuring two things at once: 2.3 asks "does a sub-count command move the
#     rotor?" (relative), this asks "once parked, does the rotor stay parked?"
#     (absolute). They are not the same question and they have different
#     answers: 2.3's control arm showed the rotor drifting 0.36 and 0.88 deg
#     with NO command at all. That drift is the thing measured here.
#     The loop is pure-P (POS_KI=0) with no damping, so a small residual hunt is
#     expected; the deliverable is the number, the bound only catches a runaway.
if (Want-Run "4.7") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    Start-Sleep -Milliseconds 500
    [void](Pos-Goto "M1" 90 7000 0.15)
    Start-Sleep -Milliseconds 400
    $angs = @(); $vels = @()
    for ($i = 0; $i -lt 8; $i++) {
        $f = Motor-Frame (Send-Cmd "M1 STATUS" 300) "M1"
        $a = Frame-Field $f "ANGLE"; $v = Frame-Field $f "VEL"
        if ($a -gt -900.0) { $angs += $a }
        if ($v -gt -900.0) { $vels += $v }
        Start-Sleep -Milliseconds 400
    }
    $drift = 0.0; $vspan = 0.0
    if ($angs.Count -gt 1) {
        $drift = (($angs | Measure-Object -Maximum).Maximum) - (($angs | Measure-Object -Minimum).Minimum)
    }
    if ($vels.Count -gt 1) {
        $vspan = (($vels | Measure-Object -Maximum).Maximum) - (($vels | Measure-Object -Minimum).Minimum)
    }
    $ok = ($angs.Count -ge 6) -and ($drift -lt 1.5)
    Check "4.7" "holding drift at a fixed target" $ok ("parked at 90 deg: drift {0:N3} deg peak-to-peak over {1} samples, VEL span {2:N2} (1 count = 0.088)" -f $drift, $angs.Count, $vspan)
    Stop-All
}

# =====================================================================
# GROUP 5 - dual-motor concurrency
# =====================================================================
Write-Host ""
Write-Host "--- Group 5: dual-motor ---" -ForegroundColor Cyan

# 5.1 both motors at the same speed simultaneously
#
# ★ The averaging window must be LONG. Measured 2026-10-03 (Debug/probe_51b.ps1,
#   which replays this step's exact protocol): at SPD 8 with both motors running,
#   the reported VEL fluctuates SLOWLY - a few seconds per swing - so a 3 s window
#   (n=8 frames) averages over less than one cycle and its mean lands anywhere in
#   6.4 ... 8.4. The full suite failed here with M1 mean=6.43 while the motor was
#   running perfectly. Widening the window is the fix, NOT widening the tolerance:
#     window 3 s  (n=8)  -> M1 spread 6.43 ... 8.37 across runs
#     window 6 s  (n=15) -> M1 spread 7.80 ... 8.02   (0.22)
#     window 12 s (n=27) -> M1 spread 8.07 ... 8.20   (0.12)
#   Over 12 s the true value is unambiguous: both motors hold 8.0-8.1 rad/s, so
#   the concurrency this step tests is genuinely fine. The first-half/second-half
#   split of a long window shows no drift (drag +-0.4, sign random), which is what
#   rules out a start-up ramp - a 2.5 s settle before a 3 s window did NOT help.
#   The precondition matters: replaying this protocol from a freewheel never
#   reproduced the low draw, but replaying it after step 4.7's `POS 90` hold does.
#   The 1500 ms settle covers the genuine start ramp (8 rad/s by ~1.4 s).
if (Want-Run "5.1") {
    Stop-All
    Send-Raw "M1 SPD 8`r`nM2 SPD 8`r`n"
    Start-Sleep -Milliseconds 1500
    $r = Read-For 6000
    $v1 = @(); $v2 = @()
    foreach ($line in (All-Lines $r)) {
        if ($line -match "M1 STATE=\S+.*VEL=(-?[0-9.]+)") { $v1 += [double]$matches[1] }
        if ($line -match "M2 STATE=\S+.*VEL=(-?[0-9.]+)") { $v2 += [double]$matches[1] }
    }
    $m1 = 0.0; $m2 = 0.0
    if ($v1.Count -gt 0) { $m1 = ($v1 | Measure-Object -Average).Average }
    if ($v2.Count -gt 0) { $m2 = ($v2 | Measure-Object -Average).Average }
    # Same rationale as 5.2: this step tests CONCURRENCY (do both motors hold
    # speed while sharing the bus?), not per-motor accuracy - 3.2 and 3.5
    # measure each motor on its own.
    $ok = ($v1.Count -gt 0) -and ($v2.Count -gt 0) -and
          ([math]::Abs($m1 - 8.0) -lt 1.2) -and ([math]::Abs($m2 - 8.0) -lt 1.2)
    Check "5.1" "both motors at 8 rad/s" $ok ("M1 mean={0:N2} (n={1})  M2 mean={2:N2} (n={3})" -f $m1, $v1.Count, $m2, $v2.Count)
    Stop-All
}

# 5.2 opposite directions at once - the two I2C buses and two PWM timers
#     must not interfere. Same long window as 5.1 and for the same reason: both
#     motors are running, so the reported VEL swings over a few seconds and a
#     short window's mean is a lottery draw rather than a measurement.
if (Want-Run "5.2") {
    Stop-All
    Send-Raw "M1 SPD 6`r`nM2 SPD -6`r`n"
    Start-Sleep -Milliseconds 1500
    $r = Read-For 6000
    $v1 = @(); $v2 = @()
    foreach ($line in (All-Lines $r)) {
        if ($line -match "M1 STATE=\S+.*VEL=(-?[0-9.]+)") { $v1 += [double]$matches[1] }
        if ($line -match "M2 STATE=\S+.*VEL=(-?[0-9.]+)") { $v2 += [double]$matches[1] }
    }
    $m1 = 0.0; $m2 = 0.0
    if ($v1.Count -gt 0) { $m1 = ($v1 | Measure-Object -Average).Average }
    if ($v2.Count -gt 0) { $m2 = ($v2 | Measure-Object -Average).Average }
    # Tolerance is loose on purpose: this step tests CONCURRENCY, not per-motor
    # accuracy (3.9 measures M2's negative direction on its own). A tight bound
    # here would report a direction asymmetry as an interference bug.
    $ok = ($v1.Count -gt 0) -and ($v2.Count -gt 0) -and
          ([math]::Abs($m1 - 6.0) -lt 1.5) -and ([math]::Abs($m2 + 6.0) -lt 1.5)
    Check "5.2" "opposite directions (M1 +6, M2 -6)" $ok ("M1 mean={0:N2}  M2 mean={1:N2}" -f $m1, $m2)
    Stop-All
}

# 5.3 stopping one motor must not disturb the other
if (Want-Run "5.3") {
    Stop-All
    Send-Raw "M1 SPD 8`r`nM2 SPD 8`r`n"
    Start-Sleep -Milliseconds 1200
    [void](Send-Cmd "M1 STOP" 300)
    Start-Sleep -Milliseconds 900
    $s1 = Send-Cmd "M1 STATUS" 300
    $st1 = "?"
    if ($s1 -match "STATE=(\S+)") { $st1 = $matches[1] }
    # M2's speed is judged on the MEAN of several frames, never on one. VEL is an
    # 8 ms fixed-window difference and M2 at a target of 8 reads 7.5-9.6 frame to
    # frame (measured 2026-10-03), so a single-frame +/-1.0 window fails on
    # sampling noise - it read 6.71 and failed a motor that was running perfectly.
    # Same lesson as 6.5; the one-frame form had just been lucky until now.
    # 12 frames over ~4.8 s, not 5 over ~2 s: the reported VEL also swings slowly
    # (see 5.1), so a 2 s mean is still a lottery draw - it takes a few seconds of
    # samples to land on the true value.
    # NOTE this is not the CLR that Stop-All now sends: probed separately, M2
    # after `M2 CLR` is at 6.14 within 227 ms and 9.40 by 655 ms, so wiping the
    # integral does not slow the build-up in any way that matters here.
    $vs = @(); $st2 = "?"
    for ($i = 0; $i -lt 12; $i++) {
        $f = Motor-Frame (Send-Cmd "M2 STATUS" 250) "M2"
        if ($f -match "M2 STATE=(\S+)") { $st2 = $matches[1] }
        $v = Frame-Field $f "VEL"
        if ($v -gt -900.0) { $vs += $v }
        Start-Sleep -Milliseconds 150
    }
    $v2 = 0.0
    if ($vs.Count -gt 0) { $v2 = ($vs | Measure-Object -Average).Average }
    $ok = ($st1 -eq "READY") -and ($st2 -eq "RUNNING") -and ([math]::Abs($v2 - 8.0) -lt 1.0)
    Check "5.3" "M1 STOP leaves M2 running" $ok ("M1={0} M2={1} VELmean={2:N2} over {3} frames (single frames span ~2)" -f $st1, $st2, $v2, $vs.Count)
    Stop-All
}

# 5.4 one motor in POSITION while the other runs in VELOCITY
if (Want-Run "5.4") {
    Stop-All
    Start-Sleep -Milliseconds 900
    [void](Send-Cmd "M1 ZERO" 300)
    [void](Send-Cmd "M2 SPD 8" 200)
    Start-Sleep -Milliseconds 300
    $ok = Pos-Goto "M1" 120
    # M2 is streaming, so pull M1's fields from ONE M1 frame (Motor-Frame).
    $f1 = Motor-Frame (Send-Cmd "M1 STATUS" 500) "M1"
    $e1 = [math]::Abs((Wrap180 ((Frame-Field $f1 "ANGLE") - (Frame-Field $f1 "TARGET"))))
    # M2's speed is the MEAN of several frames, not one. A single frame at SPD 8
    # spans ~2 rad/s (see 5.1/5.3), so a one-frame +/-1.5 window is roughly 1.5
    # sigma and fails a healthy motor every few runs - this step read 7.29 once
    # and 7.67 another time on motors that were both fine.
    $vs = @()
    for ($i = 0; $i -lt 8; $i++) {
        $v = Frame-Field (Motor-Frame (Send-Cmd "M2 STATUS" 250) "M2") "VEL"
        if ($v -gt -900.0) { $vs += $v }
        Start-Sleep -Milliseconds 150
    }
    $v2 = 0.0
    if ($vs.Count -gt 0) { $v2 = ($vs | Measure-Object -Average).Average }
    # e1 is a RE-READ after Pos-Goto already saw <0.15 deg, so it includes
    # post-arrival drift on a pure-P loop with no damping: measured 0.03 /
    # 0.12 / 0.30 across runs. The arrival claim is Pos-Goto's ($ok); this
    # bound only catches a genuine loss of position.
    Check "5.4" "M1 POSITION while M2 VELOCITY" ($ok -and ($e1 -lt 0.5) -and ([math]::Abs($v2 - 8.0) -lt 1.5)) ("M1 arrived={0} err {1:N3} deg (re-read), M2 VELmean {2:N2} over {3} frames" -f $ok, $e1, $v2, $vs.Count)
    Stop-All
}

# =====================================================================
# GROUP 6 - protection / fault handling
# =====================================================================
Write-Host ""
Write-Host "--- Group 6: protection ---" -ForegroundColor Cyan

# Poll ANGLE until two consecutive reads agree - i.e. the rotor has actually
# stopped - or the timeout expires (-1).
#
# Any test that leans on the stall guard needs this first. The guard wants a
# CLEAN 2 s window with no motion, and a rotor still coasting from an earlier
# step eats into that window. Measured 2026-10-03: after 6.1 holds `SPD 200`
# for 8 s the rotor is at ~67 rad/s, and 6.2 read ERR=0 at t=3.4 s because the
# coast plus the magnetic braking that follows `VOL 3` had not yet left a full
# quiet window behind it. The identical probe run with the rotor already parked
# tripped in **1917 ms**. Do not "fix" this by lengthening a fixed sleep: how
# long the coast takes depends on which step ran before.
function Wait-Still {
    param([string]$Motor, [int]$MaxMs = 6000)
    $prev = -999.0
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $MaxMs) {
        Start-Sleep -Milliseconds 300
        $a = -999.0
        if ((Send-Cmd "$Motor STATUS" 250) -match "ANGLE=(-?[0-9.]+)") { $a = [double]$matches[1] }
        if ($prev -gt -900.0) {
            $d = [math]::Abs($a - $prev)
            if ($d -gt 180.0) { $d = 360.0 - $d }
            if ($d -lt 0.05) { return $sw.ElapsedMilliseconds }
        }
        $prev = $a
    }
    return -1
}

# 6.1 the I2t thermal gate does NOT fire at sustained maximum output.
#     This is arithmetic, not a defect: i2t equilibrates at out^2 * DECAY, and
#     the output is hard-clamped to 0.55 * 12 V = 6.6 V, so the equilibrium is
#     6.6^2 * 4 = 174 against a 512 limit. motor_config.h:300 documents exactly
#     this. The step confirms it empirically and bounds the run at 8 s.
if (Want-Run "6.1") {
    Stop-All
    # precondition: start from a CLEAN latch. A stale ERR=32 from an earlier
    # step blocks SPD from ever leaving ERROR, and the step then "fails" for a
    # reason that has nothing to do with the thermal gate.
    [void](Send-Cmd "M1 CLR" 300); Start-Sleep -Milliseconds 300
    # Clear the PID integral as well. A poisoned integral left by an earlier
    # fault makes even a plain high-speed run trip the stall guard inside its
    # 2 s window, which would look like the thermal gate firing. KP is the only
    # command that clears the integral (see 6.4/6.5).
    [void](Send-Cmd "M1 KP 0.05" 300); Start-Sleep -Milliseconds 200
    [void](Send-Cmd "M1 SPD 200" 400)
    $tripped = $false; $maxErr = 0
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt 8000) {
        Start-Sleep -Milliseconds 900
        $s = Send-Cmd "M1 STATUS" 250
        if ($s -match "ERR=(\d+)") {
            $e = [int]$matches[1]
            if ($e -gt $maxErr) { $maxErr = $e }
            if ($e -ne 0) { $tripped = $true; break }
        }
    }
    Check "6.1" "I2t gate inert at 6.6V clamp" (-not $tripped) ("8 s at saturated output, ERR max={0} (equilibrium 174 vs limit 512 -> cannot trip)" -f $maxErr)
    Stop-All
}

# Read one field from the LAST telemetry line of a reply window (the freshest
# truth). NOTE the @() wrap at the call site: All-Lines does `return @(...)`,
# and PowerShell unrolls a one-element array on return, so the caller gets a
# bare String and indexing THAT with [Count-1] yields a Char, which has no
# .Trim(). This trap cost two probe runs while building Group 6.
#
# ★ DEPRECATED 2026-10-03 - no callers left, and do not add one. "The freshest
# line is the truth" only holds when the board is SILENT between commands. While
# a motor is RUNNING it streams unsolicited telemetry, so the window's last line
# is a HALF-RECEIVED frame - a bare prefix cut off mid-line - and this returns
# "?" for any field past the cut. That is how 6.5 read ERR=? off a window in
# which STATE and VEL parsed fine, and its `$err -ne "0"` assertion then PASSED
# on the "?" without ever reading the ERR bit. Use Frame-Str / Get-Err on a
# complete Motor-Frame instead.
function St-Field {
    param([string]$Text, [string]$Name)
    $ls = @(All-Lines $Text)
    if ($ls.Count -eq 0) { return "?" }
    $last = $ls[$ls.Count - 1]
    if ($last -match ("{0}=(\S+)" -f [regex]::Escape($Name))) { return $matches[1] }
    return "?"
}

# 6.2 stall guard (ERR=32) fires and latches.
#     Trigger: a STATIC open-loop vector. The guard's only arming condition is
#     |target_voltage| >= MOTOR_STALL_MIN_VOLT (1.5 V, motor_control.c:1333), and
#     it runs in the OPEN branch too (motor_control.c:1419), so `M1 VOL 3` arms
#     it and then holds the rotor still -> 2 s later it trips.
#     Measured 2026-10-03 by Debug/diag_stall.ps1: ERR 0 -> 32 at t=2027 ms,
#     ANGLE moved 0.08 deg, PWM 0.70,0.38,0.30 -> 0.00,0.00,0.00. The NEGATIVE
#     control came from the same run: `M1 VOL 0.5`, below the 1.5 V gate, held
#     4 s with ERR=0 in all 12 samples - so the trip really is voltage-gated and
#     not "open loop always reports a stall".
#     History: this step used to trip the guard with `M1 SPD nan`, which walked
#     past the arming gate because every comparison against NaN is false. That
#     input is rejected now (see 6.6), so the old method injected NO fault at all
#     and the step failed as "ERR 0 -> 0, STATE=READY".
#     Deliberately no Stop-All at the end: 6.3 inherits this latch, which is what
#     makes 6.3 a test of CLR rather than of an already-clean motor.
if (Want-Run "6.2") {
    Stop-All
    $still = Wait-Still "M1"
    $preE  = Get-Err "M1" 300
    [void](Send-Cmd "M1 VOL 3" 300)
    # POLL for the trip instead of sleeping a fixed 3 s. The guard fires 2 s after
    # the rotor actually stops, and how long it takes to stop depends on what the
    # previous step left spinning - a fixed sleep is a race that 6.2 lost once
    # (ERR 0 -> 0 at t=3.4 s, while the same probe on a parked rotor tripped at
    # 1917 ms). Polling also records the delay, which is the useful number.
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $err = "0"; $st = "?"
    while ($sw.ElapsedMilliseconds -lt 9000) {
        Start-Sleep -Milliseconds 250
        # One STATUS, one COMPLETE frame, both fields out of it (see Frame-Str):
        # a last-line read would take a half-received frame and could answer "?"
        # for ERR, which `-ne "0"` would then read as a trip.
        $f   = Motor-Frame (Send-Cmd "M1 STATUS" 250) "M1"
        $err = Frame-Str $f "ERR"
        $st  = Frame-Str $f "STATE"
        if ($err -ne "0") { break }
    }
    $tripMs = $sw.ElapsedMilliseconds
    $ok = ($preE -eq "0") -and ($err -eq "32") -and ($st -eq "ERROR")
    Check "6.2" "stall guard fires + latches" $ok ("ERR {0} -> {1}, STATE={2}, {3} ms after VOL 3 (rotor still after {4} ms; latch also blocks restart)" -f $preE, $err, $st, $tripMs, $still)
}

# 6.3 CLR clears the error latch. Also documents that the command is per-motor:
#     a bare `CLR` is "unknown command", it must be `M1 CLR`.
if (Want-Run "6.3") {
    $bad = Send-Cmd "CLR" 400
    $r   = Send-Cmd "M1 CLR" 400
    Start-Sleep -Milliseconds 300
    $c   = Send-Cmd "M1 STATUS" 400
    $err = Frame-Str (Motor-Frame $c "M1") "ERR"
    $ok = ($bad -match "ERR:") -and ($r -match "OK CLR") -and ($err -eq "0")
    Check "6.3" "CLR clears the ERR latch" $ok ("bare CLR='{0}'; M1 CLR -> ERR={1}" -f (First-Line $bad), $err)
}

# 6.4 (gap) CLR ALONE restores control. FIXED 2026-10-03, so this step now
#     asserts the opposite of what it used to.
#     History: `M1 CLR` answered "故障已清除，可重新发 SPD/OLTEST" but never
#     cleared the poisoned PID integral, so the very next SPD re-tripped the
#     stall guard inside its 2 s window - only a gain command (KP/KI/VEL, which
#     do clear it) brought the loop back. That was defect #5, the integral
#     outliving everything, surfacing on the fault path. CLR now clears the
#     integral, so the promise in its own reply is finally true.
#     Trigger is the static 3 V vector (see 6.2); the NaN injection this step
#     used before is rejected now.
if (Want-Run "6.4") {
    Stop-All
    [void](Wait-Still "M1")                  # clean 2 s window - see Wait-Still
    [void](Send-Cmd "M1 VOL 3" 300)
    # POLL for the trip rather than sleeping a fixed 3 s. 6.2 measured 4292 ms from
    # VOL 3 when the rotor had been coasting against 2027 ms with it already parked,
    # so a blind sleep wins or loses on which step ran before this one.
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $st1 = "?"; $e1 = "0"
    while ($sw.ElapsedMilliseconds -lt 9000) {
        Start-Sleep -Milliseconds 250
        $e1 = Get-Err "M1" 250
        if ($e1 -ne "0") { break }
    }
    $st1 = Frame-Str (Motor-Frame (Send-Cmd "M1 STATUS" 400) "M1") "STATE"
    $tripped = ($st1 -eq "ERROR") -and ($e1 -eq "32")
    # CLR, then STRAIGHT to SPD - no STOP and no KP. Both omissions ARE the
    # assertion: STOP clears the integral on its own and KP does too, so using
    # either would hide the very defect this step is about.
    [void](Send-Cmd "M1 CLR" 400)
    Start-Sleep -Milliseconds 300
    [void](Send-Cmd "M1 SPD 5" 400)
    # Judge the recovery on the MEAN of a telemetry window, not one frame. 6.4
    # measured VEL=3.07 against this same 3.0 bound on a motor that was running
    # fine - identical one-frame fragility to 5.3 and 6.5. The motor is RUNNING
    # here, so the window is full of unsolicited frames and the mean is stable.
    $vst = Vel-Stats (Read-For 1500)
    $f2  = Motor-Frame (Send-Cmd "M1 STATUS" 600) "M1"
    $st2 = Frame-Str  $f2 "STATE"
    $e2  = Frame-Str  $f2 "ERR"
    $v2  = Frame-Field $f2 "VEL"
    $mean = 0.0
    if ($vst -ne $null) { $mean = $vst.Mean }
    $restored = ($st2 -eq "RUNNING") -and ([math]::Abs($mean) -gt 3.0)
    Check "6.4" "(gap) CLR alone restores control" ($tripped -and $restored) ("tripped: STATE={0} ERR={1}; then CLR+SPD 5 with no STOP/KP -> STATE={2} VELmean={3:N2} (frame {4:N2}) ERR={5}" -f $st1, $e1, $st2, $mean, $v2, $e2)
    Stop-All
}

# 6.5 (gap) the recovered ERR bit is STICKY. STOP returns the motor to READY and
#     the next SPD runs it again, yet ERR=32 stays latched until an explicit CLR
#     - so telemetry reports a fault on a healthy, turning motor.
#     Trigger is the static 3 V vector (see 6.2); the NaN injection this step used
#     before is rejected now and would inject nothing.
#     The recovery deliberately does NOT use KP: Motor_Disable clears the PID
#     integral itself since 2026-10-03, so STOP alone is enough to let the next
#     SPD run. If this step ever needs a KP again, the integral is surviving the
#     disable path once more.
if (Want-Run "6.5") {
    Stop-All
    [void](Wait-Still "M1")                  # clean 2 s window - see Wait-Still
    [void](Send-Cmd "M1 VOL 3" 300)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tripErr = "0"
    while ($sw.ElapsedMilliseconds -lt 9000) {
        Start-Sleep -Milliseconds 250
        $tripErr = Get-Err "M1" 250
        if ($tripErr -ne "0") { break }
    }
    $tripped = ($tripErr -eq "32")
    [void](Send-Cmd "M1 STOP" 400)
    Start-Sleep -Milliseconds 600
    [void](Send-Cmd "M1 SPD 5" 400)
    Start-Sleep -Milliseconds 1200
    # Judge "is it actually running" on the MEAN of the telemetry window, not
    # on one instantaneous frame. VEL is an 8 ms fixed-window difference and at
    # SPD 5 its span is ~3.4-6.1, so a single sample can read 2.9 on a motor
    # that is turning perfectly (measured: run 2 read VEL=2.88 and 6.5 failed
    # for a reason that has nothing to do with ERR stickiness - the finding it
    # reports, ERR=32 on a RUNNING motor, was true in both runs).
    $vst = Vel-Stats (Read-For 1500)
    $s   = Send-Cmd "M1 STATUS" 600
    # Pull every field from ONE COMPLETE frame (see Frame-Str). The window's last
    # line is a half-received frame from the running motor's telemetry, and St-Field
    # read that: STATE and VEL parsed while ERR came back "?" - and the old
    # assertion `$err -ne "0"` PASSED on "?", so the step whose entire subject is
    # the ERR bit was green without ever reading it. Require exactly "32".
    $f2  = Motor-Frame $s "M1"
    $st  = Frame-Str  $f2 "STATE"
    $err = Frame-Str  $f2 "ERR"
    $vel = Frame-Field $f2 "VEL"
    $mean = 0.0
    if ($vst -ne $null) { $mean = $vst.Mean }
    $running = ($st -eq "RUNNING") -and ([math]::Abs($mean) -gt 3.0)
    Check "6.5" "(gap) ERR bit sticky after recovery" ($tripped -and $running -and ($err -eq "32")) ("tripped={0}; STATE={1} VELmean={2:N2} (frame {3:N2}) but ERR={4} (only CLR clears it)" -f $tripped, $st, $mean, $vel, $err)
    [void](Send-Cmd "M1 CLR" 300)
    Stop-All
}

# 6.6 (gap) NaN injection - FIXED 2026-10-03, so this step now asserts REJECTION.
#     History: this step used to assert the opposite (that `M1 SPD nan` was
#     accepted) and PASSED while the defect existed. newlib's strtof CONSUMES
#     "nan" and returns a real NaN, so the old full-token check let it through;
#     the NaN reached the modulator, which is why PWM read 0.00,0.00,0.00 instead
#     of the running zero vector 0.50,0.50,0.50, and why the stall guard tripped
#     2 s later (6.2). Every numeric argument now goes through the single parser
#     uart_parse_f (uart_debug.c), which rejects nan / +-inf / overflow
#     ("1e400") on all of SPD/POS/VOL/KP/KI/LIMIT/TRAJ/VEL/CALSPIN/ALIGN/OLTEST/
#     RAMP. The step is intentionally NOT renamed off "(gap)": it is still the
#     regression test for this class of input-validation gap.
if (Want-Run "6.6") {
    Stop-All
    [void](Send-Cmd "M1 CLR" 300); Start-Sleep -Milliseconds 300
    $reply = Send-Cmd "M1 SPD nan" 600
    $rejected = $reply -match "ERR:"   # "ERR:" - see the 1.25 note
    Stop-All
    Check "6.6" "(gap) nan rejected by SPD" $rejected ("reply='{0}' rejected={1}" -f (First-Line $reply), $rejected)
    [void](Send-Cmd "M1 CLR" 300)
    Stop-All
}

# =====================================================================
# Wrap up
# =====================================================================
Stop-All
Start-Sleep -Milliseconds 200

$csr1  = Get-U32 $ADDR_CSR
$tick1 = Get-U32 $ADDR_TICK

Write-Host ""
Write-Host "--- Group 0 (re-check) ---" -ForegroundColor Cyan
# 0.5 no reset happened during the whole run
if (Want-Run "0.5") {
    $noReset = ($csr1 -eq 0) -and ($tick1 -gt $tick0)
    Check "0.5" "no reset during run" $noReset ("RCC_CSR=0x{0:X8} uwTick {1} -> {2}" -f $csr1, $tick0, $tick1)
}

$script:Port.Close()

# ---------------------------------------------------------------- summary
Write-Host ""
Write-Host "===== SUMMARY =====" -ForegroundColor Cyan
$pass = 0; $fail = 0
foreach ($r in $script:Results) {
    if ($r.Result -eq "PASS") { $pass++ } else { $fail++ }
}
$script:Results | Format-Table -AutoSize Step, Result, Func | Out-String | Write-Host
Write-Host ("PASS {0}   FAIL {1}" -f $pass, $fail) -ForegroundColor $(if ($fail -eq 0) { "Green" } else { "Yellow" })
if ($fail -gt 0) {
    Write-Host "Failures:" -ForegroundColor Yellow
    foreach ($r in $script:Results) {
        if ($r.Result -eq "FAIL") {
            Write-Host ("  {0}  {1}" -f $r.Step, $r.Func) -ForegroundColor Yellow
            Write-Host ("      {0}" -f $r.Evidence) -ForegroundColor DarkGray
        }
    }
}
