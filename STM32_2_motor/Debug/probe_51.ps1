# probe_51.ps1 - why does suite step 5.1 read BOTH motors below the SPD 8 setpoint?
#
# 5.1 commands both motors at once, sleeps 900 ms, then averages VEL over a 3 s
# Read-For window and requires the mean within +/-1.2 of 8. Full-suite run
# 2026-10-03 gave M1 6.43 / M2 7.05 (FAIL M1); an earlier run gave 7.34 / 7.19.
# Both motors read LOW and the value moves run to run, which is the signature of
# the window containing part of the ACCELERATION RAMP rather than of a droop.
#
# This probe answers it directly: sample VEL every 250 ms from the instant the
# SPD commands go out, so the ramp is visible and the true steady state can be
# read off the tail instead of being inferred from a mean.
#
# ASCII only: PowerShell 5.1 on this zh-CN system reads .ps1 as GBK, so a UTF-8
# non-ASCII byte in the source decodes to garbage and breaks parsing.
param([string]$PortName = "COM4")

$ErrorActionPreference = "Stop"

$port = New-Object System.IO.Ports.SerialPort $PortName, 115200, `
    ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 50
$port.NewLine = "`n"
$port.Open()
Start-Sleep -Milliseconds 300
$port.DiscardInBuffer()

function Send-Cmd {
    param([string]$Cmd, [int]$WaitMs = 300)
    $port.DiscardInBuffer()
    $port.Write($Cmd + "`r`n")
    $ms = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $WaitMs) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $ms.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($ms.ToArray())
}

function Send-Raw {
    param([string]$Text)
    $port.DiscardInBuffer()
    $port.Write($Text)
}

function Read-For {
    param([int]$Ms = 500)
    $buf = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmp = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $Ms) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { $buf.Write($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($buf.ToArray())
}

# Last COMPLETE frame for one motor - never a half-received line.
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

function Fld {
    param([string]$Frame, [string]$Name)
    if ($Frame -match ("{0}=(-?[0-9.]+)" -f [regex]::Escape($Name))) { return [double]$matches[1] }
    return -999.0
}

# --- precondition: stopped, latches clear, integral wiped -------------------
[void](Send-Cmd "STOPALL" 400)
Start-Sleep -Milliseconds 300
[void](Send-Cmd "M1 CLR" 300)
[void](Send-Cmd "M2 CLR" 300)
Start-Sleep -Milliseconds 400
Write-Host "precondition: M1 $((Motor-Frame (Send-Cmd 'M1 STATUS' 300) 'M1'))"

# --- the ramp: both motors at SPD 8, sampled every 250 ms -------------------
Write-Host ""
Write-Host "--- ramp: 'M1 SPD 8' + 'M2 SPD 8' sent together, polled every 250 ms ---"
Send-Raw "M1 SPD 8`r`nM2 SPD 8`r`n"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.ElapsedMilliseconds -lt 4000) {
    Start-Sleep -Milliseconds 250
    $t  = $sw.ElapsedMilliseconds
    $f1 = Motor-Frame (Send-Cmd "M1 STATUS" 120) "M1"
    $f2 = Motor-Frame (Send-Cmd "M2 STATUS" 120) "M2"
    Write-Host ("  t={0,5} ms   M1 VEL={1,7:N2}  TGT={2,5:N2}   M2 VEL={3,7:N2}  TGT={4,5:N2}" -f `
        $t, (Fld $f1 "VEL"), (Fld $f1 "TARGET"), (Fld $f2 "VEL"), (Fld $f2 "TARGET"))
}

# --- steady state: exactly what 5.1 averages, but taken AFTER the ramp ------
Write-Host ""
Write-Host "--- steady state: Read-For 3000 taken now (t > 4 s) ---"
$r = Read-For 3000
$v1 = @(); $v2 = @()
foreach ($line in ($r -split "`r?`n")) {
    $t = $line.Trim()
    if ($t.StartsWith("M1 STATE=") -and ($t -match "TARGET=") -and ($t -match "ANGLE=")) {
        if ($t -match "VEL=(-?[0-9.]+)") { $v1 += [double]$matches[1] }
    }
    if ($t.StartsWith("M2 STATE=") -and ($t -match "TARGET=") -and ($t -match "ANGLE=")) {
        if ($t -match "VEL=(-?[0-9.]+)") { $v2 += [double]$matches[1] }
    }
}
$m1 = 0.0; $m2 = 0.0
if ($v1.Count -gt 0) { $m1 = ($v1 | Measure-Object -Average).Average }
if ($v2.Count -gt 0) { $m2 = ($v2 | Measure-Object -Average).Average }
$s1 = "n/a"; $s2 = "n/a"
if ($v1.Count -gt 1) { $s1 = "{0:N2}" -f (($v1 | Measure-Object -Maximum).Maximum - ($v1 | Measure-Object -Minimum).Minimum) }
if ($v2.Count -gt 1) { $s2 = "{0:N2}" -f (($v2 | Measure-Object -Maximum).Maximum - ($v2 | Measure-Object -Minimum).Minimum) }
Write-Host ("  M1 mean={0:N2} n={1} span={2}   M2 mean={3:N2} n={4} span={5}" -f $m1, $v1.Count, $s1, $m2, $v2.Count, $s2)
Write-Host ("  verdict: M1 off by {0:N2}, M2 off by {1:N2}  (5.1 allows 1.20)" -f [math]::Abs($m1 - 8.0), [math]::Abs($m2 - 8.0))

[void](Send-Cmd "STOPALL" 400)
Start-Sleep -Milliseconds 200
[void](Send-Cmd "M1 CLR" 300)
[void](Send-Cmd "M2 CLR" 300)
$port.Close()
Write-Host ""
Write-Host "done"
