param(
    [double] $Speed1   = 8.0,
    [double] $Speed2   = 15.0,
    [string] $PortName = "COM4"
)

# Dual-motor simultaneous test: drive both at DIFFERENT targets and confirm each
# tracks its own target without disturbing the other.
#
# Note the command pattern below is deliberately the hostile one: M2's command is
# sent while M1 is ALREADY running and emitting telemetry. That is the case that
# used to garble ("M2 KP 0.1; M2 SPD 5" produced help text and zero samples), so
# each command is sent separately with a gap rather than chained with ';'.
#
# ASCII only: PowerShell 5.1 on a zh-CN system reads .ps1 as GBK.

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $true
$port.RtsEnable    = $true
try { $port.Open() } catch { Write-Output "OPEN FAILED: $($_.Exception.Message)"; exit 1 }
Start-Sleep -Milliseconds 80
$port.DiscardInBuffer()

function Read-Ms([int]$ms) {
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

function Show([string[]]$lines, [string]$tag) {
    foreach ($l in $lines) {
        if ($l -match "STATE=(RUNNING|READY)") { Write-Output ("  [{0}] {1}" -f $tag, $l.Trim()) }
    }
}

$port.Write("STOPALL`r`n")
Start-Sleep -Milliseconds 600
[void](Read-Ms 400)

Write-Output "--- starting M1 at $Speed1 rad/s ---"
$port.Write("M1 SPD $Speed1`r`n")
Start-Sleep -Milliseconds 2500

Write-Output "--- now starting M2 at $Speed2 rad/s WHILE M1 is running ---"
$port.Write("M2 SPD $Speed2`r`n")
Start-Sleep -Milliseconds 2500

Write-Output "--- both running, sampling 6 times over ~3 s ---"
for ($i = 1; $i -le 6; $i++) {
    $port.Write("STATUS`r`n")
    Show (Read-Ms 450) "t=$i"
    Start-Sleep -Milliseconds 100
}

Write-Output "--- re-targeting M1 to -$Speed1 (M2 untouched, must not move) ---"
$port.Write("M1 SPD -$Speed1`r`n")
Start-Sleep -Milliseconds 3000
for ($i = 1; $i -le 4; $i++) {
    $port.Write("STATUS`r`n")
    Show (Read-Ms 450) "t=$i"
    Start-Sleep -Milliseconds 100
}

Write-Output "--- STOPALL ---"
$port.Write("STOPALL`r`n")
Start-Sleep -Milliseconds 700
Show (Read-Ms 500) "after"
$port.Close()
