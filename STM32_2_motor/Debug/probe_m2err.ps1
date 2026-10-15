# M2 has been sitting in STATE=ERROR with ERR=0 since the brownout in the
# previous full run. CLR clears ERR BITS, so if there is no bit set there is
# nothing for it to clear and the state cannot be exited that way. Find what,
# if anything, recovers it - and whether it is reachable at all.
param([string]$PortName = "COM4")

$p = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 40; $p.WriteTimeout = 1000
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open(); Start-Sleep -Milliseconds 200; $p.DiscardInBuffer()

function C([string]$c, [int]$w = 500) {
    $p.DiscardInBuffer(); $p.Write($c + "`r`n")
    $b = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $t = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $w) {
        try { $n = $p.Read($t, 0, $t.Length); if ($n -gt 0) { $b.Write($t, 0, $n) } }
        catch [System.TimeoutException] {} catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($b.ToArray())
}
function Last([string]$s) {
    $lines = @(($s -split "`r?`n") | Where-Object { $_.Trim() -ne "" })
    if ($lines.Count -eq 0) { return "<none>" }
    return $lines[$lines.Count - 1].Trim()
}
function Show([string]$tag, [string]$cmd = "M2 STATUS", [int]$w = 400) {
    Write-Output ("  {0,-12}: {1}" -f $tag, (Last (C $cmd $w)))
}

Write-Output "== initial =="
Show "M2"
Show "M1"

Write-Output "== what does the ERROR state say for itself? =="
Show "HELP-ERR" "M2"
Write-Output ("  raw M2 STATUS window: " + (C "M2 STATUS" 600).Replace("`r", "").Replace("`n", " | "))

Write-Output "== recovery attempts, one at a time =="
Show "before"      "M2 STATUS"
[void](C "M2 CLR" 400);        Start-Sleep -Milliseconds 400; Show "after M2CLR"
[void](C "M2 STOP" 400);       Start-Sleep -Milliseconds 400; Show "after M2STOP"
[void](C "M2 KP 0.05" 400);    Start-Sleep -Milliseconds 400; Show "after M2KP"
[void](C "STOPALL" 400);       Start-Sleep -Milliseconds 600; Show "after STOPALL"
[void](C "CLR" 400);           Start-Sleep -Milliseconds 400; Show "after bareCLR"
[void](C "M2 SPD 0" 400);      Start-Sleep -Milliseconds 800; Show "after SPD0"
[void](C "M2 START" 400);      Start-Sleep -Milliseconds 800; Show "after START"
[void](C "M2 OLTEST 1" 400);   Start-Sleep -Milliseconds 800; Show "after OLTEST"
[void](C "STOPALL" 400);       Start-Sleep -Milliseconds 600

Write-Output "== can it actually turn? =="
[void](C "M2 CLR" 300); [void](C "M2 KP 0.05" 300); Start-Sleep -Milliseconds 300
[void](C "M2 SPD 5" 400); Start-Sleep -Milliseconds 2500
Show "spd5"
[void](C "STOPALL" 400); Start-Sleep -Milliseconds 500
$p.Close()
