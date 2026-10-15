# Reproduce step 5.4: M1 in POSITION while M2 runs in VELOCITY.
# The suite reports M1 err ~120 deg for a POS 120 command, i.e. M1 never moved.
# Print the full state at every stage, and also run the same move with M2 IDLE
# as an A/B, so the difference is attributable.
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
function Show([string]$tag, [string]$cmd, [int]$w = 400) {
    Write-Output ("  {0,-10}: {1}" -f $tag, (Last (C $cmd $w)))
}

# clean slate
[void](C "STOPALL" 500); Start-Sleep -Milliseconds 700
[void](C "M1 CLR" 300); [void](C "M2 CLR" 300)
[void](C "M1 KP 0.05" 300); [void](C "M2 KP 0.05" 300)
Start-Sleep -Milliseconds 400

Write-Output "== A/B 1: the exact 5.4 sequence (M2 running) =="
[void](C "M1 ZERO" 300)
[void](C "M2 SPD 8" 200)
Start-Sleep -Milliseconds 300
Show "beforePOS" "M1 STATUS"
[void](C "M1 POS 120" 300)
Start-Sleep -Milliseconds 2500
Show "after2.5s " "M1 STATUS"
Start-Sleep -Milliseconds 3000
Show "after5.5s " "M1 STATUS"
Show "M2        " "M2 STATUS"
[void](C "STOPALL" 500); Start-Sleep -Milliseconds 800
[void](C "M1 CLR" 300); [void](C "M1 KP 0.05" 300)
Start-Sleep -Milliseconds 400

Write-Output "== A/B 2: same move, M2 IDLE =="
[void](C "M1 ZERO" 300)
Start-Sleep -Milliseconds 500
Show "beforePOS" "M1 STATUS"
[void](C "M1 POS 120" 300)
Start-Sleep -Milliseconds 2500
Show "after2.5s " "M1 STATUS"
Start-Sleep -Milliseconds 3000
Show "after5.5s " "M1 STATUS"
[void](C "STOPALL" 500); Start-Sleep -Milliseconds 600
$p.Close()
