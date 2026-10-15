# One-off probe: does the velocity PID integral actually wind up at SPD 100,
# and does it survive STOP? Walks the path and prints state at each stage.
param([string]$PortName = "COM4")

$p = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 40; $p.WriteTimeout = 1000
$p.DtrEnable = $true; $p.RtsEnable = $true
$p.Open(); Start-Sleep -Milliseconds 200; $p.DiscardInBuffer()

function C([string]$c, [int]$w = 400) {
    $p.DiscardInBuffer(); $p.Write($c + "`r`n")
    $b = New-Object System.IO.MemoryStream
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $t = New-Object byte[] 4096
    while ($sw.ElapsedMilliseconds -lt $w) {
        try {
            $n = $p.Read($t, 0, $t.Length)
            if ($n -gt 0) { $b.Write($t, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { break }
    }
    $x = [System.Text.Encoding]::UTF8.GetString($b.ToArray())
    return (($x -split "`r?`n") | Where-Object { $_.Trim() -ne "" } | Select-Object -First 2) -join " || "
}

Write-Output ("1 STOPALL : " + (C "STOPALL" 500))
Write-Output ("2 state   : " + (C "M1 STATUS" 400))
Write-Output ("3 SPD 100 : " + (C "M1 SPD 100" 500))
Start-Sleep -Milliseconds 2500
Write-Output ("4 running : " + (C "M1 STATUS" 400))
Write-Output ("5 LIMIT q : " + (C "M1 LIMIT" 400))
Write-Output ("6 STOP    : " + (C "M1 STOP" 400))

# Poll ANGLE until two consecutive reads agree: VEL is FROZEN in READY, so
# ANGLE is the only at-rest witness. The coast from ~66 rad/s takes seconds.
$prev = -999.0
$rest = $false
for ($i = 0; $i -lt 25; $i++) {
    Start-Sleep -Milliseconds 400
    $s = C "M1 STATUS" 300
    $a = -999.0
    if ($s -match "ANGLE=(-?[0-9.]+)") { $a = [double]$matches[1] }
    if ($prev -gt -900.0) {
        $d = [math]::Abs($a - $prev)
        if ($d -gt 180.0) { $d = 360.0 - $d }
        if ($d -lt 0.05) { $rest = $true; Write-Output ("7 at rest : after {0:N1} s, ANGLE={1}" -f (($i + 1) * 0.4), $a); break }
    }
    $prev = $a
}
if (-not $rest) { Write-Output "7 at rest : NEVER SETTLED in 10 s" }

Write-Output ("8 SPD 0   : " + (C "M1 SPD 0" 400))
Start-Sleep -Milliseconds 1500
Write-Output ("9 after   : " + (C "M1 STATUS" 400))
$p.Close()
