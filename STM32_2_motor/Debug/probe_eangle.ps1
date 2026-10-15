# Probe: what is the real relationship between ANGLE and EANGLE?
# Step the rotor in 20 deg increments and print both, so the ratio can be
# read off the data instead of assumed.
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
        try { $n = $p.Read($t, 0, $t.Length); if ($n -gt 0) { $b.Write($t, 0, $n) } }
        catch [System.TimeoutException] {} catch { break }
    }
    return [System.Text.Encoding]::UTF8.GetString($b.ToArray())
}
function F([string]$s, [string]$f, [double]$d = -999.0) {
    if ($s -match "$f=(-?[0-9.]+)") { return [double]$matches[1] }
    return $d
}

[void](C "STOPALL" 500)
Start-Sleep -Milliseconds 600
[void](C "M1 SPD 5" 400); Start-Sleep -Milliseconds 1500; [void](C "M1 STOP" 400)
Start-Sleep -Milliseconds 1500
[void](C "M1 ZERO" 400)
Start-Sleep -Milliseconds 500

$s = C "M1 STATUS" 400
$a0 = F $s "ANGLE"; $e0 = F $s "EANGLE"
Write-Output ("ref   ANGLE={0,8:N2}  EANGLE={1,8:N2}" -f $a0, $e0)

foreach ($tgt in 20, 40, 60, 80, 100) {
    [void](C "M1 POS $tgt" 300)
    Start-Sleep -Milliseconds 2500
    $s = C "M1 STATUS" 400
    $a = F $s "ANGLE"; $e = F $s "EANGLE"
    $da = $a - $a0; $de = $e - $e0
    $ratio = "n/a"
    if ([math]::Abs($da) -gt 0.5) { $ratio = ("{0:N3}" -f ($de / $da)) }
    Write-Output ("POS {0,4}  ANGLE={1,8:N2}  EANGLE={2,8:N2}   dA={3,7:N2}  dE={4,8:N2}  dE/dA={5}" -f $tgt, $a, $e, $da, $de, $ratio)
}
[void](C "M1 STOP" 400)
$p.Close()
