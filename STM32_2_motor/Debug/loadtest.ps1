# Is the reset loop driven by motor current?  IDLE / SPIN / IDLE, same port, same length.
#   .\loadtest.ps1
#
# A boot runs the calibration, which energises and turns BOTH motors. So "the board
# resets, so the motors move, so the board resets" is circular and proves nothing.
# Breaking the circle needs a controlled A/B: hold everything constant (same open
# port, same duration) and change ONLY whether the motors are drawing current.
#
# Phase 1 IDLE -> phase 2 SPIN (M1 SPD) -> phase 3 IDLE again. If resets follow the
# SPIN phase and stop in the IDLE phases, the reset source is the supply sagging
# under motor current (a POR/brownout), not a loose signal pin -- a bad contact does
# not care whether the motors are running.
#
# The motor is run slowly (a few rad/s) and stopped afterwards; this is the same
# gentle motion the boot calibration already performs.
# Keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as ANSI/GBK).
param(
    [string] $PortName = "COM4",
    [int]    $Secs     = 8,
    [double] $Vel      = 3.0
)

$cli = "K:\STM32CubeIDE\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
$CSR = "0x40021024"

function Read-U32([string]$addr) {
    $out = & $cli -c port=SWD mode=hotplug -r32 $addr 4 2>&1
    foreach ($line in $out) {
        if ($line -match '^\s*0x[0-9A-Fa-f]{8}\s*:') {
            $body = ($line -split ':', 2)[1]
            foreach ($tok in ($body -split '\s+')) {
                if ($tok -match '^[0-9A-Fa-f]{8}$') { return [Convert]::ToUInt32($tok, 16) }
            }
        }
    }
    return -1
}
function Clear-Flags { [void](& $cli -c port=SWD mode=hotplug -w32 $CSR 0x01000000 2>&1) }
function Describe([uint32]$csr) {
    if ($csr -lt 0) { return "read failed/garbage" }
    if ($csr -eq 0) { return "no reset" }
    $n = @()
    if ($csr -band 0x08000000) { $n += "POR(brownout)" }
    if ($csr -band 0x04000000) { $n += "PIN(NRST)" }
    if ($csr -band 0x10000000) { $n += "SOFT" }
    if ($csr -band 0x20000000) { $n += "IWDG" }
    if ($csr -band 0x40000000) { $n += "WWDG" }
    if ($csr -band 0x80000000) { $n += "LPWR" }
    if ($n.Count -eq 0) { return ("unknown 0x{0:X8}" -f $csr) }
    return ($n -join "+")
}

$port = New-Object System.IO.Ports.SerialPort -ArgumentList @(
    $PortName, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$port.ReadTimeout  = 100
$port.WriteTimeout = 1000
$port.DtrEnable    = $false
$port.RtsEnable    = $false
try { $port.Open() } catch { Write-Output ("OPEN FAILED: " + $_.Exception.Message); exit 1 }

# Listen for Secs, count boot banners, optionally send one command first.
function Phase([string]$name, [string]$cmd) {
    Clear-Flags
    Start-Sleep -Milliseconds 250
    if ($cmd) { $port.Write($cmd + "`r`n") }
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $buf = New-Object System.Text.StringBuilder
    $tmp = New-Object char[] 512
    while ($sw.ElapsedMilliseconds -lt ($Secs * 1000)) {
        try {
            $n = $port.Read($tmp, 0, $tmp.Length)
            if ($n -gt 0) { [void]$buf.Append($tmp, 0, $n) }
        } catch [System.TimeoutException] {
        } catch { }
    }
    $boots = ([regex]::Matches($buf.ToString(), 'STM32_2_motor ready')).Count
    $csr   = Read-U32 $CSR
    # Write-Host, not Write-Output: anything on the output stream would be added to
    # this function's return value and turn $b1/$b2/$b3 into arrays.
    Write-Host ("{0,-22} cmd={1,-10} boots={2,2}   RCC_CSR=0x{3:X8} -> {4}" -f `
        $name, ($(if ($cmd) { $cmd } else { "(none)" })), $boots, $csr, (Describe $csr))
    return $boots
}

Write-Output ("IDLE / SPIN / IDLE, {0} s each, M1 at {1} rad/s" -f $Secs, $Vel)
Write-Output ""

$b1 = Phase "phase 1: motors idle" ""
$b2 = Phase "phase 2: M1 spinning" ("M1 SPD " + $Vel)
$port.Write("STOPALL`r`n")
Start-Sleep -Milliseconds 500
$b3 = Phase "phase 3: motors idle" ""

$port.Close()
Write-Output ""
Write-Output ("banners: idle={0}  spin={1}  idle={2}" -f $b1, $b2, $b3)
if ($b2 -gt $b1 -and $b2 -gt $b3) {
    Write-Output "=> resets track motor current: the supply is sagging (POR), not a signal pin."
} elseif ($b1 + $b2 + $b3 -eq 0) {
    Write-Output "=> quiet window; the fault is not reproducing right now, re-run later."
} else {
    Write-Output "=> resets are not correlated with motor current -- suspect a loose pin/reset line."
}
